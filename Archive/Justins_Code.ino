/**
 * based on https://learn.openenergymonitor.org
 * todo speed optimisation
 * todo millis() used alot - check for rollover (49 days)
 * todo test overload conditions
 * todo write unit tests
 * todo extensive use of floats - memory issues can be optimised
 */
#include <Arduino.h>
#include <EmonLib.h>
#include <HardwareSerial.h>
#include "EnergyBucket.h"
#include "PowerBucket.h"

const byte version = 1;
// Version 1.0 - Initial implementation.

enum debugLevel : int {
	off = 0, low = 1, med, high, nonewline, newline
};
int debug = high;

/**
 * pin configurations
 */
int analogPinIslandOutVoltage = A0;     // SI out voltage analogue pin
int analogPinIslandOutCurrent = A1;     // SI out clamp connected to analog pin
int analogPinSunnyBoyOut = A2;     // SB clamp connected to analog pin
int analogPinCU1 = A3;         // CU1 clamp connected to analog pin
int analogPinGenerator = A4;        // generator clamp connected to analog pin
int analogPinBattery = A5;     // battery clamp connected to analog pin
int digitalSOCpin = 13;     // SOC relay connected to digital pin

/**
 * hardware serial communication
 * hardware serial ports are not explicitly configured - defaults are used and variables assigned for coe information
 * not used - can be used for logging
 * int serial1TX = 18;
 * int serial1RX = 19;
 */
HardwareSerial *ImmersionSerial = &Serial3;
HardwareSerial *EvseSerial = &Serial2;
int pinImmersionTX = 14;
int pinImmersionRX = 15;
int pinEVSETX = 16;
int pinEVSERX = 17;
const char charStartMarker = '<'; //<
const char charEndMarker = '>'; //>

const byte numBytes = 8;
char receivedChar[numBytes];

//int numBytes = 6;
//char receivedChar[numBytes];
byte numReceived = 0;
boolean newData = false;
boolean newInstruction = false;

/**
 * variables for serial communication
 */
int softwareBaudRate = 9600;
int delayAfterSendingMessage = 50;
float countOFAllBufferResets = 0.00;
float countOfAllMessagesCorreclyBounded = 0.00;
float bufferResetMax = 400000000.00; //4294967040

/**
 * callibration variables
 * https://learn.openenergymonitor.org/electricity-monitoring/ctac/calibration
 * mascot Voltage Calibration Coefficient 234.26 https://learn.openenergymonitor.org/electricity-monitoring/ctac/calibration
 * float Vcal = 268.97; // (230V x 13) / (9V x 1.2) = 276.9 Calibration for UK AC-AC adapter 77DB-06-09
 * float Vcal = 234.26; // (230V x 13) / (9V x 1.2) = 276.9 Calibration for UK AC-AC adapter 77DB-06-09
 * byte Vrms = 230; // Vrms for apparent power readings (when no AC-AC voltage sample is present)
 *
 */
float Vcal = 236.09; // (230V x 13) / (9V x 1.2) = 276.9 Calibration for UK AC-AC adapter 77DB-06-09
const float phase_shift = 1.700; //todo callibration seems to have little effect
double SIcurCal = 58;
double SBcurCal = 58;
double Cu1CurCal = 58;
double GenCurCal = 58;
float batteryCalibration = 160;
int numberOfSamplesForBatteryCurrent = 400;
int delayAfterEachBatteryReading = 1;

//todo this needs callibrating in situ
//float Vref = 4.952; //arduino voltage ref
float Vref = 5.020; //arduino voltage ref

// calcVI Calculate all. No.of half wavelengths (crossings), time-out
int halfWavelengths_Power = 20; //40 gives a more accurate reading
unsigned int halfWavelengths_Freq = 40; //80 gives a more accurate reading
unsigned int timeout = 2000;

/**
 * external inputs, controls and loads
 */
byte CT_count = 0; //todo was used to keep track of transformers connected - should be deprecated
boolean CTSI, CTSB, CTCU1, CTGen, ACAC; //current transformers

/**
 * EVSE status information
 */
enum EVSE_initialised : int {
	EVSEnotInitialised = 0,
	EVSEoff = 1,
	EVSEwaiting = 2,
	EVSEready = 3,
	EVSEcharging = 4,
	EVSEchargeError = 5,
	EVSEcommError = 6,
	SerComError = 7,
};

int EVSE_status = EVSEnotInitialised;
int EVSEErrorCount = 0;
boolean evseStarted = false;
boolean evseFinished = false;

enum logType : int {
	Information = 0, Warning = 1, Error
};

/**
 * Bucket classes have not been implemented -these are placeholders/interfaces
 */
//PowerBucket powerBucket;
//EnergyBucket energyBucket;
/**
 * current and power variables
 */
const float sunnyIslandShortTermMaxPower = 3500.00;
const float sunnyIslandUltimateMaxPower = 4500.00;
volatile float sunnyIslandCurrent;
volatile float sunnyBoyCurrent;
volatile float consumerUnit1Current;
volatile float generatorCurrent;
volatile float sunnyIslandPower;
volatile float sunnyBoyPower;
volatile float consumerUnit1Power;
volatile float generatorPower;
volatile float batteryCurrent;
int stateOfCharge = LOW;

volatile float currentFrequency;
const float nominalFrequency = 50.00;

const float fACstartDelta = 1.00;
const float fACstartDeltaLimit = 2.00;

const float availablePVPower = 8000;
volatile float freqShift;
volatile float excessPVPower;
volatile boolean latentPVPowerAvailable;
volatile boolean systemOverload = false;
float thresholdExcessPVpower = 1000;

enum chargingStates : int {
	chargingStateNotInitialised = 0,
	charging = 1,
	idle = 2,
	discharging = 3,
	generator = 4,
	overload = 5,
};
int chargingState = chargingStateNotInitialised;

/**
 * Thresholds and variables for time functions called
 * loads only active when SoC levels are above threshold levels (See sunny island configuration)
 * batCurrentThresholdImmersion_Off could be the charging current once Soc threshold reached
 *
 */
float thresholdBatCurrent_High = 10.00;
float thresholdBatCurrent_Low = -5.00;
float thresholdGeneratorPower = 100.00; //CT sensor reads +ve power for generator set at 100W

float batCurrentThresholdImmersion_Off = -15.00;
float batCurrentThresholdImmersion_ShortTermLimit = 15.00;
float batCurrentThresholdImmersion_On = -35.00;
boolean flipping = false;

float SBPowerThresholdEvse_Start = -600.00;
float batCurrentThresholdEvse_Start = -10.00;
float batCurrentThresholdEvse_Start_Reduce = 30.00;

float batCurrentThresholdEvse_Off = 80.00;
float batCurrentThresholdEvse_Reduce = 6.00;
float batCurrentThresholdEvse_ReduceQuickly = 35.00;
float batCurrentThresholdEvse_Danger = 80.00;
float batCurrentThresholdEvse_Increase = -7.00;

/**
 * variables for timers
 */
unsigned long timeStartCalled = 0;
int startUpTime = 120000; //time for evse to respond to start resistance and actually start up

unsigned long storedTime = 0;
unsigned long previousLoopStartTime = 0;
unsigned long durationOfLastLoop = 0;

unsigned long previousTimeHeatIncreased = 0;
unsigned long storedTimeHeatIncreased = 0;
unsigned long previousTimeHeatDecreased = 0;
unsigned long storedTimeHeatDecreased = 0;
unsigned long shortTermHeatLimitPeriod = 30000;
unsigned long storedTimeForShortTermHeatLimit = 0;
unsigned long previousTimeForShortTermHeatLimit = 0;

int flippingCountVariable = 2;

unsigned long previousTimeEvseDecreased = 0;
unsigned long storedTimeEvseDecreased = 0;
unsigned long previousTimeEvseIncreased = 0;
unsigned long storedTimeEvseIncreased = 0;
unsigned long countOfSuccessiveCallsToEvseDecrease = 0;
unsigned long shortTermEvseLimitPeriod = 30000;
unsigned long storedTimeForShortTermEvseLimit = 0;
unsigned long previousTimeForShortTermEvseLimit = 0;

EnergyMonitor SI_Emonitor, SB_Emonitor, CU1_Emonitor, Gen_Emonitor,
		Bat_Emonitor; // Create instances
		/**
		 * todo
		 * export data via shield or hardware serial
		 * sunny island current/power
		 * SOC high/low
		 * battery current
		 */
boolean persistData() {
	//
	return false;
}

/**
 *  determines RMS value of voltage input
 *  178 samples takes about 20ms
 */
double calc_rms(int pin, int samples) {
	unsigned long sum = 0;
	for (int i = 0; i < samples; i++) {
		int raw = (analogRead(0) - 512);
		sum += (unsigned long) raw * raw;
	}
	double rms = sqrt((double) sum / samples);
	return rms;
}

/**
 * Calculate if there is an AC-AC adapter on analog input analogPinIslandOutVoltage
 * to do this does not reliable determine if power supply is available!
 */
boolean checkForACvoltageInput() {
	double vrms = calc_rms(analogPinIslandOutVoltage, 1780)
			* (Vcal * (Vref / 1024.000));
	if (vrms > 90) {
		ACAC = 1;
		if (debug >= high) {
			Serial.print("vrms: ");
			Serial.println(vrms);
		}
	} else
		ACAC = 0;
	return ACAC = 1;
}

void initialiseMonitors() {
	checkForACvoltageInput();
	if (ACAC) {
		Serial.println("AC-AC detected - Real Power measure enabled");
		Serial.println("assuming pwr from AC-AC (jumper closed)");
		Serial.print("Vcal: ");
		Serial.println(Vcal);
		Serial.print("Phase Shift: ");
		Serial.println(phase_shift);
	} else {
		Serial.println("AC-AC NOT detected - Apparent Pwr measure enabled");
		Serial.print("Assuming VRMS: ");
		//Serial.print(Vrms);
		Serial.println("V");
		Serial.println(
				"Assuming power from batt / 5V USB - power save enabled");
	}
	if (ACAC) {
		SI_Emonitor.voltage(analogPinIslandOutVoltage, Vcal, phase_shift); // ADC pin, Calibration, phase_shift
		SB_Emonitor.voltage(analogPinIslandOutVoltage, Vcal, phase_shift); // ADC pin, Calibration, phase_shift
		CU1_Emonitor.voltage(analogPinIslandOutVoltage, Vcal, phase_shift); // ADC pin, Calibration, phase_shift
		Gen_Emonitor.voltage(analogPinIslandOutVoltage, Vcal, phase_shift); // ADC pin, Calibration, phase_shift
	}

	SI_Emonitor.current(analogPinIslandOutCurrent, SIcurCal);
	SB_Emonitor.current(analogPinSunnyBoyOut, SBcurCal);
	CU1_Emonitor.current(analogPinCU1, Cu1CurCal);
	Gen_Emonitor.current(analogPinGenerator, GenCurCal);

//todo cannot check CTavailability like this - what if there is no current?
//	if (analogRead(analogPinIslandOutCurrent) > 0) {
//		CTSI = 1;
//		//SI_Emonitor.current(analogPinIslandOutCurrent, 30);
//		SI_Emonitor.current(analogPinIslandOutCurrent, SIcurCal);
//		CT_count++;
//	} else
//		CTSI = 0; // check to see if CT is connected to CT1 input, if so enable that channel
//
//	if (analogRead(analogPinSunnyBoyOut) > 0) {
//		CTSB = 1;
//		SB_Emonitor.current(analogPinSunnyBoyOut, SBcurCal);
//		CT_count++;
//	} else
//		CTSB = 0; // check to see if CT is connected to CT2 input, if so enable that channel
//
//	if (analogRead(analogPinCU1) > 0) {
//		CTCU1 = 1;
//		CU1_Emonitor.current(analogPinCU1, Cu1CurCal);
//		CT_count++;
//	} else
//		CTCU1 = 0; // check to see if CT is connected to CT3 input, if so enable that channel
//
//	if (analogRead(analogPinGenerator) > 0) {
//		CTGen = 1;
//		Gen_Emonitor.current(analogPinGenerator, GenCurCal);
//		CT_count++;
//	} else
//		CTGen = 0; // check to see if CT is connected to CT4 input, if so enable that channel
//
//	if (CT_count == 0) {
//		Serial.println("NO CT's detected");
//	} else {
//		if (CTSI)
//			Serial.println("CT SI detected");
//		if (CTSB)
//			Serial.println("CT SB detected");
//		if (CTCU1)
//			Serial.println("CT CU1 detected");
//		if (CTGen)
//			Serial.println("CT Gen detected");
//	}

	// calcVI Calculate all. No.of half wavelengths (crossings), time-out
	SI_Emonitor.calcVI(halfWavelengths_Power, timeout);
	SB_Emonitor.calcVI(halfWavelengths_Power, timeout);
	CU1_Emonitor.calcVI(halfWavelengths_Power, timeout);
	Gen_Emonitor.calcVI(halfWavelengths_Power, timeout);

	if (debug <= med) {
		Serial.println("initialised monitors SI, SB, CU1, Gen: ");
		SI_Emonitor.serialprint();
		SB_Emonitor.serialprint();
		CU1_Emonitor.serialprint();
		Gen_Emonitor.serialprint();
	}
}

void setup(void) {
	Serial.begin(9600);  //Initiate Serial communication.
	if (debug >= high)
		Serial.println("in setup ...");

	//serial1 not used
	//Serial1.begin(softwareBaudRate); //, SERIAL_8N1, pinImmersionRX, pinImmersionTX);
	Serial2.begin(softwareBaudRate);  //, SERIAL_8N1, pinEVSERX, pinEVSETX);
	Serial3.begin(softwareBaudRate); //, SERIAL_8N1, pinImmersionRX, pinImmersionTX);
	//powerBucket= new PowerBucket();
	//energyBucket = new EnergyBucket();

	initialiseMonitors();
	pinMode(digitalSOCpin, INPUT);

	EVSE_status = EVSEnotInitialised;
	if (debug >= high)
		Serial.println("... finished setup");
}

void checkIfSOCisHigh() {
	stateOfCharge = digitalRead(digitalSOCpin);  // read input value
}

/**
 * 	a delay is required 70ms may do check out delay and milli()
 * 	 flush seems to have no effect
 */
void wrapCharMessage(HardwareSerial &port, char message[]) {
//	if (debug >= three) {
//		Serial.println("wrapping and sending... ");
//	}
	port.print(charStartMarker);
	port.print(message);
	port.print(charEndMarker);
	//port.flush();
	Serial.print("sent ");
	Serial.println(message);
	delay(delayAfterSendingMessage);
}

/**
 *
 * 	this returns true if the system should wait for the start up sequence to complete
 */
boolean waitForEvseStartup() {
	//millis() rolls over to zero after 49 days
	//looking for a small difference between current time and timeStartCalled
	long currentTime = millis();
	long timeSinceStartCalled = currentTime - timeStartCalled;
//	Serial.print("waitForEvseStartup ");
//	Serial.print(" currentTime ");
//	Serial.print(currentTime);
//	Serial.print(" timeStartCalled ");
//	Serial.print(timeStartCalled);
//	Serial.print(" startUpTime ");
//	Serial.print(startUpTime);
//	Serial.print(" timeSinceStartCalled ");
//	Serial.println(timeSinceStartCalled);

	if (timeSinceStartCalled < 0) {
		//millis)() must have rolled over so reset
		//todo check rollover actions
		timeStartCalled = 0;
		timeSinceStartCalled = startUpTime + 1;
	}
	if (timeSinceStartCalled < startUpTime)
		return true;
	else
		return false;
}

boolean settleEvse() {
	long currentTime = millis();
	long timeSinceStartCalled = currentTime - timeStartCalled;
	if (timeSinceStartCalled < 0) {
		//millis)() must have rolled over so reset
		timeStartCalled = 0;
		timeSinceStartCalled = 2 * (startUpTime + 1);
	}
	if (timeSinceStartCalled > startUpTime
			&& timeSinceStartCalled < (2 * startUpTime)) {
		return true;
	} else
		return false;
}

void stopEVSE() {
	Serial.println("stopping ...");
	wrapCharMessage(*EvseSerial, "EVoff");
}
void setEVSEtoStartingLevel() {
	Serial.println("starting ...");
	wrapCharMessage(*EvseSerial, "EVstr");
	timeStartCalled = millis();
}
//void setEVSEtoMinLevel() {
//	Serial.println("set min ...");
//	wrapCharMessage(*EvseSerial, "EVmin");
//}
void increaseEVSE() {
	wrapCharMessage(*EvseSerial, "EVinc");
}
void softDecreaseEVSE() {
	wrapCharMessage(*EvseSerial, "EVdecS");
}
void hardDecreaseEVSE() {
	wrapCharMessage(*EvseSerial, "EVdecH");
}
void logError(int type, String message) {
//todo
	;
}

void stopHeatingLoad() {
//Serial.println("imm stop");
//	disableReduction = false;
//	avoidOscillation = false;
	wrapCharMessage(*ImmersionSerial, "alloff");
}

void increaseHeatingLoad() {
	previousTimeHeatIncreased = storedTimeHeatIncreased;
	storedTimeHeatIncreased = millis();
	wrapCharMessage(*ImmersionSerial, "inc+");
}
void reduceHeatingLoad() {
	previousTimeHeatDecreased = storedTimeHeatDecreased;
	storedTimeHeatDecreased = millis();
	wrapCharMessage(*ImmersionSerial, "inc-");
}

/**
 * first check if ultimate max power is exceeded and if so turn off heat and EVSE
 * otherwise, if short term max is exceeded, turn down loads
 * then check that battery current is within limits
 */
boolean respondToOverload() {
	systemOverload = false;
	if (sunnyIslandPower > sunnyIslandUltimateMaxPower) {
		Serial.println("Overload Ultimate Max power ...");
		stopHeatingLoad();
		stopEVSE();
		logError(Warning, "overload" + char(sunnyIslandPower));
		systemOverload = true;
		chargingState = overload;
	} else if (sunnyIslandPower > sunnyIslandShortTermMaxPower) {
		Serial.println("short term Max power reached...");
		stopHeatingLoad();
		hardDecreaseEVSE();
		logError(Warning,
				"high load sunny island power" + char(sunnyIslandPower));
		systemOverload = false;
		chargingState = overload;
	}
	if (batteryCurrent > batCurrentThresholdEvse_Danger) {
		Serial.println("Overload EVSE danger ...");

		stopEVSE();
		stopHeatingLoad();
		systemOverload = true;
		chargingState = overload;
	}
	return systemOverload;
}

/**
 * compares frequency and estimates available power based on full capacity of solar
 * SI pushes freq above fACstartDelta to signal to SB to ramp down
 * SB ramps down until the frequency is between nomimal and fACstartDelta, then holds power at this level
 * latent power is available if the frequency is above nominal
 * SB signals to SB to ramp up power by reducing frequency
 *
 * todo this could actually be based on the max power recorded from SB in the last half hour
 */
float measureFreqAndDetermineLatentPVPower() {
	SI_Emonitor.calcF(halfWavelengths_Freq, timeout);
	currentFrequency = SI_Emonitor.frequency;
	freqShift = currentFrequency - nominalFrequency;
	Serial.print("currentFrequency: ");
	Serial.println(currentFrequency, 6);
	Serial.print("freqShift: ");
	Serial.println(freqShift);

//	if (freqShift > fACstartDelta)
//	excessPVPower = availablePVPower * (freqShift - fACstartDelta)
//			/ (fACstartDeltaLimit - fACstartDelta);
	// if freq above nomimal eg 50.5 Hz identify that excess power is available
	//if (freqShift > fACstartDelta + .18) {
	if (freqShift > 0.18) {
		//power is down regulated
		latentPVPowerAvailable = true;
		excessPVPower = availablePVPower * freqShift
				/ (fACstartDeltaLimit - fACstartDelta);
	} else {
		latentPVPowerAvailable = false;
		excessPVPower = 0.00;
	}
//	Serial.print("excessPVPower: ");
//	Serial.println(excessPVPower);
	return excessPVPower;
}

boolean checkLatentPVpowerAvailable() {
	measureFreqAndDetermineLatentPVPower();
//	Serial.print("thresholdExcessPVpower: ");
//	Serial.println(thresholdExcessPVpower);
//	Serial.print("excessPVPower: ");
//	Serial.println(excessPVPower);
//	latentPVPowerAvailable = measureFreqAndDetermineLatentPVPower()
//			> thresholdExcessPVpower;

	return latentPVPowerAvailable;
}

/**
 * first checks overload
 * then updates charging state
 * checks SOC
 * buckets not implemented yet
 */
void checkFlowsAndUpdateBuckets() {
	if (debug >= low) {
		Serial.println(
				String("currents: sICur ") + sunnyIslandCurrent + " sBCur "
						+ sunnyBoyCurrent + " cU1Cur " + consumerUnit1Current
						+ " generatorCurrent " + generatorCurrent
						+ " battery current " + batteryCurrent
						+ " stateOfCharge " + stateOfCharge);
	}
	respondToOverload();
	if (!systemOverload) {
		if (batteryCurrent < thresholdBatCurrent_Low) {
			chargingState = charging;
		}
		if (batteryCurrent > thresholdBatCurrent_High) {
			chargingState = discharging;
		}
		if (batteryCurrent < thresholdBatCurrent_High
				&& batteryCurrent > thresholdBatCurrent_Low) {
			chargingState = idle;
		}
		if (generatorPower > thresholdGeneratorPower) {
			//chargingOrDischarging = "generator";
			Serial.println("generator on");
			chargingState = generator;
		}
	}
	checkIfSOCisHigh();
//todo time should be logged so should the balance of currents in vs out
//	powerBucket.logState(chargingState, sunnyIslandCurrent, sunnyBoyCurrent,
//			consumerUnit1Current, batteryCurrent, stateOfCharge);
//	energyBucket.logState(chargingState, sunnyIslandCurrent, sunnyBoyCurrent,
//			consumerUnit1Current, batteryCurrent, stateOfCharge);
}

/**
 * if charging then increase heating
 * otherwise decrease heating
 * status of EVSE should be checked before as this just responds to available power
 */
void manageHeatingLoads() {
	if (stateOfCharge == HIGH) {
		if (chargingState == charging || chargingState == idle
				|| chargingState == discharging) {
			if (batteryCurrent < batCurrentThresholdImmersion_On
					|| latentPVPowerAvailable) {
				increaseHeatingLoad();
				flipping = false;
				flippingCountVariable = 2;
			}
			if (batteryCurrent > batCurrentThresholdImmersion_Off) {
				if (batteryCurrent > batCurrentThresholdImmersion_ShortTermLimit
						&& !latentPVPowerAvailable) {
					reduceHeatingLoad();
					Serial.println("reduce 1");
				} else {
					long timeBetweenIncAndDec = storedTimeHeatIncreased
							- storedTimeHeatDecreased;
					if (timeBetweenIncAndDec
							< (flippingCountVariable * durationOfLastLoop)) {
						flipping = true;
						flippingCountVariable++;
					} else {
						flipping = false;
						flippingCountVariable = 2;
					}

					if (!flipping && !latentPVPowerAvailable) {
						reduceHeatingLoad();
						Serial.println("reduce 2");
					} else {
						previousTimeForShortTermHeatLimit =
								storedTimeForShortTermHeatLimit;
						storedTimeForShortTermHeatLimit = millis();
						long flippingTime = storedTimeForShortTermHeatLimit
								- previousTimeForShortTermHeatLimit;

						if (flippingTime < 0) {
							//millis)() must have rolled over so reset
							//todo check rollover actions
							storedTimeForShortTermHeatLimit = 0;
							previousTimeForShortTermHeatLimit = 0;
							flippingTime = shortTermHeatLimitPeriod + 1;
						}
						if (flippingTime > shortTermHeatLimitPeriod
								&& !latentPVPowerAvailable) {
							//Serial.println("reducing");
							reduceHeatingLoad();
							Serial.println("reduce 3");
							flipping = false;
							flippingCountVariable = 2;
						}
					}
				}
			}
//			if (batteryCurrent < batCurrentThresholdImmersion_Off
//					&& batteryCurrent > batCurrentThresholdImmersion_On) {
//				//powerBucket.logStability();
//			}
		}
//		if (chargingState == idle) {
//			if (batteryCurrent < batCurrentThresholdImmersion_On
//					|| latentPVpowerAvailable()) {
//				increaseHeatingLoad();
//			}
//			if (batteryCurrent > batCurrentThresholdImmersion_Off
//					&& !latentPVpowerAvailable()) {
//				reduceHeatingLoad();
//			}
//		} else if (chargingState == discharging) {
//			if (latentPVpowerAvailable())
//				increaseHeatingLoad();
//			else
//				reduceHeatingLoad();
//		} else
		if (chargingState == generator) {
			Serial.println("generator on - managing loads");
			stopHeatingLoad();
			stopEVSE();
		}
	} else {
		Serial.println("SOC low - managing loads");
		stopHeatingLoad();
		stopEVSE();
	}
}

/**
 * Serial buffer is 64 bytes.SERIAL_RX_BUFFER_SIZE. If this fills up the message may not be completely transmitted
 * This will cause an overflow. If the buffer has overflowed it will be emptied.
 * A timeout is include to avoid the funcion hanging
 */
void checkOverflowAndEmptyBuffer(HardwareSerial &port) {
//	if (port.available() == SERIAL_RX_BUFFER_SIZE) { // Just to be general, 64 bytes in most cases
//buffer is overrun, do what u will
	Serial.println("overflow!");
	int timeout = 0;
//buffer size is 64 bytes, each character can represent a byte
	while (port.available() && timeout < SERIAL_RX_BUFFER_SIZE) {
		port.read();
		timeout++;
	}
//	}
}

/**
 * Serial communication is inherently unreliable
 * When the buffer fills up messages will be lost
 * This method just counts errors to help with configuring the system
 */
void updateErrorCount() {
	countOFAllBufferResets++;
	if (countOFAllBufferResets > bufferResetMax) {
		Serial.println("overflow! Reset");
		countOFAllBufferResets = 0.00;
		countOfAllMessagesCorreclyBounded = 0.00;
	}
	if (countOfAllMessagesCorreclyBounded > bufferResetMax) {
		Serial.println("overflow! Reset");
		countOfAllMessagesCorreclyBounded = 0.00;
	}
	Serial.print("bufferErrorCount: ");
	Serial.println(countOFAllBufferResets);
	Serial.print(" correct messages: ");
	Serial.println(countOfAllMessagesCorreclyBounded);
	if (countOFAllBufferResets < bufferResetMax
			&& countOfAllMessagesCorreclyBounded < bufferResetMax) {
		Serial.print("% errors ");
		if (countOfAllMessagesCorreclyBounded > 0.00) {

			Serial.print("countOFAllBufferResets ");
			Serial.println(countOFAllBufferResets);
			Serial.print("countOfAllMessagesCorreclyBounded");
			Serial.println(countOfAllMessagesCorreclyBounded);

			Serial.println(
					(countOFAllBufferResets / countOfAllMessagesCorreclyBounded)
							* 100);
		}
	}
}

/**
 * Serial communication is inherently unreliable
 * When the buffer fills up messages will be lost
 * This method can be called if poor messages are recieved
 */
void addressIssues(HardwareSerial &port) {
//updateErrorCount();
	checkOverflowAndEmptyBuffer(port);

}

/**
 *
 * This method just counts messages to help with configuring the system
 */
void countMessageNumber(boolean messageStarted) {
	if (messageStarted) {
		countOfAllMessagesCorreclyBounded++;
		if (countOfAllMessagesCorreclyBounded < 0.00) {
			Serial.println(
					"countOfAllMessagesCorreclyBounded overflow! Reset both countOfAllMessagesCorreclyBounded and countOFAllBufferResets to zero");
			countOfAllMessagesCorreclyBounded = 0.00;
			countOFAllBufferResets = 0.00;
		}
	}
}

/**
 * Serial communication - receives messages and unpacks from start and end characters
 */
void recvCharsWithStartEndMarkers(HardwareSerial &port) {
	static boolean recvInProgress = false;
	static byte ndx = 0;
	char startMarker = char(0x3C);
	char endMarker = char(0x3E);
	char charReadFromPort;
	boolean newData = false;
	newInstruction = false;
	boolean messageStarted = false;
	memset(receivedChar, 0, sizeof receivedChar);	//clear message buffer

	if (debug >= high) {
//		Serial.print("port.available:");
//		Serial.print((port.available()));
//		Serial.println(":");
		while (port.available() > 0 && newData == false) {
			charReadFromPort = (char) port.read();
//			if (debug >= med) {
//				Serial.print("charReadFromPort:");
//				Serial.println(charReadFromPort);
//			}
			if (recvInProgress == true) {
				if (charReadFromPort != endMarker) {
					receivedChar[ndx] = charReadFromPort;
					ndx++;
					if (ndx >= numBytes) {
						ndx = numBytes - 1;
					}
				} else {
					//countMessageNumber(messageStarted);
					receivedChar[ndx] = '\0'; // terminate the string
					recvInProgress = false;
					numReceived = ndx; // save the number for use when printing
					ndx = 0;
					newData = true;
					newInstruction = true;
				}
			} else if (charReadFromPort == startMarker) {
				recvInProgress = true;
				messageStarted = true;
			}
		}
	}
	newData = false;
}

/**
 * translates message from EVSE to an int value for EVSE_status
 * the message seems to get corrupted every 13 or so times, clearing the buffer seems to address this
 *
 */
void updateEVSEstatus() {
//	if (debug >= high) {
//	Serial.println("updateEVSEstatus ");
//	Serial.print("receivedChar ");
//	Serial.println(receivedChar);
//	}
	if (newInstruction) {
//Serial.println("updating ... ");
		if (strcmp(receivedChar, "EV0") == 0) {
			EVSE_status = EVSEnotInitialised;
			if (debug >= low)
				Serial.println("EVSEnotInitialised");
		} else if (strcmp(receivedChar, "EV1") == 0) {
			EVSE_status = EVSEoff;
			if (debug >= low)
				Serial.println("EVSEoff");
		} else if (strcmp(receivedChar, "EV2") == 0) {
			EVSE_status = EVSEwaiting;
			if (debug >= low)
				Serial.println("EVSEwaiting");
		} else if (strcmp(receivedChar, "EV3") == 0) {
			EVSE_status = EVSEready;
			if (debug >= low)
				Serial.println("EVSEready");
		} else if (strcmp(receivedChar, "EV4") == 0) {
			EVSE_status = EVSEcharging;
			if (debug >= low)
				Serial.println("EVSEcharging");
		} else if (strcmp(receivedChar, "EV5") == 0) {
			EVSE_status = EVSEchargeError;
			if (debug >= low)
				Serial.println("EVSEchargeError");
		} else if (strcmp(receivedChar, "EV6") == 0) {
			EVSE_status = EVSEcommError;
			if (debug >= low)
				Serial.println("EVSEcommError");
		} else if (strcmp(receivedChar, "EV7") == 0) {
			EVSE_status = SerComError;
			if (debug >= low)
				Serial.println("SerComError");
		} else {
			EVSE_status = EVSEnotInitialised;
			if (debug >= low) {
				Serial.println("not matched EVSEnotInitialised");
				Serial.println("clearing buffer");
			}
			addressIssues(*EvseSerial);
		}
		newInstruction = false;
	}
}

/**
 * configuarbles
 * delayAfterEachReading to avoid capacitance error
 * numberOfSamplesForBatteryCurrent
 *
 */
float readBatteryTransducer() {
// take a number of analog samples and add them up
	float sum = 0.000;
	float sample_count = 0;
	float interceptAdjustment = 1.00;
	for (int i = 0; i < numberOfSamplesForBatteryCurrent; i++) {
		sum += analogRead(analogPinBattery);
		sample_count++;
		delay(delayAfterEachBatteryReading);
	}
// Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):

	float voltage = (sum / (sample_count + interceptAdjustment))
			* (Vref / 1024.000);
	voltage = voltage - (Vref / 2.000);
	return voltage;
}

void measureBatteryCurrent() {
	batteryCurrent = readBatteryTransducer() * batteryCalibration;
}

/**
 * use EnergyMonitors to determine current and power
 * battery current based on hall effect sensor input
 * SB can down regulate based on frequency signals
 * we  determine frequency shift here with a call to measureFreqAndDetermineLatentPVPower();
 *
 */
void measureCurrentAndPowerFlows() {
	SI_Emonitor.calcVI(halfWavelengths_Power, timeout);
	SB_Emonitor.calcVI(halfWavelengths_Power, timeout);
	CU1_Emonitor.calcVI(halfWavelengths_Power, timeout);
	Gen_Emonitor.calcVI(halfWavelengths_Power, timeout);
	measureBatteryCurrent(); // read the input pin
//	if (debug >= low) {
//		Serial.println(
//				"SI: realpower, apparent power, Vrms, Irms, power factor");
//		SI_Emonitor.serialprint();
//		Serial.println(
//				"SB: realpower, apparent power, Vrms, Irms, power factor");
//		SB_Emonitor.serialprint();
//		Serial.println(
//				"CU1: realpower, apparent power, Vrms, Irms, power factor");
//		CU1_Emonitor.serialprint();
//		Serial.println(
//				"Gen: realpower, apparent power, Vrms, Irms, power factor");
//		Gen_Emonitor.serialprint();
//	}
	sunnyIslandCurrent = SI_Emonitor.Irms;
	sunnyBoyCurrent = SB_Emonitor.Irms;
	consumerUnit1Current = CU1_Emonitor.Irms;
	generatorCurrent = Gen_Emonitor.Irms;

	sunnyIslandPower = SI_Emonitor.realPower;
	sunnyBoyPower = SB_Emonitor.realPower;
	consumerUnit1Power = CU1_Emonitor.realPower;
	generatorPower = Gen_Emonitor.realPower;

	latentPVPowerAvailable = checkLatentPVpowerAvailable();
}

/**
 * assume that EVSE will be in operation
 * reduce heating loads and make adjustments for EVSE charging
 *
 */
void updateEVSE() {
//todo this looks uneccessary
//	measureCurrentAndPowerFlows();
//	boolean overload = respondToOverload();
//	if (overload)
//		return;
	reduceHeatingLoad();
	if (stateOfCharge == HIGH && !systemOverload) {
		if (evseStarted) {
			if (waitForEvseStartup()) {
				Serial.println("waitForEvseStartup");
				if (batteryCurrent < batCurrentThresholdEvse_Start) {
					increaseEVSE();
				}
				if (batteryCurrent > batCurrentThresholdEvse_Start_Reduce
						&& !latentPVPowerAvailable) {
					softDecreaseEVSE();
				}
/**
 * currentFrequency: 50.251255
freqShift: 0.25
currents: sICur 8.07 sBCur 7.03 cU1Cur 14.73 generatorCurrent 0.00 battery current 31.20 stateOfCharge 1
sent inc-
sent EVinc
sent EVdecS
 */
			} else if (settleEvse()) {
				if (batteryCurrent > batCurrentThresholdEvse_Reduce) {
					softDecreaseEVSE();
				}
			} else {
				if (batteryCurrent < batCurrentThresholdEvse_Increase
						|| latentPVPowerAvailable) {
					increaseEVSE();
				}
				if (batteryCurrent > batCurrentThresholdEvse_Reduce
						&& !latentPVPowerAvailable) {
					softDecreaseEVSE();
				}
				if (batteryCurrent > batCurrentThresholdEvse_ReduceQuickly) {//this can be called if there is latent power
					//call decrease again to ramp up decrease
					softDecreaseEVSE();
				}
				if (batteryCurrent > batCurrentThresholdEvse_Increase
						&& batteryCurrent < batCurrentThresholdEvse_Reduce) {
					//this is perfect - no change
					//log stability - increase score
				}
				if (batteryCurrent > batCurrentThresholdEvse_Off) {
					stopEVSE();
				}
			}
		} else if (!evseStarted) {
			if ((batteryCurrent < batCurrentThresholdEvse_Start)
			//&& sunnyBoyPower < SBPowerThresholdEvse_Start)
					|| latentPVPowerAvailable) {
				//Serial.println("starting evse");
				setEVSEtoStartingLevel();
			}
			if (batteryCurrent < batCurrentThresholdEvse_Increase
					&& batteryCurrent > batCurrentThresholdEvse_Start) {
				//no change
			}
		}
	} else {
//this will only be called when SoC ==LOW
		Serial.println("this is actually called");
		stopEVSE();
		stopHeatingLoad();
	}
}

/**
 * gets status from EVSE
 * updates status which measures flows again!
 */
void checkEVSEandManageLoads() {
	recvCharsWithStartEndMarkers(*EvseSerial);
	updateEVSEstatus();
	switch (EVSE_status) {
	case EVSEnotInitialised:
		evseStarted = false;
//stopEVSE();
		manageHeatingLoads();
		break;
	case EVSEoff:
		evseStarted = false;
		manageHeatingLoads();
		break;
	case EVSEwaiting:
		evseStarted = false;
		//to do what happens when car is charged
		updateEVSE();
		break;
	case EVSEready:
		evseStarted = false;
		//to do what happens when car is charged
		updateEVSE();

		//todo check
		//manageHeatingLoads();

		break;
	case EVSEcharging:
		evseStarted = true;
		updateEVSE();
		break;
	case EVSEchargeError:
		evseStarted = false;
		stopEVSE();
		stopHeatingLoad();
		break;
	case EVSEcommError:
		evseStarted = false;
		stopEVSE();
		stopHeatingLoad();
		break;
	case SerComError:
		evseStarted = false;
		stopEVSE();
		stopHeatingLoad();
		EVSEErrorCount++;
		break;
	default:
//C Statements
//this should not happen, throw error
		logError(Warning, "issue with EVSE status");
	}
}

void callibrateCalcVolSetPointsForFreqMeasurement(unsigned int _inPinV) {
	int minVoltageReading = 0;
	int maxVoltageReading = 0;
	int currentVolReading = 0;
	for (int i = 0; i < 50; i++) {
		currentVolReading = analogRead(_inPinV);
		if (currentVolReading > maxVoltageReading) {
			maxVoltageReading = currentVolReading;
		}
		if (currentVolReading < minVoltageReading) {
			minVoltageReading = currentVolReading;
		}
		delay(10);
	}
	Serial.print("minVoltageReading: ");
	Serial.println(minVoltageReading);
	Serial.print("maxVoltageReading: ");
	Serial.println(maxVoltageReading);
	Serial.print("zeroPointCrossing: ");
	Serial.println((maxVoltageReading + minVoltageReading) / 2);
}

void loop(void) {
	const boolean testingSystem = false;

	const boolean callibrateVoltage = false;
	const boolean testingEvse = false;
	const boolean testingImmersion = false;
	const boolean testingCallibration = false;

	previousLoopStartTime = storedTime;
	storedTime = millis();
	durationOfLastLoop = storedTime - previousLoopStartTime;
//	Serial.print("durationOfLastLoop");
//	Serial.println(durationOfLastLoop);

	if (!testingSystem) {

		measureCurrentAndPowerFlows();
		checkFlowsAndUpdateBuckets();
		checkEVSEandManageLoads();

	} else if (testingSystem) {
		if (callibrateVoltage) {
//			SI_Emonitor.callibrateCalcF(analogPinIslandOutVoltage);
			callibrateCalcVolSetPointsForFreqMeasurement(
					analogPinIslandOutVoltage);
		}
		if (testingEvse) {
			Serial.println("testing starting level....");
			setEVSEtoStartingLevel();
			delay(1000);
		}
		if (testingImmersion) {

//			 batCurrentThresholdImmersion_Off = 5.00;
//			 batCurrentThresholdImmersion_ShortTermLimit = 8.00;
//			 batCurrentThresholdImmersion_On = -17.00;
			Serial.println("testing SOC low");
			int delayTime = 2000;

			stateOfCharge = LOW;
			manageHeatingLoads();
			delay(delayTime);

			stateOfCharge = HIGH;
			chargingState = charging;

			Serial.print("testing batteryCurrent = ");
			Serial.println(batCurrentThresholdImmersion_On - 1);
			batteryCurrent = batCurrentThresholdImmersion_On - 1;
			manageHeatingLoads();
			manageHeatingLoads();
			manageHeatingLoads();
			delay(delayTime * 10000000);

			Serial.println("testing batteryCurrent = -20");
			batteryCurrent = -20;
			manageHeatingLoads();
			delay(delayTime);

			Serial.println("testing batteryCurrent = 6");
			batteryCurrent = 6;
			manageHeatingLoads();
			delay(delayTime);

			Serial.println("testing batteryCurrent = -18");
			batteryCurrent = -18;
			manageHeatingLoads();
			delay(delayTime);

			Serial.println("testing batteryCurrent = 6");
			batteryCurrent = 6;
			manageHeatingLoads();
			delay(delayTime);

			Serial.println("testing batteryCurrent = 6");
			batteryCurrent = 6;
			manageHeatingLoads();
			delay(delayTime);

			Serial.println("testing batteryCurrent = 6");
			batteryCurrent = 6;
			manageHeatingLoads();
			delay(shortTermHeatLimitPeriod);

			Serial.println("testing batteryCurrent = 6");
			batteryCurrent = 6;
			manageHeatingLoads();
			delay(delayTime);

			Serial.println("testing batteryCurrent = -18");
			batteryCurrent = -18;
			manageHeatingLoads();
			delay(delayTime);

			Serial.println("testing batteryCurrent = 9");
			batteryCurrent = 9;
			manageHeatingLoads();
			delay(delayTime);

			delay(10 * delayTime);

//			for (int i = 0; i < 4; i++) {
//				increaseHeatingLoad();
//				Serial.println("inc....");
//				delay(1000);
//			}
//			for (int i = 0; i < 4; i++) {
//				reduceHeatingLoad();
//				Serial.println("red....");
//
//				delay(1000);
		}
		if (testingCallibration) {
			stopHeatingLoad();
			//int i = 0;
			int samples = 30;
			for (int i = 0; i < 80; i++) {
				if (i == 12 || i == 48 || i == 60) {
					increaseHeatingLoad();
					Serial.println("inc....");
				}
				measureCurrentAndPowerFlows();
				checkFlowsAndUpdateBuckets();
				delay(300);
			}
			//delay(10000);
			stopHeatingLoad();
		}
	}
}

