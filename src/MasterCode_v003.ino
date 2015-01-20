/*
 * RC Retracts Sequencer and LED controller
 * Code by Thomas Nilsen (thomas.nilsen (at) doc-s (dot) co (dot uk)
 *
 * Code for reading RC changes based on code by rcarduino.blogspot.com - Duane B
 *
 * Requires the following libraries:
 *
 *    PinChangeInt -  https://github.com/GreyGnome/PinChangeInt
 *    SimpleTimer  -  http://playground.arduino.cc/Code/SimpleTimer
 *
 * Functions of project
 * 6 servo channels
 * 5 led channels
 * 3 RX inputs

 * SERVO 1 = Left inner door
 * SERVO 2 = Right inner door
 * SERVO 3 = Left main retract
 * SERVO 4 = Right main retract
 * SERVO 5 = Retract wheel
 * SERVO 6 = Flap radioator
 * SERVO 7 = Landing Light servo
 *
 */

/*
 * 1. Check if we have RX change
 * 1.1 Set RX Change flag and time 
 * 2. If different, set Move flag 
 * 3. Walk array for new positions
 * 3.1 Set new position for servo
 * 3.2 Set old position for servo ???
 * 4. Set statusflag servo move = 1 to prevent other operations
 * 5. Ease servo over time to new position
 * 6. Check new position - against current position, and when hit, set flag to 0
 * 7. Update old position array to mach new
 * 8 Clear statusflag move

 Old = 1000		New = 2000
 Diff = +1000

 Old = 2000		New = 1000
 Diff = -1000
 
  * Switch ON check
  * 1. Is switch in the default start position
  * 1.a If not, wait till it is but make now movements
  * 2.a If it is, all is ok - set parameters
  * 
  * */
#include <EEPROM.h>
#include <PinChangeInt.h>  // https://github.com/GreyGnome/PinChangeInt
#include <SimpleTimer.h>
#include <Servo.h>
#define DEBUG 1

/* These variables defines where the servo will start and end as well as
 * First parameter is for servo1, last for servo 6
 */

#define SERVO1_MIN 1000
#define SERVO1_MAX 2000

#define SERVO2_MIN 1200
#define SERVO2_MAX 1800

#define SERVO3_MIN 1200
#define SERVO3_MAX 1800

#define SERVO4_MIN 1200
#define SERVO4_MAX 1800

#define SERVO5_MIN 1200
#define SERVO5_MAX 1800

#define SERVO6_MIN 1200
#define SERVO6_MAX 1800

#define SERVO7_MIN  1200
#define SERVO7_MAX 1800

// If the servo direction needs to be reveres, just swap the position of SERVOx_UP and SERVOx_DOWN in the ServoUp_Pos() and ServoDown_Pos() arrays
int ServoUp_Pos[]   = {SERVO1_MAX, SERVO2_MIN, SERVO3_MIN, SERVO4_MIN, SERVO5_MIN, SERVO6_MIN, SERVO7_MIN}; // Array to hold the startup position of the servo. This will be overwritten if values exists in NVRAM
int ServoDown_Pos[] = {SERVO1_MIN, SERVO2_MAX, SERVO3_MAX, SERVO4_MAX, SERVO5_MAX, SERVO6_MAX, SERVO7_MAX}; // Array to hold the max travel of each servo

/*
 * First sequence is for gear up. Paramerers are {SERVONUM, DELAY TILL NEXT ACTION}
 *  Second sequence is for gear down.
 */

#define GEAR_UP     1  // Do not change
#define GEAR_DOWN   0	  // Do not change
#define LED		  1  // Used for cycle ID of LEDs - Do NOT change
#define SERVO       0	  // Used for Cycle ID of servos - Do NOT change
#define OFF         0
#define ON          1
byte DefaultStartState = GEAR_UP;	// Set which default state we should use - this will be overridern by values in NVRAM

#define CYCLE_STEPS 6  // Defines how many servo steps there are for a full cycle
int ServoSeq[2][CYCLE_STEPS][4] = {
                        // Gear Up (Type, Device-Channel, Delay, ON/OFF (applies to LED Only))
                        {
                          {SERVO, 5, 1000, 0},
                          {SERVO, 4, 2000, 0},
                          {SERVO, 3, 3000, 0},
                          {SERVO, 2, 4000, 0},
                          {SERVO, 1,    0, 0},
							 {LED,   1, 2000, OFF}
                        },

                        // Gear Down (Type, Device-Channel, Delay, ON/OFF (applies to LED Only))
                        {
                          {LED,   1, 2000, ON}, 
							 {SERVO, 1,  500, 0},
                          {SERVO, 2, 2000, 0},
                          {SERVO, 3, 4000, 0},
                          {SERVO, 4, 4000, 0},
                          {SERVO, 5,    0, 0}
							 
                        }};
//Serial.println(ServoSeq[0][3][0]); Would print 1 which is the last entry of gear down

byte ServoSteps[]   = {10,   10,   10,   10,   5,    5};  // Controls how far the servo should move for each cycle.
int ServoDelay[]    = {20,  20,   20,   21,   22,    20};  // Controls how fast the servo should move in ms

int GearSwUp[]    = {1000, 1500};  // Values below this means gear is to be set in Up position
int GearSwDown[]  = {1510, 2000};  // Values higher than this gear is to be set in Down position

// The following variable defines how to read the LED RX Channel. It allows for a 3 position/switch to be used.
//  The array needs 6 parameteres, two for each channel of low and high.
// First group - low,high, second group - low, high, 3rd group low, high
// Leave some gap beteen each of the 3 goups
//						LOW   HIGH  LOW   HIGH  LOW   HIGH
int LedSw[]	    = {1000, 1400, 1410, 1800, 1810, 2000};	

int LedSwOn     = 1500;  // Values below this means LEDs is to be enabled
int LedSwOff    = 1510;  // Values below this means LEDs are to be enabled
int LedAuxSwOn  = 1500;  // Used to turn LED 5 on
int LedAuxSwOff = 1510;  // Used to tunr LED 5 off


/* Set up PINs used by the components */
#define SERVO1_PIN 7  // Door1 Servo - Digital port 2
#define SERVO2_PIN 8  // Door2 Servo - Digital port 3
#define SERVO3_PIN 9  // Main retract 1 servo
#define SERVO4_PIN 10  // Main retract 2 servo Digital port 5
#define SERVO5_PIN 11  // Tail/Nose Retract
#define SERVO6_PIN 12  // Radioator flap Door Servo - Digital port 7
#define SERVO7_PIN 13  // Servo for landing light in wing

// PINS assigned to LEDs
#define LED1_PIN 6  // Digital pin controlling LED 1
#define LED2_PIN 5
#define LED3_PIN 4
#define LED4_PIN 3
#define LED5_PIN 2  // This is also connected to the internal LED on the arduino

#define RX1_PIN A3  	// RX Channel input 1 
#define RX2_PIN A1	// RX Channel input 2
#define RX3_PIN A2	// RX Channel input 3 

/******* Values below should not need to be modofied unless you're altering the code **********/
#define RX1_FLAG 1  // Used to check which RX input is triggered by interrupt
#define RX2_FLAG 2
#define RX3_FLAG 4

#define LED_ON      1
#define LED_OFF     0
#define LED_AUX_ON  1
#define LED_AUX_OFF 0



//#define SERVO_REFRESH 30  // How many ms to wait between servo updates
#define NUM_SERVOS  7

// Values below should not need to be changed...
int ServoPinIndex[]    = {SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN, SERVO5_PIN, SERVO6_PIN, SERVO7_PIN};
int ServoCurrent_Pos[] = {0,0,0,0,0,0,0};  // Array to hold the last known position of the servo
int ServoOld_Pos[]     = {0,0,0,0,0,0,0};  // Contains the previously used position
int ServoNext_Pos[]    = {0,0,0,0,0,0,0};  // Will hold where the servo is to move to
int Servo_Min[]        = {0,0,0,0,0,0,0};
int Servo_Max[]        = {0,0,0,0,0,0,0};
long ServoTimer[]      = {0,0,0,0,0,0,0};  // Holds millis() for when the last servo move was completed. Used to delay the servo movement
byte ServoMoveStatus[] = {0,0,0,0,0,0,0};  // Will be set to 1 when servo is in process


byte LedPinIndex[] = {LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN, LED5_PIN};
byte Led_Delay[]   = {10, 10, 10, 10, 10};
long Led_Timer[]   = {0, 0 ,0 ,0 ,0};

boolean Armed_Flag;	// Will be set to true once init has verified Retract switch is verified in the right position after startup
boolean Gear_Flag;	
boolean Led_Flag;
boolean LedAux_Flag;

byte GearCurrentState = 0;
byte GearCycleStep    = 0;  // Will hold which cycle the current GearDown/Up function is currently processing
byte startGearCycle   = 0;

boolean GearDownComplete = 0;
boolean GearUpComplete   = 0;

int iMaxUp   = max(GearSwUp[0], GearSwUp[1]);
int iMinUp   = min(GearSwUp[0], GearSwUp[1]);
int iMaxDown = max(GearSwDown[0], GearSwDown[1]);
int iMinDown = min(GearSwDown[0], GearSwDown[1]);


int i, a, b;
long GearTimer, LedTimer, LedAuxTimer;

// holds the update flags for RX interrupt routine
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unRX1InShared, unRX2InShared, unRX3InShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulRX1Start, ulRX2Start, ulRX3Start;

Servo myServo[NUM_SERVOS];	// Generate servo objects

SimpleTimer timer;
long tim1;

void setup() {
	delay(2000);
	
	#ifdef DEBUG
		Serial.begin(115200);
	#endif
	getNVRAM();	// Read last known state of gear up/down position
	InitServos();

	tim1=millis();


	// using the PinChangeInt library, attach the interrupts
	// used to read the channels
	PCintPort::attachInterrupt(RX1_PIN, calcRX1,CHANGE);
	PCintPort::attachInterrupt(RX2_PIN, calcRX2,CHANGE);
	PCintPort::attachInterrupt(RX3_PIN, calcRX3,CHANGE);
}

void loop() {
	boolean Led_Flag, LedAux_Flag, Gear_Flag;


	for (i =0; i < NUM_SERVOS; i++) {
		MoveServo(i);
	}

	#ifdef DEBUG
		//Serial.println(i);
	#endif
	// create local variables to hold a local copies of the channel inputs
	// these are declared static so that thier values will be retained
	// between calls to loop.
	static uint16_t unRX1In;
	static uint16_t unRX2In;
	static uint16_t unRX3In;
	// local copy of update flags
	static uint8_t bUpdateFlags;

	// check shared update flags to see if any channels have a new signal
	if (bUpdateFlagsShared) {
		noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

		// take a local copy of which channels were updated in case we need to use this in the rest of loop
		bUpdateFlags = bUpdateFlagsShared;

		// in the current code, the shared values are always populated
		// so we could copy them without testing the flags
		// however in the future this could change, so lets
		// only copy when the flags tell us we can.

		if( bUpdateFlags & RX1_FLAG ) {
		  unRX1In = unRX1InShared;
		}

		if( bUpdateFlags & RX2_FLAG ) {
		  unRX2In = unRX2InShared;
		}

		if( bUpdateFlags & RX3_FLAG ) {
		  unRX3In = unRX3InShared;
		}

		// clear shared copy of updated flags as we have already taken the updates
		// we still have a local copy if we need to use it in bUpdateFlags
		bUpdateFlagsShared = 0;

		interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
		// as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
		// service routines own these and could update them at any time. During the update, the
		// shared copies may contain junk. Luckily we have our local copies to work with :-)
	}

	Gear_Flag   = getGearSwitch(unRX1In);
	Led_Flag    = getLedSwitch(unRX2In);
	LedAux_Flag = getLedAuxSwitch(unRX3In);

	switch (Gear_Flag) {
	case GEAR_UP:
		  GearUp();
		  break;

	case GEAR_DOWN:
		  GearDown();
		  break;

	}
	
	switch (Led_Flag) {
	case LED_ON:
		break;
	
	case LED_OFF:
		break;
		
	}
//
//
//  if(bUpdateFlags & RX2_FLAG)
//  {
//    if(servoSteering.readMicroseconds() != unSteeringIn)
//    {
//      Serial.println(unSteeringIn);
//      servoSteering.writeMicroseconds(unSteeringIn);
//    }
//  }
//
//  if(bUpdateFlags & RX3_FLAG)
//  {
//    if(servoAux.readMicroseconds() != unAuxIn)
//    {
//
//      servoAux.writeMicroseconds(unAuxIn);
//    }
//  }

  bUpdateFlags = 0;
}

void MoveServo(int ServoId){
	int NewServoPos, tmpPos;

//  if ((millis() - ServoTimer[ServoId]) > SERVO_REFRESH ) {
	if ( ServoOld_Pos[ServoId] != ServoNext_Pos[ServoId] ){

		if ( (millis()-ServoTimer[ServoId]) > ServoDelay[ServoId] ) {
			/* Check direction of movement - but only when the servo has yet to be moved */

			if ((ServoNext_Pos[ServoId] - ServoOld_Pos[ServoId]) < 0 ) {
				tmpPos = ServoCurrent_Pos[ServoId] - ServoSteps[ServoId];
			} else {
				tmpPos = ServoCurrent_Pos[ServoId] + ServoSteps[ServoId];
			}
		
			NewServoPos = constrain(tmpPos, Servo_Min[ServoId], Servo_Max[ServoId]);
			ServoCurrent_Pos[ServoId] = NewServoPos;
			myServo[ServoId].writeMicroseconds(ServoCurrent_Pos[ServoId]);
			ServoTimer[ServoId] = millis();

			if(NewServoPos == ServoNext_Pos[ServoId]) {
				ServoOld_Pos[ServoId] = NewServoPos;	// Set old pos once the servo has reached is final destination
			}

/*			#ifdef DEBUG
				Serial.print("ServoID: ");
				Serial.print(ServoId);
				Serial.print(" Servo Delay: ");
				Serial.print(ServoDelay[ServoId]);
				Serial.print(" Position: ");
				Serial.print(ServoCurrent_Pos[ServoId]);
				Serial.print(" Next: ");
				Serial.print(ServoNext_Pos[ServoId]);
				Serial.print(" Old: ");
				Serial.println(ServoOld_Pos[ServoId]);
			#endif*/
		}
    }
}


/*
 * Function will initiate the servo upon start of controller
 */
void InitServos(){
  int tmpPos;
  // Set pulldown on all servo pins to avoud uncontrolled movements
  for (int b = 0; b < NUM_SERVOS; b++){
    digitalWrite(ServoPinIndex[b], LOW);
    Servo_Min[b] = min(ServoUp_Pos[b], ServoDown_Pos[b]);
    Servo_Max[b] = max(ServoUp_Pos[b], ServoDown_Pos[b]);
  }
  // Attach servos to arduino
  for (int i = 0; i < NUM_SERVOS; i++){
    
	/* Define the defaults startup position for each servo */
	if ( DefaultStartState == GEAR_UP ){
		tmpPos = ServoUp_Pos[i];
	} else {
		tmpPos = ServoDown_Pos[i];
	}
	myServo[i].attach(ServoPinIndex[i]);	
	myServo[i].writeMicroseconds(tmpPos);
	ServoOld_Pos[i] = tmpPos;
	ServoNext_Pos[i] = tmpPos;
	ServoCurrent_Pos[i] = tmpPos;
	ServoTimer[i] = millis();
	#ifdef DEBUG
	Serial.print(ServoPinIndex[i]);     
	Serial.print(" ServoID: ");
	Serial.print(i);
	Serial.print(" Servo Pos: ");
	Serial.println(tmpPos);
    #endif
  }
  
  //FIX init routine to read last current setting in NVRAM and use as base

  
}


void calcRX1() {
	// if the pin is high, its a rising edge of the signal pulse, so lets record its value
	if(digitalRead(RX1_PIN) == HIGH) {
		ulRX1Start = micros();
	}  else {
		// else it must be a falling edge, so lets get the time and subtract the time of the rising edge
		// this gives use the time between the rising and falling edges i.e. the pulse duration.
		unRX1InShared = (uint16_t)(micros() - ulRX1Start);
		// use set the flag to indicate that a new channel signal has been received
		bUpdateFlagsShared |= RX1_FLAG;
	}
}
void calcRX2() {
	if(digitalRead(RX2_PIN) == HIGH) {
		ulRX2Start = micros();
	} else { 
		unRX2InShared = (uint16_t)(micros() - ulRX2Start);
		bUpdateFlagsShared |= RX2_FLAG;
	}
}
void calcRX3() {
	if(digitalRead(RX3_PIN) == HIGH) {
		ulRX3Start = micros();
	} else {
		unRX3InShared = (uint16_t)(micros() - ulRX3Start);
		bUpdateFlagsShared |= RX3_FLAG;
	}
}

boolean getGearSwitch(uint16_t pulse){
	int iMax, iMin;
	static int GearSwCurPulse;
	
	// 1. Check if switch is in default start position - if not abort till it is.
	// 2. Once it is, set Armed_Flag = On
	if ( ! Armed_Flag ) {
		switch (DefaultStartState){
			case GEAR_UP:
				iMax = iMaxUp;
				iMin = iMinUp;
				GearSwCurPulse = iMaxUp;
				break;
			case GEAR_DOWN:
				iMax = iMaxDown;
				iMin = iMinDown;
				GearSwCurPulse = iMaxDown;
				break;
		}
		/* If pulse is within the parameteres specified, we are good to go and can arm */
		if( pulse >= iMin && pulse <= iMax) {
			Armed_Flag = true;	// Arm controller
			#ifdef DEBUG
				Serial.println("Controller is armed");
			#endif
		}
	}
	
	/* Check if switch has changed to Gear UP */
	
	if ( Armed_Flag ) {
		/* First check if switch is within UP range */
		if( pulse <= iMaxUp && pulse >= iMinUp && GearCurrentState != GEAR_UP ) {
		
			#ifdef DEBUG
				Serial.print("Gear SW UP : Current State : ");
				Serial.println(GearCurrentState);
			#endif
			GearCurrentState = GEAR_UP;
			return GEAR_UP;

		} else if ( pulse <= iMaxDown && pulse >= iMinDown && GearCurrentState != GEAR_DOWN ){

			#ifdef DEBUG
				Serial.print("Gear SW Down | Current State : ");
				Serial.println(GearCurrentState);
			#endif
			GearCurrentState = GEAR_DOWN;
			return GEAR_DOWN;
		}
		GearSwCurPulse = pulse;
		return GearCurrentState;
	
	}

}
boolean getLedSwitch(uint16_t pulse){

    if( pulse <= LedSwOn ) {
      #ifdef DEBUG
        //Serial.println("LED SW On");
      #endif
      return LED_ON;
    } else if ( pulse >= LedSwOff ){
      #ifdef DEBUG
        //Serial.println("LED SW Off");
      #endif
      return LED_OFF;
    }
}
boolean getLedAuxSwitch(uint16_t pulse){

    if( pulse <= LedAuxSwOn ) {
      #ifdef DEBUG
        //Serial.println("LED SW On");
      #endif
      return LED_AUX_ON;
    } else if ( pulse >= LedAuxSwOff ){
      #ifdef DEBUG
        //Serial.println("LED SW Off");
      #endif
      return LED_AUX_OFF;
    }
}


void GearDown(){

	//startGearCycle=1;  // Set status flag to indicate we are processing a move cycle
	if ( Armed_Flag ) {
		if ( ! GearDownComplete ) {
			
			if (GearCycleStep > CYCLE_STEPS){
				GearCycleStep = 0;
			}

			if (GearCycleStep < CYCLE_STEPS ){

				int iLed = ServoSeq[1][startGearCycle][3];		// Only used by LED to hold ON or OFF
				int iWait = ServoSeq[1][startGearCycle][2];      // Delay for next action
				int Id   = ServoSeq[1][startGearCycle][1] - 1;  // id from seq array
				int CycleType = ServoSeq[1][startGearCycle][0];		// Will hold SERVO or LED
				
				if ( (millis() - GearTimer) > iWait ) {
			
					if ( startGearCycle < CYCLE_STEPS) {

						if (CycleType == SERVO ) {

							ServoNext_Pos[Id]   = ServoDown_Pos[Id];
							ServoMoveStatus[Id] = 1;
							GearTimer = millis();
							#ifdef DEBUG
							Serial.print("Gear down start seq ");
							Serial.print(" Delay: ");
							Serial.print(iWait);
							Serial.print(" Current Pos: ");
							Serial.print(ServoOld_Pos[Id]);
							Serial.print(" Next Pos: ");
							Serial.print(ServoNext_Pos[Id]);
							Serial.print(" ID: ");
							Serial.println(Id);
							#endif
							
						} else if ( CycleType == LED ) {
							setLed(Id, iLed);							
						}
						++startGearCycle;

					} else {
						// Last cycle has been completed - reset values and set flags
						startGearCycle = 0;  // Reset cycle counter
						GearDownComplete = true;
						GearUpComplete = false;
						GearCurrentState = GEAR_DOWN;
						setNVRAM();
						#ifdef DEBUG
						Serial.println("Gear down finished");
						#endif
/*						for (int j =0; j<6; j++){
							Serial.print("Servo CUR: ");
							Serial.println(ServoCurrent_Pos[j]);
						}*/
						
					}
				}
			} 
		}
	}
}

void GearUp(){

	if ( Armed_Flag ) {
		if (! GearUpComplete) {
			//Serial.println("Gear up triggered");
			//Serial.println(GearCycleStep);
			if (GearCycleStep > CYCLE_STEPS){
				GearCycleStep = 0;
			}

			if (GearCycleStep < CYCLE_STEPS ){

				//int  ServoWait = ServoSeq[0][startGearCycle][2];      // Delay for next action
				//byte ServoId   = ServoSeq[0][startGearCycle][1] - 1;  // Servoid from seq array

				int iLed = ServoSeq[0][startGearCycle][3];	// Only used by LED to hold ON or OFF
				int iWait = ServoSeq[0][startGearCycle][2];      // Delay for next action
				int Id   = ServoSeq[0][startGearCycle][1] - 1;  // Servoid from seq array
				int CycleType = ServoSeq[0][startGearCycle][0];		// Will hold SERVO or LED

				if ( millis() - GearTimer > iWait ) {
					if (startGearCycle < CYCLE_STEPS) {
						if (CycleType == SERVO ) {
							ServoNext_Pos[Id]   = ServoUp_Pos[Id];
							ServoMoveStatus[Id] = 1;
					
							GearTimer = millis();
							#ifdef DEBUG
							Serial.print("Gear up start seq ");
							Serial.print(" Delay: ");
							Serial.print(iWait);
							Serial.print(" Current Pos: ");
							Serial.print(ServoOld_Pos[Id]);
							Serial.print(" Next Pos: ");
							Serial.print(ServoNext_Pos[Id]);
							Serial.print(" ID: ");
							Serial.println(Id);
							#endif
						} else if ( CycleType == LED ) {
							setLed(Id, iLed);
							/*if ( iLed == 1 ) {
								LedOn(Id);
							} else {
								LedOff(Id);
							}*/
						}
	
						++startGearCycle;
					} else {
						startGearCycle   = 0;  // Reset cycle counter
						GearDownComplete = false;
						GearUpComplete   = true;
						GearCurrentState = GEAR_UP;
						setNVRAM();
						Serial.println("Gear Up finished");
						
					}
				}
			}
		}
	}

}

/* Write the current state of the GearCycle to NVRAM */
void getNVRAM(){
	GearCurrentState = EEPROM.read(0);
	DefaultStartState = GearCurrentState;

	if ( GearCurrentState == GEAR_UP) {
		GearDownComplete = false;  
		GearUpComplete   = true;
	} else {
		GearDownComplete = true;  
		GearUpComplete   = false;		
	}
	
	#ifdef DEBUG
		Serial.print("Read DefaultGear state from NVRAM: ");
		Serial.print(GearCurrentState);
	#endif

}

void setNVRAM(){
	EEPROM.write(0,GearCurrentState);
	#ifdef DEBUG
		Serial.print("Write DefaultGear state to NVRAM: ");
		Serial.print(GearCurrentState);
	#endif
}

/*
 * Turn off all LEDs
 */
void InitLed(){
	digitalWrite(LedPinIndex[0], LOW);
	digitalWrite(LedPinIndex[1], LOW);
	digitalWrite(LedPinIndex[2], LOW);
	digitalWrite(LedPinIndex[3], LOW);
	digitalWrite(LedPinIndex[4], LOW);
}
void LedOn(byte bLed){
	//if ((millis()-Led_Timer[bLed] ) > Led_Delay[bLed]){
		digitalWrite(LedPinIndex[bLed], HIGH);
		Led_Timer[bLed] = millis();
	//}
}
void LedOff(byte bLed){
	//if ((millis()-Led_Timer[bLed] ) > Led_Delay[bLed]){
		digitalWrite(LedPinIndex[bLed], LOW);
		Led_Timer[bLed] = millis();
	//}
}

void setLed(int iLed, int iStatus){
	String sText= "";
	if (iStatus == ON) {
		sText = " LED On: ";
		digitalWrite(LedPinIndex[iLed], HIGH);
			
	} else {
		sText = " LED Off: ";
		digitalWrite(LedPinIndex[iLed], LOW);
	}
	Led_Timer[iLed] = millis();
	#ifdef DEBUG
	Serial.print(sText);
	Serial.println(iLed);
	#endif
}