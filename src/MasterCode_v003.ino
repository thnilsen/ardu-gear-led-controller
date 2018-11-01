/*
 * RC Retracts Sequencer and LED controller
 * Code by Thomas Nilsen (thomas.nilsen (at) doc-s (dot) co (dot) uk
 * 
 * @Version 1.5
 * 
 * Code for reading RC RX input based on code by rcarduino.blogspot.com - Duane B
 *
 * Requires the following libraries:
 *
 *    PinChangeInt -  https://github.com/GreyGnome/PinChangeInt
 *    SimpleTimer  -  http://playground.arduino.cc/Code/SimpleTimer
 *
 * Functions of project
 * 7 servo channels
 * 5 led channels
 * 3 RX inputs
 * 	- Input 1 = Gear 
 * 	- Input 2 = LED switch 1
 *	- Input 3 = LED switch 2

 * SERVO 1 = Left inner door
 * SERVO 2 = Right inner door
 * SERVO 3 = Left main retract
 * SERVO 4 = Right main retract
 * SERVO 5 = Retract wheel
 * SERVO 6 = Flap radioator
 * SERVO 7 = Landing Light servo
 *
 *
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
  * 1.a If not, wait till it is but make no movements
  * 2.a If it is, all is ok - set parameters
  * 
  */
  
#include <EEPROM.h>
//#include <PinChangeInt.h>  // https://github.com/GreyGnome/PinChangeInt
#include <SimpleTimer.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <SBUS.h>
#define DEBUG 1

/* These variables defines where the servo will start and end as well as
 * First parameter is for servo1, last for servo 7
 */

#define SERVO1_MIN 1200
#define SERVO1_MAX 1600

#define SERVO2_MIN 1200
#define SERVO2_MAX 1800

#define SERVO3_MIN 1100
#define SERVO3_MAX 1800

#define SERVO4_MIN 1200
#define SERVO4_MAX 1800

#define SERVO5_MIN 988
#define SERVO5_MAX 2012

#define SERVO6_MIN 1200
#define SERVO6_MAX 1800

#define SERVO7_MIN 1200
#define SERVO7_MAX 1800

// If the servo direction needs to be reveres, just swap the position of SERVOx_UP and SERVOx_DOWN in the ServoUp_Pos() and ServoDown_Pos() arrays
int ServoUp_Pos[]   = {SERVO1_MAX, SERVO2_MIN, SERVO3_MAX, SERVO4_MIN, SERVO5_MIN, SERVO6_MIN, SERVO7_MIN}; // Array to hold the startup position of the servo. This will be overwritten if values exists in NVRAM
int ServoDown_Pos[] = {SERVO1_MIN, SERVO2_MAX, SERVO3_MIN, SERVO4_MAX, SERVO5_MAX, SERVO6_MAX, SERVO7_MAX}; // Array to hold the max travel of each servo

#define GEAR_UP     1		// Do not change
#define GEAR_DOWN	0		// Do not change
#define LED			1		// Used for cycle ID of LEDs - Do NOT change
#define SERVO       0		// Used for Cycle ID of servos - Do NOT change
#define OFF         0		// Do not change
#define ON          1		// Do not change


/*
 * First sequence is for gear up. Paramerers are {TYPE, INDEXNUM, DELAY TILL NEXT ACTION, LED ON/OFF}
 * Second sequence is for gear down.
 */
byte DefaultStartState = GEAR_UP;	// Set which default state we should use - this will be overridern by values in NVRAM

#define CYCLE_STEPS 10  // Defines how many servo+LED steps there are for a full cycle

int ServoSeq[2][CYCLE_STEPS][4] = {
                        // Gear Up (Type, Device-Channel, Delay till next event, ON/OFF (applies to LED Only - set to 0 for servo))
                        {
							{SERVO, 7, 1010, 0},
							{SERVO, 6,  510, 0},
							{SERVO, 5, 1000, 0},
							{SERVO, 4,  550, 0},
							{SERVO, 3, 1000, 0},
							{SERVO, 2,  500, 0},
							{SERVO, 1, 1000, 0},
							{LED,   1, 1000, OFF},
							{LED,   2, 1000, OFF},
							{LED,   3, 1000, OFF},
                        },

                        // Gear Down (Type, Device-Channel,  Delay till next event, ON/OFF (applies to LED Only - set to 0 for servo))
                        {
							
							{LED,	3, 1000, ON},
							{LED,	2, 1000, ON},
							{LED,	1, 1000, ON},
							{SERVO, 1, 1000, 0},
							{SERVO, 2,  500, 0},
							{SERVO, 3, 1000, 0},
							{SERVO, 3,  500, 0},
							{SERVO, 5, 1000, 0},
							{SERVO, 6, 1010, 0},
							{SERVO, 7,  510, 0},
                        }};

// Controls how far the servo should move for each cycle. First value = servo1, last = servo 7
byte ServoSteps[]   = {5,   10,   10,   100,   100,    100, 100};  

// Controls how delay between servo step moves in ms. Higher value means slower movements. First value = servo1, last = servo 7
int ServoDelay[]    = {200,  200,   20,   100,   22,    200, 100};  

// Define which LED channels that should always be on an not controlled by RX channels (nav lights etc)
// These should not be listed in the GEAR sequence or in the RX channel controlled LEDs
// Use values 1 to 5. Separate with comman for multiple channels
byte LedAlwaysOn 	= {5};

// Define which LEDs should be controlld by the 2 RX Input. use vaules 1-5
// If you have assinged LEDs to the Gear Up/Down sequence, do not assign them here.
// Use values 1 to 5. Separate with comman for multiple channels
byte RX2_LED[] = {4};
byte RX3_LED[] = {3};


// The following variable sets the cycle for each of the 5 led channels, led ch1 top to led ch5 last
//								ON  	OFF 	ROUNDS DELAY(s)
volatile int LedCycle[5][4] = {
								{15,	 1,		0,    1000},	// LED1
								{13,	42,		0,    0},		// LED2
								{15,	40,		15,    500},	// LED3
								{15,	40,		8,    500},  // LED4
								{16,	40,		8,    500},  // LED5
							  };


byte LedDefault[]   = {OFF, OFF, OFF ,OFF ,OFF};		// Default state of leds. If led is used in gear cycle this setting is ignored
byte LedBlinkRate[] = {10, 10, 10, 10, 10};		// Set the rate of blink per led channel - 0 for no blink.
						
volatile int LedCount 			= 5;	// Number of leds to cycle
volatile byte LedState[5]   	= {OFF, OFF, OFF, OFF, OFF};	// If LED is currently set to be on or off. Controlled by GEAR sequences or RX2 / RX3
volatile byte LedStatus[5] 		= {0, 0, 0, 0, 0};	// 0 = off, 1 = on, 2 = delay
volatile byte LedCycleCount[5] 	= {0, 0, 0, 0, 0};	
volatile byte LedId[5] 			= {1, 2, 3, 4, 5};	// LED->PIN Channel order 
volatile long LedTimer[5] 		= {};	// Used to hold timer in millis for when the last event happened


int GearSwUp[]    = {900, 1400};  // Values in this range will intiate gear up position
int GearSwDown[]  = {1610, 2100};  // Values in this range will  initiate gear down position

// The following variable defines how to read the LED RX Channel. It allows for a 3 position/switch to be used.
// The array needs 6 parameteres, two for each channel of low and high.
// First group - low,high, second group - low, high, 3rd group low, high
// Leave some gap beteen each of the 3 goups
//				  LOW   HIGH  LOW   HIGH  LOW   HIGH
int LedSw[6]    = {900, 1400, 1410, 1800, 1810, 2100};	

int LedSwOn     = 1500;  // Values below this means LEDs is to be enabled
int LedSwOff    = 1610;  // Values below this means LEDs are to be enabled
int LedAuxSwOn  = 1500;  // Used to turn LED 5 on
int LedAuxSwOff = 1610;  // Used to tunr LED 5 off
long Led_Timer[]   = {0, 0 ,0 ,0 ,0};

/* Set up PINs used by the components */
#define SERVO1_PIN 7  // Door1 Servo - Digital port 2
#define SERVO2_PIN 8  // Door2 Servo - Digital port 3
#define SERVO3_PIN 12 // Main retract 1 servo
#define SERVO4_PIN 11 // Main retract 2 servo Digital port 5
#define SERVO5_PIN 10 // Tail/Nose Retract
#define SERVO6_PIN 9  // Radioator flap Door Servo - Digital port 7
#define SERVO7_PIN 13 // Servo for landing light in wing

// PINS assigned to LEDs
#define LED1_PIN 6  // Digital pin controlling LED 1
#define LED2_PIN 5
#define LED3_PIN 4
#define LED4_PIN 3
#define LED5_PIN 2  // This is also connected to the internal LED on the arduino

//#define RX1_PIN A3  // RX Channel input 1 
//#define RX2_PIN A2	// RX Channel input 2
//#define RX3_PIN A1	// RX Channel input 3 

/******* Values below should not need to be modified unless you're altering the code **********/
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

boolean Armed_Flag;	// Will be set to true once init has verified Retract switch is verified in the right position after startup
boolean Gear_Flag;	
boolean Led_Flag;
boolean LedAux_Flag;
boolean LedFlag1, LedFlag2, LedFlag3;

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
long GearTimer, LedAuxTimer;

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

SoftwareSerial mySerial(A3, A2);

SBUS sbus(Serial);  // Connect SBUS signal to RX1 pin

Servo myServo[NUM_SERVOS];	// Generate servo objects

//SimpleTimer timer;
long tim1, tim2, tim3, tim4;
int ServoPosLast[7] = {};

void setup() {
	//delay(2000);
	sbus.begin();
	#ifdef DEBUG
		mySerial.begin(115200);
	#endif
	getNVRAM();	// Read last known state of gear up/down position
	
	//pinMode(RX1_PIN, INPUT);
	//pinMode(RX2_PIN, INPUT);
	//pinMode(RX3_PIN, INPUT);
	
	InitServos();

	tim1=millis();


	// using the PinChangeInt library, attach the interrupts
	// used to read the channels
	//PCintPort::attachInterrupt(RX1_PIN, calcRX1, CHANGE);
  //PCintPort::attachInterrupt(RX2_PIN, calcRX2, CHANGE);
 	//PCintPort::attachInterrupt(RX3_PIN, calcRX3, CHANGE);

	setLedDefault();
  mySerial.println("Init Complete!");
}
// this is timer2, which triggers ever 1ms and processes the incoming SBUS datastream
ISR(TIMER2_COMPA_vect)
{
  sbus.process();
}

void loop() {
	boolean Led_Flag, LedAux_Flag, Gear_Flag;


	for (i =0; i < NUM_SERVOS; i++) {
		MoveServo(i);
	}

	#ifdef DEBUG
		//mySerial.println(i);
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
//	if (bUpdateFlagsShared) {
//		//noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
//
//		// take a local copy of which channels were updated in case we need to use this in the rest of loop
//		bUpdateFlags = bUpdateFlagsShared;
//
//		// in the current code, the shared values are always populated
//		// so we could copy them without testing the flags
//		// however in the future this could change, so lets
//		// only copy when the flags tell us we can.
//
//		if( bUpdateFlags & RX1_FLAG ) {
//		  unRX1In = unRX1InShared;
//		}
//
//		if( bUpdateFlags & RX2_FLAG ) {
//		  unRX2In = unRX2InShared;
//		}
//
//		if( bUpdateFlags & RX3_FLAG ) {
//		  unRX3In = unRX3InShared;
//		}
//
//		// clear shared copy of updated flags as we have already taken the updates
//		// we still have a local copy if we need to use it in bUpdateFlags
//		bUpdateFlagsShared = 0;
//
//		//interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
//		// as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
//		// service routines own these and could update them at any time. During the update, the
//		// shared copies may contain junk. Luckily we have our local copies to work with :-)
//	}

	Gear_Flag   = getGearSwitch(getSBUSPWM(5));
	Led_Flag    = getLedSwitch(getSBUSPWM(6));
	LedAux_Flag = getLedAuxSwitch(getSBUSPWM(7));

	switch (Gear_Flag) {	// RXIn1
	case GEAR_UP:
		  GearUp();
		  break;

	case GEAR_DOWN:
		  GearDown();
		  break;

	}
	
	switch (Led_Flag) {	// RXIn2
	case LED_ON:
		if ( ! LedFlag1 ) {
			setLed(4,ON);
			mySerial.println("LED_FLAG ON");
			LedFlag1 = true;
		}
		break;
	
	case LED_OFF:
		if ( LedFlag1 ) {
			mySerial.println("LED_FLAG OFF");
			LedFlag1 = false;
			setLed(4,OFF);
		}
		break;
		
	}
	
	switch (LedAux_Flag) {	// RXIn3
	case LED_AUX_ON:
		if ( ! LedFlag2 ) {
			mySerial.println("LEDAUX_FLAG ON");
			LedFlag2 = true;
			setLed(3,ON);
		}
		break;
	
	case LED_AUX_OFF:
		if ( LedFlag2 ) {
			mySerial.println("LEDAUX_FLAG OFF");
			LedFlag2 = false;
			setLed(3,OFF);
		}
		break;
		
	}
	
	//checkBlinkLed();
	LedSwitch();

	#ifdef DEBUG
	if (millis()-tim2 > 1000 ) {
	/*	mySerial.print(ServoTimer[6]);
		mySerial.print(" Delay... ");
		mySerial.print(ServoDelay[5]);
		
		
		mySerial.print(" Current... ");
		mySerial.print(ServoCurrent_Pos[6]);
		mySerial.print(" Old... ");
		mySerial.print(ServoOld_Pos[6]);
		mySerial.print(" Next ");
		mySerial.println(ServoNext_Pos[6]);
		 */
    //printSBUSStatus();
    
//		mySerial.print("RX1 In: ");
//		mySerial.print(unRX1In);
//		mySerial.print(" | RX2.In: ");
//		mySerial.print(unRX2In);
//		mySerial.print(" | RX3 In: ");
//		mySerial.println(unRX3In);

		tim2 = millis();
		
	}
	#endif
//
//
//  if(bUpdateFlags & RX2_FLAG)
//  {
//    if(servoSteering.readMicroseconds() != unSteeringIn)
//    {
//      mySerial.println(unSteeringIn);
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
			
/*			if (ServoId == 1 ){
				mySerial.print("ServoID: ");
				mySerial.print(ServoId);
				mySerial.print(" Servo Delay: ");
				mySerial.print(ServoDelay[ServoId]);
				mySerial.print(" Position: ");
				mySerial.print(ServoCurrent_Pos[ServoId]);
				mySerial.print(" Next: ");
				mySerial.print(ServoNext_Pos[ServoId]);
				mySerial.print(" Old: ");
				mySerial.println(ServoOld_Pos[ServoId]);
				
			}*/
			if(NewServoPos == ServoNext_Pos[ServoId]) {
				ServoOld_Pos[ServoId] = NewServoPos;	// Set old pos once the servo has reached is final destination
			}
			
			#ifdef DEBUG

			#endif
		}
	}
}


/*
 * Function will initiate the servo upon start of controller
 */
void InitServos(){
	int tmpPos;
	// Set pulldown on all servo pins to avoid uncontrolled movements
	for (int b = 0; b < NUM_SERVOS; b++){
		pinMode(ServoPinIndex[b], OUTPUT);
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
		myServo[i].attach(ServoPinIndex[i], Servo_Min[i], Servo_Max[i] );	
		myServo[i].writeMicroseconds(tmpPos);
		ServoOld_Pos[i] = tmpPos;
		ServoNext_Pos[i] = tmpPos;
		ServoCurrent_Pos[i] = tmpPos;
		ServoTimer[i] = millis();

		#ifdef DEBUG
		mySerial.print(ServoPinIndex[i]);     
		mySerial.print(" ServoID: ");
		mySerial.print(i);
		mySerial.print(" Servo Pos: ");
		mySerial.println(tmpPos);
		#endif
  }
  
  //FIX init routine to read last current setting in NVRAM and use as base

  
}


//void calcRX1() {
//	// if the pin is high, its a rising edge of the signal pulse, so lets record its value
//	if(digitalRead(RX1_PIN) == HIGH) {
//		ulRX1Start = micros();
//	}  else {
//		// else it must be a falling edge, so lets get the time and subtract the time of the rising edge
//		// this gives use the time between the rising and falling edges i.e. the pulse duration.
//		unRX1InShared = (uint16_t)(micros() - ulRX1Start);
//		// use set the flag to indicate that a new channel signal has been received
//		bUpdateFlagsShared |= RX1_FLAG;
//	}
//}
//void calcRX2() {
//	if(digitalRead(RX2_PIN) == HIGH) {
//		ulRX2Start = micros();
//	} else { 
//		unRX2InShared = (uint16_t)(micros() - ulRX2Start);
//		bUpdateFlagsShared |= RX2_FLAG;
//	}
//}
//void calcRX3() {
//	if(digitalRead(RX3_PIN) == HIGH) {
//		ulRX3Start = micros();
//	} else {
//		unRX3InShared = (uint16_t)(micros() - ulRX3Start);
//		bUpdateFlagsShared |= RX3_FLAG;
//	}
//}

boolean getGearSwitch(uint16_t pulse){
	int iMax, iMin;
	static int GearSwCurPulse;
	
  
	// 1. Check if switch is in default start position - if not abort till it is.
	// 2. Once it is, set Armed_Flag = On
	if ( ! Armed_Flag ) {
		switch (DefaultStartState){
			default:
			case GEAR_UP:
				iMax = iMaxUp;
				iMin = iMinUp;
				GearSwCurPulse = iMaxUp;
         mySerial.print("GearUp    | ");
				break;
			case GEAR_DOWN:
				iMax = iMaxDown;
				iMin = iMinDown;
				GearSwCurPulse = iMaxDown;
        mySerial.print("GearDown     |");
				break;
		}
//   mySerial.print("iMax");
//   mySerial.print(iMaxUp);
//   mySerial.print("  | iMin");
//   mySerial.println(iMinUp);

		/* If pulse is within the parameteres specified, we are good to go and can arm */
		if( pulse >= iMin && pulse <= iMax) {
			Armed_Flag = true;	// Arm controller
			#ifdef DEBUG
				mySerial.println("Controller is armed");
			#endif
		}
	}
	
	/* Check if switch has changed to Gear UP */
	
	if ( Armed_Flag ) {
		/* First check if switch is within UP range */
		if( pulse <= iMaxUp && pulse >= iMinUp && GearCurrentState != GEAR_UP ) {
		
			#ifdef DEBUG
				mySerial.print("Gear SW UP : Current State : ");
				mySerial.println(GearCurrentState);
			#endif
			GearCurrentState = GEAR_UP;
			return GEAR_UP;

		} else if ( pulse <= iMaxDown && pulse >= iMinDown && GearCurrentState != GEAR_DOWN ){

			#ifdef DEBUG
				mySerial.print("Gear SW Down | Current State : ");
				mySerial.println(GearCurrentState);
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
		//mySerial.println("LED SW On");
		return LED_ON;
    } else if ( pulse >= LedSwOff ){
        //mySerial.println("LED SW Off");
		return LED_OFF;
    }
}
boolean getLedAuxSwitch(uint16_t pulse){

    if( pulse <= LedAuxSwOn ) {
        //mySerial.println("LED AUX SW On");
		return LED_AUX_ON;
    } else if ( pulse >= LedAuxSwOff ){      
        //mySerial.println("LED AUX SW Off");
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

				int iLed      = ServoSeq[1][startGearCycle][3];		// Only used by LED to hold ON or OFF
				int iWait     = ServoSeq[1][startGearCycle][2];		// Delay for next action
				int Id        = ServoSeq[1][startGearCycle][1] - 1;	// id from seq array
				int CycleType = ServoSeq[1][startGearCycle][0];		// Will hold SERVO or LED
				
				
			
				if ( startGearCycle < CYCLE_STEPS) {
					if ( (millis() - GearTimer) > iWait ) {
						
						GearTimer = millis();

						if (CycleType == SERVO ) {
							
							ServoNext_Pos[Id]   = ServoDown_Pos[Id];
							ServoMoveStatus[Id] = 1;
							//GearTimer = millis();
							#ifdef DEBUG
							mySerial.print("Gear down start seq ");
							mySerial.print(" Delay: ");
							mySerial.print(iWait);
							mySerial.print(" Current Pos: ");
							mySerial.print(ServoOld_Pos[Id]);
							mySerial.print(" Next Pos: ");
							mySerial.print(ServoNext_Pos[Id]);
							mySerial.print(" ID: ");
							mySerial.println(Id);
							#endif
							
						} else if ( CycleType == LED ) {
							setLed(Id, iLed);							
						}
						
						++startGearCycle;
						
						//mySerial.print("Cycle Down : ");
						//mySerial.print(startGearCycle);
						//mySerial.print(" - CYCLES : ");
						//mySerial.print(CYCLE_STEPS);
						//mySerial.print(" - iWait : ");
						//mySerial.println(iWait);
					}
				} else {
					
					// Last cycle has been completed - reset values and set flags
					startGearCycle = 0;  // Reset cycle counter
					GearDownComplete = true;
					GearUpComplete = false;
					GearCurrentState = GEAR_DOWN;
					setNVRAM();
					#ifdef DEBUG
					mySerial.println("Gear down finished");
					#endif
/*					for (int j =0; j<6; j++){
						mySerial.print("Servo CUR: ");
						mySerial.println(ServoCurrent_Pos[j]);
					}*/
						
				}
					
				
			} 
		}
	}
}

void GearUp(){

	if ( Armed_Flag ) {
		if (! GearUpComplete) {
			//mySerial.println("Gear up triggered");
			//mySerial.println(GearCycleStep);
			if (GearCycleStep > CYCLE_STEPS){
				GearCycleStep = 0;
			}

			if (GearCycleStep < CYCLE_STEPS ){

				//int  ServoWait = ServoSeq[0][startGearCycle][2];      // Delay for next action
				//byte ServoId   = ServoSeq[0][startGearCycle][1] - 1;  // Servoid from seq array

				int iLed      = ServoSeq[0][startGearCycle][3];		// Only used by LED to hold ON or OFF
				int iWait     = ServoSeq[0][startGearCycle][2];     // Delay for next action
				int Id        = ServoSeq[0][startGearCycle][1] - 1; // Servoid from seq array
				int CycleType = ServoSeq[0][startGearCycle][0];		// Will hold SERVO or LED


				if (startGearCycle <= CYCLE_STEPS) {
					if ( millis() - GearTimer > iWait ) {
						GearTimer = millis();
						if (CycleType == SERVO ) {
							ServoNext_Pos[Id]   = ServoUp_Pos[Id];
							ServoMoveStatus[Id] = 1;
					
							//GearTimer = millis();
							#ifdef DEBUG
							mySerial.print("Gear up start seq ");
							mySerial.print(" Delay: ");
							mySerial.print(iWait);
							mySerial.print(" Current Pos: ");
							mySerial.print(ServoOld_Pos[Id]);
							mySerial.print(" Next Pos: ");
							mySerial.print(ServoNext_Pos[Id]);
							mySerial.print(" ID: ");
							mySerial.println(Id);
							#endif
						} else if ( CycleType == LED ) {							
							setLed(Id, iLed);							
						}
	
						++startGearCycle;
						//mySerial.print("Cycle Up : ");
						//mySerial.print(startGearCycle);
						//mySerial.print(" - CYCLES : ");
						//mySerial.println(CYCLE_STEPS);
					}
				} else {
					
					startGearCycle   = 0;  // Reset cycle counter
					GearDownComplete = false;
					GearUpComplete   = true;
					GearCurrentState = GEAR_UP;
					setNVRAM();
					mySerial.println("Gear Up finished");
						
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
		GearUp();
	} else {
		GearDownComplete = true;  
		GearUpComplete   = false;		
		GearDown();
	}
	
	#ifdef DEBUG
		mySerial.print("Read Default Gear state from NVRAM: ");
		mySerial.println(GearCurrentState);
	#endif

}

void setNVRAM(){
	EEPROM.write(0,GearCurrentState);
	#ifdef DEBUG
		mySerial.print("Write DefaultGear state to NVRAM: ");
		mySerial.println(GearCurrentState);
	#endif
}

/*
 * Turn off all LEDs
 */
void InitLed(){
	
	for (int i=0; i < 5; ++i ){
		pinMode(LedPinIndex[i], OUTPUT);
		digitalWrite(LedPinIndex[i], LOW);
	}
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
	LedState[iLed]  = iStatus;
	Led_Timer[iLed] = millis();
	#ifdef DEBUG
	mySerial.print(sText);
	mySerial.println(iLed);
	#endif
}

void setLedDefault(){
	for ( int i=0; i < LedCount ; i++){
		digitalWrite(LedPinIndex[i], LedDefault[i]);
		Led_Timer[i] = millis();
		LedState[i] = LedDefault[i];
		LedTimer[5] = millis();
	}
}

void checkBlinkLed(){
	
	for ( int i=0; i < 5 ; i++){
		if (LedBlinkRate[i] > 0 && LedState[i] == ON) {

			if ( (millis() - Led_Timer[i] > LedBlinkRate[i]) ) {

				if (LedStatus[i] ==  ON) {
					digitalWrite(LedPinIndex[i], LOW);
					LedStatus[i] = OFF;

				} else {
					digitalWrite(LedPinIndex[i], HIGH);
					LedStatus[i] = ON;
				}
				Led_Timer[i] = millis();
			}	
		}
	}
}

void LedSwitch() {
	for ( byte i = 0; i < LedCount; ++i) {

		// Turn off led
		if ( LedState[i] == ON && (LedTimer[i] < millis()-LedCycle[i][0]) ) {
			LedStatus[i] = 0;	// turned off
			digitalWrite(LedId[i], LOW);
			LedState[i] = OFF;
			
			++LedCycleCount[i];
			if (LedCycleCount[i] > LedCycle[i][2]) {
				LedStatus[i] 	 = 2;	// Reached round cycle - next stage -> delay
				LedCycleCount[i] = 0;
			}
			LedTimer[i] = millis();
			
		}
		// LED is in delay mode 
		if ( LedState[i] == OFF && LedStatus[i] == 2 ){
			if ( LedTimer[i] < millis()-LedCycle[i][3] ){
				LedStatus[i] = 0;				
			}
		}
		
		if ( LedState[i] == OFF && (LedTimer[i] < millis()-LedCycle[i][1] ) && LedStatus[i] == 0 ){
			// Turn on led
			digitalWrite(LedId[i], HIGH);
			LedState[i] = ON;
			LedTimer[i] = millis();
			
		} 
	}
}

/* Converts SBUS standard value to PWM standard value where 1500 is center, 1000 is low and 2000 is high */
int getSBUSPWM(int channel) {
  return (5 * sbus.getChannel(channel) / 8 + 880);
}

void printSBUSStatus()
{
  mySerial.print("Ch1  ");
  mySerial.println(getSBUSPWM(1));
  mySerial.print("Ch2  ");
  mySerial.println(getSBUSPWM(2));
  mySerial.print("Ch3  ");
  mySerial.println(getSBUSPWM(3));
  mySerial.print("Ch4  ");
  mySerial.println(getSBUSPWM(4));
  mySerial.print("Ch5  ");
  mySerial.println(getSBUSPWM(5));
  mySerial.print("Ch6  ");
  mySerial.println(getSBUSPWM(6));
  mySerial.print("Ch7  ");
  mySerial.println(getSBUSPWM(7));
  mySerial.print("Ch8  ");
  mySerial.println(getSBUSPWM(8));
  mySerial.print("Ch9  ");
  mySerial.println(getSBUSPWM(9));
  mySerial.print("Ch10 ");
  mySerial.println(getSBUSPWM(10));
  mySerial.print("Ch11 ");
  mySerial.println(getSBUSPWM(11));
  mySerial.print("Ch12 ");
  mySerial.println(getSBUSPWM(12));
  mySerial.print("Ch13 ");
  mySerial.println(getSBUSPWM(13));
  mySerial.print("Ch14 ");
  mySerial.println(getSBUSPWM(14));
  mySerial.print("Ch15 ");
  mySerial.println(getSBUSPWM(15));
  mySerial.print("Ch16 ");
  mySerial.println(getSBUSPWM(16));
  mySerial.println();
  mySerial.print("Failsafe: ");
  if (sbus.getFailsafeStatus() == SBUS_FAILSAFE_ACTIVE) {
    mySerial.println("Active");
  }
  if (sbus.getFailsafeStatus() == SBUS_FAILSAFE_INACTIVE) {
    mySerial.println("Not Active");
  }

  mySerial.print("Data loss on connection: ");
  mySerial.print(sbus.getFrameLoss());
  mySerial.println("%");

  mySerial.print("Frames: ");
  mySerial.print(sbus.getGoodFrames());
  mySerial.print(" / ");
  mySerial.print(sbus.getLostFrames());
  mySerial.print(" / ");
  mySerial.println(sbus.getDecoderErrorFrames());

  mySerial.print("Time diff: ");
//  mySerial.println(millis() - sbus.getLastTime());
}
