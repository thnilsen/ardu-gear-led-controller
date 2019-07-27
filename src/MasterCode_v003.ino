
/*
   RC Retracts Sequencer and LED controller
   Code by Thomas Nilsen (thomas.nilsen (at) doc-s (dot) co (dot) uk

   @Version 1.5

   Code for reading RC RX input based on code by rcarduino.blogspot.com - Duane B

   Requires the following libraries:

      PinChangeInt -  https://github.com/GreyGnome/PinChangeInt
      SimpleTimer  -  http://playground.arduino.cc/Code/SimpleTimer

   Functions of project
   7 servo channels
   5 led channels
   3 RX inputs
  	- Input 1 = Gear
  	- Input 2 = LED switch 1
 	- Input 3 = LED switch 2

   SERVO 1 = Left inner door
   SERVO 2 = Right inner door
   SERVO 3 = Left main retract
   SERVO 4 = Right main retract
   SERVO 5 = Retract wheel
   SERVO 6 = Flap radioator
   SERVO 7 = Landing Light servo


   1. Check if we have RX change
   1.1 Set RX Change flag and time
   2. If different, set Move flag
   3. Walk array for new positions
   3.1 Set new position for servo
   3.2 Set old position for servo ???
   4. Set statusflag servo move = 1 to prevent other operations
   5. Ease servo over time to new position
   6. Check new position - against current position, and when hit, set flag to 0
   7. Update old position array to mach new
   8 Clear statusflag move

  Old = 1000		New = 2000
  Diff = +1000

  Old = 2000		New = 1000
  Diff = -1000

    Switch ON check
    1. Is switch in the default start position
    1.a If not, wait till it is but make no movements
    2.a If it is, all is ok - set parameters

*/

#include <EEPROM.h>
#include <SimpleTimer.h>
#include <Wire.h>
//#include <Servo.h>
#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h>
#include <SBUS.h>

//#define DEBUG              1
//#define DEBUG_SBUS         1
#define INPUT_CHANNEL_GEAR 7     // Select which SBUS channel controls the gear
#define INPUT_CHANNEL_LED1 15 // Select which SBus channel controls LED OPT 1 - Landing Light
#define INPUT_CHANNEL_LED2 16 // Select which SBus channel controls LED OPT 2
#define SERVO_FREQUENCY    50  // Hz for servos - standard is 50
#define NUM_SERVOS         8

/* These variables defines where the servo will start and end as well as
   First parameter is for servo1, last for servo 7                       */
// Servo Range on the PWM controller is about 200 - 400 - 300 midle.
#define SERVO_MIN  800
#define SERVO_MAX  2200


// Define which LEDs should be controlld by the 2 RX Input. use vaules 1-5
// If you have assinged LEDs to the Gear Up/Down sequence, do not assign them here.
// Use values 1 to 5. Separate with comman for multiple channels
#define LED_OPT1 2 // Which LED channel to use for INPUT_CHANNEL_OPT1
#define LED_OPT2 5  // Which LED channel to use for INPUT_CHANNEL_OPT2

// If the servo direction needs to be reveres, just swap the position of SERVOx_UP and SERVOx_DOWN in the ServoUp_Pos() and ServoDown_Pos() arrays
//int ServoUp_Pos[]   = {SERVO1_MAX, SERVO2_MIN, SERVO3_MAX, SERVO4_MIN, SERVO5_MIN, SERVO6_MIN, SERVO7_MIN, SERVO8_MIN}; // Array to hold the startup position of the servo. This will be overwritten if values exists in NVRAM
//int ServoDown_Pos[NUM_SERVOS] = {SERVO1_MIN, SERVO2_MAX, SERVO3_MIN, SERVO4_MAX, SERVO5_MAX, SERVO6_MAX, SERVO7_MAX, SERVO8_MAX}; // Array to hold the max travel of each servo
int ServoUp_Pos[NUM_SERVOS]   = {};
int ServoDown_Pos[NUM_SERVOS] = {};

#define GEAR_UP     1		// Do not change
#define GEAR_DOWN   0		// Do not change
#define LED         1		// Used for cycle ID of LEDs - Do NOT change
#define SERVO       0		// Used for Cycle ID of servos - Do NOT change
#define OFF         0		// Do not change
#define ON          1		// Do not change
#define STEADY      2   // Will be used for always on LEDs
#define CYCLE       3   // Used to set LED to cycle mode (on for x ms, off for x ms, delay x ms before repeat process - useful for guns)
/*
   First sequence is for gear up. Paramerers are {TYPE, INDEXNUM, DELAY TILL NEXT ACTION, LED ON/OFF}
   Second sequence is for gear down.
*/
byte DefaultStartState = GEAR_UP;	// Set which default state we should use - this will be overridern by values in NVRAM

#define CYCLE_STEPS 6  // Defines how many servo+LED steps there are for a full cycle

int ServoSeq[2][CYCLE_STEPS][4] = {
  // Gear Down (Type, Device-Channel, Delay till next event, ON/OFF (for LED) or new POSITION for servos)
  {

  //  {LED,   4, 0, OFF},
   // {LED,   2, 0, OFF},
    

    {SERVO, 1,    0, 1920}, // Left inner gear door
    {SERVO, 2,    0, 1020}, // Right inner gear door
    {SERVO, 5, 1000, 1300}, // Tail doors
    {SERVO, 3, 2000, 1900}, // Left retract
    {SERVO, 4, 1000, 1900}, // Right retract
    {SERVO, 6,    0, 1700}, // Tail retract    
    //{LED,   2,    0, ON},
  },

  // Gear Up (Type, Device-Channel,  Delay till next event, ON/OFF (applies to LED Only - set to 0 for servo))
  {

    //{LED,   1, 0, ON},

    {SERVO, 6,    0, 1400}, // Tail retract
    {SERVO, 3, 1000, 1000}, // Left retract
    {SERVO, 4, 1000, 1000}, // Right retract
    {SERVO, 1, 8000,  995}, // Left gear door
    {SERVO, 2,  300, 1958}, // Right gear door    
    {SERVO, 5, 1000, 1650}, // Tail doors
    //{LED,   2,    0,  OFF},
   // {LED, 4, 0, ON},

  }
};

// Controls how far the servo should move for each cycle. First value = servo1, last = servo 8
byte ServoSteps[]   = {20, 10, 100, 100, 100, 100, 100, 100};

// Controls how delay between servo step moves in ms. Higher value means slower movements. First value = servo1, last = servo8
int ServoDelay[]    = {50, 20, 20, 100, 22, 200, 100, 100};

// Define which LED channels that should always be on an not controlled by RX channels (nav lights etc)
// These should not be listed in the GEAR sequence or in the RX channel controlled LEDs
// Use values 1 to 5. Separate with comman for multiple channels
byte LedAlwaysOn 	= {1};



// The following variable sets the cycle for each of the 5 led channels, led ch1 top to led ch5 last
//
// TYPE definitions are
//      OFF - Can be turned on and off by switch. All other parameters are ignored (but some value must exist)
//      STEADY - Led will turn on at power-on and remain on, ignoring switch settings.  All other parameters are ignored (but some value must exist)
//      CYCLE - LED will blink/cycle according to the on/off/rounds/delay parameters being set.
//  To get a normal blink type led, set the ON, OFF and DELAY to the same value, ie 100. Set Rounds to 0
//

volatile int LedCycle[5][5] = {
  // TYPE, ON(ms), OFF(ms),ROUNDS, DELAY(ms)
  {STEADY,  15,	   100,		  0,     1000},	// LED1
  {OFF,    100,	   100,		  0,      100},	// LED2
  {OFF,     15,	    40,		 15,      500},	// LED3
  {STEADY,  15,	    40,		  8,      500}, // LED4
  {CYCLE,   15,	    50,		  8,     1000}, // LED5
};


//byte LedDefault[]   = {OFF, OFF, OFF ,OFF ,OFF};		// Default state of leds. If led is used in gear cycle this setting is ignored
//byte LedBlinkRate[] = {10, 10, 10, 10, 200};       // Set the rate of blink per led channel - 0 for no blink.

volatile int LedCount 			    = 5;	// Number of leds to cycle
volatile byte LedState[5]   	  = {OFF, OFF, OFF, OFF, OFF};	// If LED is currently set to be on or off. Controlled by GEAR sequences or RX2 / RX3
volatile byte LedStatus[5] 		  = {0, 0, 0, 0, 0};	// 0 = off, 1 = on, 2 = delay
volatile byte LedStateCycle[5]  = {};  // Will be used for status checking on LEDs that are enabled for Cycle
volatile byte LedCycleCount[5] 	= {0, 0, 0, 0, 0};
//volatile byte LedId[5] 			  = {1, 2, 3, 4, 5};	// LED->PIN Channel order
volatile long LedTimer[5] 		  = {};	// Used to hold timer in millis for when the last event happened


int GearSwUp[]    = {900, 1400};  // Values in this range will intiate gear up position
int GearSwDown[]  = {1610, 2100};  // Values in this range will  initiate gear down position

// The following variable defines how to read the LED RX Channel. It allows for a 3 position/switch to be used.
// The array needs 6 parameteres, two for each channel of low and high.
// First group - low,high, second group - low, high, 3rd group low, high
// Leave some gap beteen each of the 3 goups
//				        LOW   HIGH  LOW   HIGH  LOW   HIGH
int LedSw[6]    = {900, 1400, 1410, 1800, 1810, 2100};


int LedSwOn[2]     = {1600,2000};  // LED OPT1 Values below this means LEDs is to be enabled
int LedSwOff[2]    = {1000,1450};  // LED OPT1 Values below this means LEDs are to be enabled
int LedAuxSwOn[2]  = {1000,1450};  // LED OPT2 Used to turn LED OPT2 on
int LedAuxSwOff[2] = {1610,2000};  // LED OPT2 Used to turn LED OPT2 off
long Led_Timer[]   = {0, 0 , 0 , 0 , 0};
unsigned long ArmedTimeNow, ArmedTimePrev;

/* Set up PINs used by the components */
#define SERVO1_PIN 0  // Inner Gear door left Servo 
#define SERVO2_PIN 1  // inner Gear door right Servo 
#define SERVO3_PIN 2  // Main retract left servo
#define SERVO4_PIN 3  // Main retract right servo 
#define SERVO5_PIN 4  // Tail/Nose Retract
#define SERVO6_PIN 5  // Radioator flap Door Servo
#define SERVO7_PIN 6  // Tail Gear door
#define SERVO8_PIN 7  // Servo for landing light in wing

// PINS assigned to LEDs
#define LED1_PIN 3  // Digital pin controlling LED 1
#define LED2_PIN 4
#define LED3_PIN 5
#define LED4_PIN 6
#define LED5_PIN 2  // This is also connected to the internal LED on the arduino

//#define LED_ON      1
//#define LED_OFF     0
//#define LED_AUX_ON  1
//#define LED_AUX_OFF 0

//#define SERVO_REFRESH 30  // How many ms to wait between servo updates


// Values below should not need to be changed...
//int ServoPinIndex[]    = {SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN, SERVO5_PIN, SERVO6_PIN, SERVO7_PIN, SERVO8_PIN };
int ServoCurrent_Pos[] = {0, 0, 0, 0, 0, 0, 0, 0}; // Array to hold the last known position of the servo
int ServoOld_Pos[]     = {0, 0, 0, 0, 0, 0, 0, 0}; // Contains the previously used position
int ServoNext_Pos[]    = {0, 0, 0, 0, 0, 0, 0, 0}; // Will hold where the servo is to move to
int Servo_Min[]        = {0, 0, 0, 0, 0, 0, 0, 0};
int Servo_Max[]        = {0, 0, 0, 0, 0, 0, 0, 0};
long ServoTimer[]      = {0, 0, 0, 0, 0, 0, 0, 0}; // Holds millis() for when the last servo move was completed. Used to delay the servo movement
byte ServoMoveStatus[] = {0, 0, 0, 0, 0, 0, 0, 0}; // Will be set to 1 when servo is in process


byte LedPinIndex[] = {LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN, LED5_PIN};

boolean Armed_Flag;	// Will be set to true once init has verified Retract switch is verified in the right position after startup
boolean Armed_Blinked; // Will be set once armed LEDs has flashed once
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
unsigned long GearTimer, LedAuxTimer;

SoftwareSerial mySerial(A3, A2);  // Define software serial ports
SBUS sbus(Serial);  // Connect SBUS signal to RX1 pin
//Servo myServo[NUM_SERVOS];	// Generate servo objects

Adafruit_PWMServoDriver myServo = Adafruit_PWMServoDriver();


//SimpleTimer timer;
unsigned long tim1, tim2, tim3, tim4;
int ServoPosLast[7] = {};

#ifdef DEBUG    //Macros are usually in all capital letters.
#define DEB(...)    mySerial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
#define DEBLN(...)  mySerial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
#define DEB(...)     //now defines a blank line
#define DEBLN(...)   //now defines a blank line
#endif

void setLed(int iLed, int iStatus, boolean bShowMsg=false);

void setup() {

  
  #ifdef DEBUG
  mySerial.begin(115200);
  DEBLN("Starting..");
  #endif

  sbus.begin();
  getNVRAM();	// Read last known state of gear up/down position
  
  myServo.begin();
  myServo.setPWMFreq(50);

  InitServos();

  tim1 = millis();
  tim3 = millis();
  DEBLN("Init Complete!");

}

// this is timer2, which triggers ever 1ms and processes the incoming SBUS datastream
ISR(TIMER2_COMPA_vect) {
  sbus.process();
}

void loop() {

  for (i = 0; i < NUM_SERVOS; i++) {
    MoveServo(i);
  }

  // Get RX channe pulses
  Gear_Flag   = getGearSwitch(getSBUSPWM(INPUT_CHANNEL_GEAR));
  Led_Flag    = getLedSwitch(getSBUSPWM(INPUT_CHANNEL_LED1));
  LedAux_Flag = getLedAuxSwitch(getSBUSPWM(INPUT_CHANNEL_LED2));

  if ( sbus.getFailsafeStatus() ) {
    DEB("Failsafe!");
    Gear_Flag = GEAR_DOWN;  // Force gear down...
  }

  switch (Gear_Flag) {
    case GEAR_UP:
      GearUp();
      break;

    case GEAR_DOWN:
      GearDown();
      break;

  }
  if ( Armed_Flag && Armed_Blinked ) {
    switch (Led_Flag) {
      case ON:
        if ( ! LedFlag1 ) {
          setLed(LED_OPT1 - 1, ON);
          DEBLN("LED_FLAG ON");
          LedFlag1 = true;
        }
        break;
  
      case OFF:
        if ( LedFlag1 ) {
          DEBLN("LED_FLAG OFF");
          LedFlag1 = false;
          setLed(LED_OPT1 - 1, OFF);
        }
        break;
  
    }
  
    switch (LedAux_Flag) {
      case ON:
        if ( ! LedFlag2 ) {
          DEBLN("LEDAUX_FLAG ON");
          LedFlag2 = true;
          setLed(LED_OPT2 - 1, ON);
        }
        break;
  
      case OFF:
        if ( LedFlag2 ) {
          DEBLN("LEDAUX_FLAG OFF");
          LedFlag2 = false;
          setLed(LED_OPT2 - 1, OFF);
        }
        break;
  
    }
  }
  // Trigger LED warning lights for armed/non-armed status
  if (! Armed_Flag ) { 
    ArmedTimeNow = millis();
    BlinkLedWarnArmed(); 
  }
  
  if ( Armed_Flag && !Armed_Blinked ) {
      BlinkLedOKArmed();
      Armed_Blinked = true;
  }  
  
  if ( Armed_Flag && Armed_Blinked) { LedSwitch(); }
  

#ifdef DEBUG
  if (millis() - tim2 > 3000 ) {
    /*	DEB(ServoTimer[6]);
    	DEB(" Delay... ");
    	DEB(ServoDelay[5]);


    	DEB(" Current... ");
    	DEB(ServoCurrent_Pos[6]);
    	DEB(" Old... ");
    	DEB(ServoOld_Pos[6]);
    	DEB(" Next ");
    	DEBLN(ServoNext_Pos[6]);
    */
    #ifdef DEBUG_SBUS
    printSBUSStatus();
    #endif

    tim2 = millis();

  }
#endif
}

void MoveServo(int ServoId) {
  int NewServoPos, tmpPos;
  if ( ServoOld_Pos[ServoId] != ServoNext_Pos[ServoId] ) {
  
    if ( (millis() - ServoTimer[ServoId]) > ServoDelay[ServoId] ) {
      /* Check direction of movement - but only when the servo has yet to be moved */

      if ((ServoNext_Pos[ServoId] - ServoOld_Pos[ServoId]) < 0 ) {
        tmpPos = ServoCurrent_Pos[ServoId] - ServoSteps[ServoId];
      } else {
        tmpPos = ServoCurrent_Pos[ServoId] + ServoSteps[ServoId];
      }

      NewServoPos = constrain(tmpPos, Servo_Min[ServoId], Servo_Max[ServoId]);
      ServoCurrent_Pos[ServoId] = NewServoPos;
      myServo.setPWM(ServoId, 0, pulseWidth(ServoCurrent_Pos[ServoId], Servo_Min[ServoId], Servo_Max[ServoId] ));
      ServoTimer[ServoId] = millis();
      
//		if (ServoId == 1 ){
//				DEB("ServoID: ");
//				DEB(ServoId);
//				DEB(" Servo Delay: ");
//				DEB(ServoDelay[ServoId]);
//        DEB(" Min:");
//        DEB(Servo_Min[ServoId]);
//        DEB(" Max: ");
//        DEB(Servo_Max[ServoId]);
//				DEB(" Position: ");
//				DEB(ServoCurrent_Pos[ServoId]);
//				DEB(" Next: ");
//				DEB(ServoNext_Pos[ServoId]);
//				DEB(" Old: ");
//				DEBLN(ServoOld_Pos[ServoId]);
//
//			}
      if (NewServoPos == ServoNext_Pos[ServoId]) {
        ServoOld_Pos[ServoId] = NewServoPos;	// Set old pos once the servo has reached is final destination
      }
    }
  }
}


/*
   Function will initiate the servo upon start of controller
*/
void InitServos() {
  int tmpPos;

  findServoMinMax();

  // Attach servos to arduino
  for (int i = 0; i < NUM_SERVOS; i++) {
    /* Define the defaults startup position for each servo */
    if ( DefaultStartState == GEAR_UP ) {
      // Read last entry per servo in sequence and use value
      tmpPos = ServoUp_Pos[i];
    } else {
      tmpPos = ServoDown_Pos[i];
    }
    if ( tmpPos == 0 ) { tmpPos = 1500; } // Make sure we have values for all servo channels - and if not set default to center 1500ms
    myServo.setPWM(i, 0, pulseWidth(tmpPos, Servo_Min[i], Servo_Max[i] ));
    ServoOld_Pos[i] = tmpPos;
    ServoNext_Pos[i] = tmpPos;
    ServoCurrent_Pos[i] = tmpPos;
    ServoTimer[i] = millis();

    #ifdef DEBUG
    //DEB(ServoPinIndex[i]);
    DEB("ServoID: ");
    DEB(i);
    DEB(" Pos: ");
    DEBLN(tmpPos);
    #endif
  }

  // TODO : Fix init routine to read last current setting in NVRAM and use as base
}

boolean getGearSwitch(uint16_t pulse) {
  int iMax, iMin;
  static int GearSwCurPulse;


  // 1. Check if switch is in default start position - if not abort till it is.
  // 2. Once it is, set Armed_Flag = On
  if ( ! Armed_Flag ) {
    switch (DefaultStartState) {
      default:
      case GEAR_UP:
        iMax = iMaxUp;
        iMin = iMinUp;
        GearSwCurPulse = iMaxUp;
        //DEB("GearUp    | ");
        break;
      case GEAR_DOWN:
        iMax = iMaxDown;
        iMin = iMinDown;
        GearSwCurPulse = iMaxDown;
        //DEB("GearDown     |");
        break;
    }
    //   DEB("iMax");
    //   DEB(iMaxUp);
    //   DEB("  | iMin");
    //   DEBLN(iMinUp);

    /* If pulse is within the parameteres specified, we are good to go and can arm */
    if ( pulse >= iMin && pulse <= iMax) {
      Armed_Flag = true;	// Arm controller      
      #ifdef DEBUG
      DEBLN("Controller is armed");
      #endif
    }
  }

  /* Check if switch has changed to Gear UP */

  if ( Armed_Flag ) {
    /* First check if switch is within UP range */
    if ( pulse <= iMaxUp && pulse >= iMinUp && GearCurrentState != GEAR_UP ) {


      DEB("GearSW UP : Curr State: ");
      DEB(GearCurrentState);

      GearCurrentState = GEAR_UP;
      return GEAR_UP;

    } else if ( pulse <= iMaxDown && pulse >= iMinDown && GearCurrentState != GEAR_DOWN ) {
      DEB("GearSW Dwn | Curr State: ");
      DEBLN(GearCurrentState);

      GearCurrentState = GEAR_DOWN;
      return GEAR_DOWN;
    }
    GearSwCurPulse = pulse;
    return GearCurrentState;

  }

}
boolean getLedSwitch(uint16_t pulse) {

  if ( intRange(pulse, LedSwOn[0], LedSwOn[1] ) ) {
    //DEBLN("LED SW On");
    return ON;
  } else if ( pulse >= LedSwOff ) {
    //DEBLN("LED SW Off");
    return OFF;
  }
}
boolean getLedAuxSwitch(uint16_t pulse) {

  if ( intRange(pulse, LedAuxSwOn[0], LedAuxSwOn[1]) ) {
    //DEBLN("LED AUX SW On");
    return ON;
  } else if ( pulse >= LedAuxSwOff ) {
    //DEBLN("LED AUX SW Off");
    return OFF;
  }
}


void GearDown() {

  //startGearCycle=1;  // Set status flag to indicate we are processing a move cycle
  if ( Armed_Flag ) {
    if ( ! GearDownComplete ) {

      if (GearCycleStep > CYCLE_STEPS) {
        GearCycleStep = 0;
      }

      if (GearCycleStep < CYCLE_STEPS ) {

        int iValue    = ServoSeq[1][startGearCycle][3];		// Only used by LED to hold ON or OFF
        int iWait     = ServoSeq[1][startGearCycle][2];		// Delay for next action
        int Id        = ServoSeq[1][startGearCycle][1] - 1;	// id from seq array
        int CycleType = ServoSeq[1][startGearCycle][0];		// Will hold SERVO or LED



        if ( startGearCycle < CYCLE_STEPS) {
          if ( (millis() - GearTimer) > iWait ) {

            GearTimer = millis();

            if (CycleType == SERVO ) {

              ServoNext_Pos[Id]   = iValue;   //ServoDown_Pos[Id];
              ServoMoveStatus[Id] = 1;
              //GearTimer = millis();

              #ifdef DEBUG
              DEB("Gear dwn start seq ");
              DEB(" Delay: ");
              DEB(iWait);
              DEB(" Current Pos: ");
              DEB(ServoOld_Pos[Id]);
              DEB(" Next Pos: ");
              DEB(ServoNext_Pos[Id]);
              DEB(" Min:");
              DEB(Servo_Min[Id]);
              DEB(" Max: ");
              DEB(Servo_Max[Id]);
              DEB(" ID: ");
              DEBLN(Id);
              #endif

            } else if ( CycleType == LED ) {
              setLed(Id, iValue);
            }

            ++startGearCycle;

            //DEB("Cycle Down : ");
            //DEB(startGearCycle);
            //DEB(" - CYCLES : ");
            //DEB(CYCLE_STEPS);
            //DEB(" - iWait : ");
            //DEBLN(iWait);
          }
        } else {

          // Last cycle has been completed - reset values and set flags
          startGearCycle = 0;  // Reset cycle counter
          GearDownComplete = true;
          GearUpComplete = false;
          GearCurrentState = GEAR_DOWN;
          setNVRAM();
          DEBLN("Gear down finished");

          /*					for (int j =0; j<6; j++){
          						DEB("Servo CUR: ");
          						DEBLN(ServoCurrent_Pos[j]);
          					}*/

        }


      }
    }
  }
}

void GearUp() {

  if ( Armed_Flag ) {
    if (! GearUpComplete) {
      //DEBLN("Gear up triggered");
      //DEBLN(GearCycleStep);
      if (GearCycleStep > CYCLE_STEPS) {
        GearCycleStep = 0;
      }

      if (GearCycleStep < CYCLE_STEPS ) {

        //int  ServoWait = ServoSeq[0][startGearCycle][2];      // Delay for next action
        //byte ServoId   = ServoSeq[0][startGearCycle][1] - 1;  // Servoid from seq array

        int iValue    = ServoSeq[0][startGearCycle][3];		// Only used by LED to hold ON or OFF
        int iWait     = ServoSeq[0][startGearCycle][2];     // Delay for next action
        int Id        = ServoSeq[0][startGearCycle][1] - 1; // Servoid from seq array
        int CycleType = ServoSeq[0][startGearCycle][0];		// Will hold SERVO or LED


        if (startGearCycle < CYCLE_STEPS) {
          if ( millis() - GearTimer > iWait ) {
            GearTimer = millis();
            if (CycleType == SERVO ) {
              ServoNext_Pos[Id]   = iValue ;    //ServoUp_Pos[Id];
              ServoMoveStatus[Id] = 1;

              //GearTimer = millis();
              #ifdef DEBUG
              DEB("Gear up start seq ");
              DEB(" Delay: ");
              DEB(iWait);
              DEB(" Current Pos: ");
              DEB(ServoOld_Pos[Id]);
              DEB(" Next Pos: ");
              DEB(ServoNext_Pos[Id]);
              DEB(" Min:");
              DEB(Servo_Min[Id]);
              DEB(" Max: ");
              DEB(Servo_Max[Id]);
              DEB(" ID: ");
              DEBLN(Id);
              #endif
            } else if ( CycleType == LED ) {
              setLed(Id, iValue);
            }

            ++startGearCycle;
            //DEB("Cycle Up : ");
            //DEB(startGearCycle);
            //DEB(" - CYCLES : ");
            //DEBLN(CYCLE_STEPS);
          }
        } else {

          startGearCycle   = 0;  // Reset cycle counter
          GearDownComplete = false;
          GearUpComplete   = true;
          GearCurrentState = GEAR_UP;
          setNVRAM();
          DEBLN("Gear Up finished");

        }

      }
    }
  }

}

/*
   Find the maximum and minimum values stored for servo movements
   Found max and min values per servoid will be stored to Servo_Max[] and Servo_Min[]
*/
void findServoMinMax()
{
  int i = 0;
  int j = 0;
  int iServo = 0;
  int iValues[CYCLE_STEPS] = {}; // Store servo movements to find max and min. Probably enough to use half size?

  // Start with looping over each possible servo ID
  while (iServo < NUM_SERVOS) {
//    cout << "\n *** iServo: ";
//    cout << iServo;
//    cout << "\n";
    clearArr(iValues, CYCLE_STEPS);  // Reset values to make sure we don't inherit when the next servoid is being processed
    j = 0;
    i = 0;

    // With each servo ID, loop over the cycles specified in the sequence
    while (i < CYCLE_STEPS) {

      // Check GearUp section
      if ((ServoSeq[0][i][0] == SERVO) && (ServoSeq[0][i][1] == iServo + 1)) {
        iValues[j] = ServoSeq[0][i][3];
        ServoUp_Pos[iServo] = ServoSeq[0][i][3];
//        cout << "GUp ";
//        cout << iValues[j];
//        cout << "\n";
        j++;
      }

      // Now check GearDown section
      if ((ServoSeq[1][i][0] == SERVO) && (ServoSeq[1][i][1] == iServo + 1)) {
        iValues[j] = ServoSeq[1][i][3];
        ServoDown_Pos[iServo] = ServoSeq[1][i][3];
//        cout << "GDwn ";
//        cout << iValues[j];
//        cout << "\n";
        j++;
      }
      DEB("Ser: ");
      DEB(iServo);
      DEB(" Dwn: ");
      DEB(ServoDown_Pos[iServo]);
      DEB(" Up: ");
      DEBLN(ServoUp_Pos[iServo]);
     
      i++;
    }

    //printArrn(iValues);
    Servo_Max[iServo] = getMax(iValues, CYCLE_STEPS);
    Servo_Min[iServo] = getMin(iValues, CYCLE_STEPS);
    
    DEB("servo; ");
    DEB(iServo);
    DEB(" max value found: ");
    DEB(Servo_Max[iServo]);
    DEB(" | min value found: ");
    DEBLN(Servo_Min[iServo]);
    

    iServo++;
  }
}


/* Write the current state of the GearCycle to NVRAM */
void getNVRAM() {
  GearCurrentState = EEPROM.read(0);
  DefaultStartState = GearCurrentState;

  if ( GearCurrentState == GEAR_UP) {
    GearDownComplete = false;
    GearUpComplete   = true;
    GearUp();
  } else {
    GearDownComplete = true;
    GearUpComplete   = false;
    DefaultStartState = GEAR_DOWN;
    GearDown();
  }

  DEB("Read Default Gear state from NVRAM: ");
  DEBLN(DefaultStartState);

}

void setNVRAM() {
  EEPROM.write(0, GearCurrentState);
  DEB("Write DefaultGear state to NVRAM: ");
  DEBLN(GearCurrentState);

}

/*
   Turn off all LEDs
*/
void InitLed() {

  for (int i = 0; i < 5; ++i ) {
    pinMode(LedPinIndex[i], OUTPUT);
    digitalWrite(LedPinIndex[i], LOW);
  }
}
/*
   Not in use!!!
  void LedOn(byte bLed){
	//if ((millis()-Led_Timer[bLed] ) > Led_Delay[bLed]){
		digitalWrite(LedPinIndex[bLed], HIGH);
		Led_Timer[bLed] = millis();
    if ( bLed == 4 ) { DEBLN("5 hit LedOn"); }
	//}
  }
  void LedOff(byte bLed){

	//if ((millis()-Led_Timer[bLed] ) > Led_Delay[bLed]){
		digitalWrite(LedPinIndex[bLed], LOW);
		Led_Timer[bLed] = millis();
   if ( bLed != 4 ) { DEBLN("5 hit LedOff"); }
	//}
  }
*/

void setLed(int iLed, int iStatus, boolean bShowMsg = false) {
  String sText = "";
  if (iStatus == ON) {
    sText = " LED On ID: ";
    digitalWrite(LedPinIndex[iLed], HIGH);
  } else {
    sText = " LED Off ID: ";
    digitalWrite(LedPinIndex[iLed], LOW);
  }
  LedState[iLed]  = iStatus;
  Led_Timer[iLed] = millis();
  if ( bShowMsg) {
    #ifdef DEBUG
    DEB(sText);
    DEB(iLed + 1);
    DEB(" - PIN: ");
    DEBLN(LedPinIndex[iLed]);  
    #endif
  }
}

//void setLedDefault(){
//	for ( int i=0; i < LedCount ; i++){
//		digitalWrite(LedPinIndex[i], LedDefault[i]);
//		Led_Timer[i] = millis();
//		LedState[i] = LedDefault[i];
//		//LedTimer[5] = millis();
//	}
//}

/*

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
*/

void LedSwitch() {
  for ( byte i = 0; i < LedCount; ++i) {

    // Check if LED is enabled for cycling on - otherwise ignore
    if ( LedCycle[i][0] == CYCLE && LedState[i] == ON) {

      // Turn off led
      if ( LedStateCycle[i] == ON && (LedTimer[i] < millis() - LedCycle[i][1]) ) {
        LedStatus[i] = 0;	// turned off
        digitalWrite(LedPinIndex[i], LOW);
        LedStateCycle[i] = OFF;

        ++LedCycleCount[i];
        if (LedCycleCount[i] > LedCycle[i][3]) {
          LedStatus[i] 	 = 2;	// Reached round cycle - next stage -> delay
          LedCycleCount[i] = 0;
        }
        LedTimer[i] = millis();


      }
      // LED is in delay mode
      if ( LedStateCycle[i] == OFF && LedStatus[i] == 2 ) {
        if ( LedTimer[i] < millis() - LedCycle[i][4] ) {
          LedStatus[i] = 0;
        }

      }

      if ( LedStateCycle[i] == OFF && (LedTimer[i] < millis() - LedCycle[i][2] ) && LedStatus[i] == 0 ) {
        // Turn on led
        digitalWrite(LedPinIndex[i], HIGH);
        LedStateCycle[i] = ON;
        LedTimer[i] = millis();

      }

    } else if ( LedCycle[i][0] == STEADY ) {
      // Set an always on LED
      if (LedState[i] != ON ) {
        setLed(i, ON);
        DEB("LED ID STEADY: ");
        DEBLN(i + 1); // Add 1 to get human count rather than starting from 0
      }
    }
  }
}

/* Converts SBUS standard value to PWM standard value where 1500 is center, 1000 is low and 2000 is high */
int getSBUSPWM(int channel) {
  return (5 * sbus.getChannel(channel) / 8 + 880);
}

#ifdef DEBUG_SBUS
void printSBUSStatus()
{
  DEB("Ch1  ");
  DEB(getSBUSPWM(1));
  DEB(" Ch2  ");
  DEB(getSBUSPWM(2));
  DEB(" Ch3  ");
  DEB(getSBUSPWM(3));
  DEB(" Ch4  ");
  DEB(getSBUSPWM(4));
  DEB(" Ch5  ");
  DEBLN(getSBUSPWM(5));
  DEB("Ch6  ");
  DEB(getSBUSPWM(6));
  DEB(" Ch7  ");
  DEB(getSBUSPWM(7));
  DEB(" Ch8  ");
  DEB(getSBUSPWM(8));
  DEB(" Ch9  ");
  DEB(getSBUSPWM(9));
  DEB(" Ch10 ");
  DEBLN(getSBUSPWM(10));
  DEB("Ch11 ");
  DEB(getSBUSPWM(11));
  DEB(" Ch12 ");
  DEB(getSBUSPWM(12));
  DEB(" Ch13 ");
  DEB(getSBUSPWM(13));
  DEB(" Ch14 ");
  DEB(getSBUSPWM(14));
  DEB(" Ch15 ");
  DEB(getSBUSPWM(15));
  DEB(" Ch16 ");
  DEBLN(getSBUSPWM(16));
  DEBLN();
  DEB("F/S:");
  if (sbus.getFailsafeStatus() == SBUS_FAILSAFE_ACTIVE) {
    DEBLN("Act");
  }
  if (sbus.getFailsafeStatus() == SBUS_FAILSAFE_INACTIVE) {
    DEBLN("Dis");
  }

  DEB("Data loss con:");
  DEB(sbus.getFrameLoss());
  DEBLN("%");

  DEB("Frms: ");
  DEB(sbus.getGoodFrames());
  DEB(" / ");
  DEB(sbus.getLostFrames());
  DEB(" / ");
  DEBLN(sbus.getDecoderErrorFrames());

  //DEB("Time diff: ");
  //  DEBLN(millis() - sbus.getLastTime());
}
#endif

// Origin code by http://wiki.sunfounder.cc/index.php?title=PCA9685_16_Channel_12_Bit_PWM_Servo_Driver
// Maps standard servo pwm values to PCA9685 related values
int pulseWidth(int pulseInValue, int ServoMinPulse, int ServoMaxPulse)
{
  int pulse_wide, analog_value;
  pulse_wide   = map(pulseInValue, 900, 2100, ServoMinPulse, ServoMaxPulse);
  analog_value = int(float(pulse_wide) / 1000000 * SERVO_FREQUENCY * 4096);
  /*DEB("Pulse_wide: ");
    DEB(pulse_wide);
    DEB(" | analog value");
    DEBLN(analog_value);*/
  return analog_value;
}

/*
   Clear array
*/
void clearArr(int* iValues, int length)
{
  for (int i = 0; i < length; i++) {
    iValues[i] = 0;
  }
}
/*
   Get maximum value from array
   @param int iValue - array of integer for values to check
   @param int sizeArr - size of integer array (number of elements)
   @return int maximum value found - but withing limits of SERVO_MAX constant
*/
int getMax(int* iValue, int sizeArr) {
  int i, max;

  max = iValue[0];
  if (max > SERVO_MAX || max == 0) {
    max = SERVO_MAX;
  }
  //index = 0;

  for (i = 1; i < sizeArr; i++) {
    if ( (iValue[i] > max) && (iValue[i] != 0) ) {
      max = iValue[i];
    }

  }
  if ( max > SERVO_MAX) {
    max = SERVO_MAX;  // Value found to be above maximum default, overriding value to default MAX
  }


  return max;
}

/*
  Get minimum value from array
  @param int iValue - array of integer for values to check
  @param int sizeArr - size of integer array (number of elements)
  @return int minimum value found - but withing limits of SERVO_MAX constant
*/

int getMin(int* iValue, int sizeArr) {
  int i, min;

  min = iValue[0];

  for (i = 1; i < sizeArr; i++) {
    if ( (iValue[i] < min) && (iValue[i] != 0) ) {
      min = iValue[i];
    }
  }
  if (min < SERVO_MIN) {
    min = SERVO_MIN;  // Value found to be below minimum default, overriding value to default MIN
  }

  return min;
}
/*
 * This functino will blink LED1 evert 400ms 6 times to indicated that the arming process has completed
 */
void BlinkLedOKArmed(){
  byte i;
  if ( Armed_Flag ) {
    for ( i = 0; i < 5; ++i) {
      setLed(0, ON, false);
      delay(200);    
      setLed(0, OFF, false);
      delay(400);
    }  
    Armed_Blinked = true;
  }
}

/*
 * This functino will blink LED1 ever 80ms till arming is complete.
 */
void BlinkLedWarnArmed() {
  static byte BlinkLedState = OFF;
  
  if ( ArmedTimeNow - ArmedTimePrev >= 80 ) {
    ArmedTimePrev = ArmedTimeNow;
    if ( BlinkLedState == OFF ) {
      BlinkLedState = ON;
    } else {
      BlinkLedState = OFF;
    }
    setLed(0, BlinkLedState, false);
  }  

}
/*
 *
*/
bool intRange(int checkValue, int minimum, int maximum )
{
  return ( (minimum <= checkValue) && (checkValue <= maximum) );
}

