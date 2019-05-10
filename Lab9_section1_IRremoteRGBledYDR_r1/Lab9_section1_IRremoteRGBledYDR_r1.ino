// Lab9_section1_IRremoteRGBledYDR_r1.ino
// Clark Hochgraf
// April 16, 2017

#include <IRremote.h>  // can conflict with timerTone.cpp.o (symbol from plugin): In function `timer0_pin_port': 
//(.text+0x0): multiple definition of `__vector_7'
// tone needs timer resources and so do PING and IR remote and PWM for the motor.
// We don't use the PING library in this code.

#define IR_RECV_PIN A3 // connect IR remote receiver to this pin

/* IR remote control codes - define symbols UP / DOWN to corresponding IR codes 0xFF629D */
#define UP  0xFF629D
#define DOWN 0xFFA857
#define LEFT 0xFF22DD
#define RIGHT 0xFFC23D
#define CENTER 0xFF02FD
#define ONE 0xFF6897
#define TWO 0xFF9867
#define THREE 0xFFB04F
#define REPEATCODE 0xFFFFFFFF
#define SHOWCODES true  // use for debug to show raw codes from IR remote

/*  a list of all the codes corresponding to buttons on the remote */
/*
  case 0xFF629D: Serial.println(" FORWARD"); break;
  case 0xFF22DD: Serial.println(" LEFT");    break;
  case 0xFF02FD: Serial.println(" -OK-");    break;
  case 0xFFC23D: Serial.println(" RIGHT");   break;
  case 0xFFA857: Serial.println(" REVERSE"); break;
  case 0xFF6897: Serial.println(" 1");    break;
  case 0xFF9867: Serial.println(" 2");    break;
  case 0xFFB04F: Serial.println(" 3");    break;
  case 0xFF30CF: Serial.println(" 4");    break;
  case 0xFF18E7: Serial.println(" 5");    break;
  case 0xFF7A85: Serial.println(" 6");    break;
  case 0xFF10EF: Serial.println(" 7");    break;
  case 0xFF38C7: Serial.println(" 8");    break;
  case 0xFF5AA5: Serial.println(" 9");    break;
  case 0xFF42BD: Serial.println(" *");    break;
  case 0xFF4AB5: Serial.println(" 0");    break;
  case 0xFF52AD: Serial.println(" #");    break;
  case 0xFFFFFFFF: Serial.println(" REPEAT");break;
*/

long previousIRcommand = 0;
IRrecv irrecv(IR_RECV_PIN);
decode_results IR_command_received; //create a variable named IR_command_received of type decode_results

// RGB Led data and clock pins on the arduino
#define LED_DATA_PIN 12
#define LED_CLOCK_PIN 11

/* define RGB LED colors */
#define DIM_WHITE_COLOR 0x1F1F1F
#define DIM_GREEN_COLOR 0x001F00
#define DIM_RED_COLOR 0x1F0000
#define DIM_BLUE_COLOR 0x00001F
#define DIM_RED_BLUE_COLOR 0x1F001F
#define DIM_RED_GREEN_COLOR 0x1F1F00
#define DIM_GREEN_BLUE_COLOR 0x001F1F
#define MED_BLUE_COLOR 0x0000FF
// Timers
unsigned long timerInMsec = 0;
float timeStepS = 0.01;

// declare a new type of data (directionState_type), with enumerated values
enum directionState_type {FORWARD, BACKWARD, LEFT_TURN, RIGHT_TURN, STOPPED, SCAN_FOR_NEAREST_OBJECT,
                          TURN_TO_NEAREST_OBJECT, CALIBRATE_MOTOR_STICTION};
                          
directionState_type directionState = CALIBRATE_MOTOR_STICTION; // create a variable named directionState
directionState_type previousDirectionState = STOPPED; // set previous direction state to an impossible value
boolean isNewState = true;

void setup() {
  Serial.begin(115200);
  Serial.println("I'm alive - RGB led with IR remote control");

  irrecv.enableIRIn(); // Start the receiver
  pinMode(IR_RECV_PIN, INPUT_PULLUP); // try this to improve noise immunity (may not matter).
  delay(1000);

  StripInit();
  display_color_on_RGB_led(DIM_BLUE_COLOR);

  Serial.println("done with setup");
  
} //setup

//================================================================
void loop() {
  timerInMsec = millis();
  
  get_IR_command_if_any(); // this may update the directionState
  
  isNewState = ( directionState != previousDirectionState); // if state != prevState, then isNewState

  switch (directionState) {
    case FORWARD:
      //entry housekeeping
      if (isNewState) {
        Serial.println(F("new state is FORWARD"));
        display_color_on_RGB_led(DIM_GREEN_COLOR);
      }
      //state business
      
      //exit housekeeping
      break;

    case LEFT_TURN:
      //entry housekeeping
      if (isNewState) {
        Serial.println(F("new state is LEFT_TURN")); 
        display_color_on_RGB_led(DIM_GREEN_BLUE_COLOR);
      }
      //state business

      //exit housekeeping
      break;

    case RIGHT_TURN:
      //entry housekeeping
      if (isNewState) {
        Serial.println(F("new state is RIGHT_TURN"));
        display_color_on_RGB_led(DIM_RED_GREEN_COLOR);
      }
      //state business

      //exit housekeeping
      break;

    case BACKWARD:
      //entry housekeeping
      if (isNewState) {
        Serial.println(F("new state is BACKWARD"));
        display_color_on_RGB_led(DIM_RED_BLUE_COLOR);
      }
      //state business

      //exit housekeeping
      break;

    case STOPPED:
      //entry housekeeping
      if (isNewState) {

        Serial.println(F("new state is STOPPED")); 
        display_color_on_RGB_led(DIM_RED_COLOR);
      }
      //state business

      //exit housekeeping
      break;

    case SCAN_FOR_NEAREST_OBJECT:
      //entry housekeeping
      if (isNewState) {
        Serial.println(F("new state is SCAN_FOR_NEAREST_OBJECT"));
        display_color_on_RGB_led(DIM_WHITE_COLOR);
       }
      //state business

      //exit housekeeping
      break;

    case TURN_TO_NEAREST_OBJECT:
      //entry housekeeping
       if (isNewState) {
          Serial.println(F("new state is TURN_TO_NEAREST_OBJECT"));
          display_color_on_RGB_led(DIM_RED_COLOR);
        }
  
      //state business

      //exit housekeeping
      break;

    case CALIBRATE_MOTOR_STICTION:
      //entry housekeeping
      if (isNewState) {
        Serial.println(F("State is CALIBRATE_MOTOR_STICTION")); 
        display_color_on_RGB_led(MED_BLUE_COLOR);
      }
      //state business

      //exit housekeeping
 
      break;

    default:
      break;

  }//switch

  previousDirectionState =  directionState;

} // loop()



//==============================================================
// Initialize the pins used to control the WS2801 driver
void StripInit(void)
{
  pinMode(LED_DATA_PIN, OUTPUT);
  pinMode(LED_CLOCK_PIN, OUTPUT);
  digitalWrite(LED_DATA_PIN, LOW);
  digitalWrite(LED_CLOCK_PIN, LOW);
}


//==============================================================
void get_IR_command_if_any()
{
  if (irrecv.decode(&IR_command_received)) { // takes up to 6 ms to run

    if (SHOWCODES) Serial.println(IR_command_received.value, HEX);
    if (IR_command_received.value == REPEATCODE) { // key held down, repeat last command
      IR_command_received.value = previousIRcommand;
      if (SHOWCODES) {
        Serial.print("repeating command IR_command_received.value ");
        Serial.println(IR_command_received.value, HEX);
      }
    }
    if (IR_command_received.value == LEFT)     directionState = LEFT_TURN;
    if (IR_command_received.value == CENTER)   directionState = STOPPED;
    if (IR_command_received.value == RIGHT)    directionState = RIGHT_TURN;
    if (IR_command_received.value == UP)       directionState = FORWARD;
    if (IR_command_received.value == DOWN)     directionState = BACKWARD;
    if (IR_command_received.value == ONE)      directionState = SCAN_FOR_NEAREST_OBJECT;
    if (IR_command_received.value == TWO)      directionState = TURN_TO_NEAREST_OBJECT;
    if (IR_command_received.value == THREE)    directionState = CALIBRATE_MOTOR_STICTION;

    previousIRcommand = IR_command_received.value;
    //Serial.println(directionState);
    irrecv.resume(); // Receive the next value
    //Serial.print("elapsed time "); Serial.println(millis()-starttime);
  }
}

// =======================================================================
void display_color_on_RGB_led(unsigned long color) {
  unsigned long bitmask = 0UL; // UL unsigned long literal (forces compiler to use long data type)
  unsigned long masked_color_result = 0UL;
  digitalWrite(LED_CLOCK_PIN, LOW); //start with clock low.
  for (int i = 23; i >= 0; i--) { // clock out one data bit at a time, starting with the MSB first
    bitmask = (1UL << i); // build bit mask. Note must use "1UL" unsigned long literal, not "1"
    masked_color_result = color & bitmask; // reveals just one bit of color at time
    boolean data_bit = !(masked_color_result == 0); // this is the bit of data to be clocked out.
    digitalWrite(LED_DATA_PIN, data_bit);
    digitalWrite(LED_CLOCK_PIN, HIGH);
    digitalWrite(LED_CLOCK_PIN, LOW);
  }
  digitalWrite(LED_CLOCK_PIN, HIGH);
  delayMicroseconds(501); // after writing data to LED driver, must hold clock line
  // high for 500 microseconds to latch color data in led shift register.
}//display_color_on_RGB_led()
