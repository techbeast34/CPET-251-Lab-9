// Lab9_BTControlledRGBled.ino
// Written by Clark Hochgraf <cghiee@rit.edu> & Hari Krishna Kandasamy <hkkiee@rit.edu>
// Date: April 8, 2018
// Revised:  
// Description: ++ PLEASE READ ALL COMMENTS ++
//       This program enable bluetooth communication to an Arduino using an HC-05 module.
//       


#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu; // declare a variable called mpu of datatype MPU6050
unsigned long timeStampStartOfLoopMs = 0;
float timeStepS = 0.01;
float pitch,roll,yaw = 0.0f; // pitch, roll and yaw values
Vector normalizedGyroDPS; //stores the three gyroscope readings XYZ in degrees per second (DPS)


/*  FOR BLUETOOTH */
#include <SoftwareSerial.h>
SoftwareSerial BT(A0, A1);  // Pin A0 directly connected to BT TX
                            // Pin A1 directly connected to BT RX 
                            // Keeps TX and RX (1 and 0) available for Serial Monitor

int BT_value;

//D-Pad
#define UP       49 // ASCII Character 1. Forwards
#define DOWN     50 // ASCII Character 2. Backwards
#define LEFT     51 // ASCII Character 3. Left
#define RIGHT    52 // ASCII Character 4. Right

//Action Buttons
#define TRIANGLE 53 // ASCII Character 5. scan for nearest object
#define X_BUTTON 54 // ASCII Character 6. turn to nearest object
#define SQUARE   55 // ASCII Character 7. calibrate motor stiction
#define CIRCLE   56 // ASCII Character 8. Stop
#define REPEATCODE 0
#define SHOWCODES true  // use for debug to show raw value from Bluetooth


// Data and clock pin locations on the arduino for RGB LED Strip
#define LED_DATA_PIN 12
#define LED_CLOCK_PIN 11

//RGB LED color information
#define DIM_WHITE_COLOR 0x1F1F1F
#define DIM_GREEN_COLOR 0x001F00
#define DIM_RED_COLOR 0x1F0000
#define DIM_BLUE_COLOR 0x00001F
#define DIM_RED_BLUE_COLOR 0x1F001F
#define DIM_RED_GREEN_COLOR 0x1F1F00
#define DIM_GREEN_BLUE_COLOR 0x001F1F
#define MED_BLUE_COLOR 0x0000FF

enum directionState_t {FORWARD, BACKWARD, LEFT_TURN, RIGHT_TURN, STOPPED, SCAN_FOR_NEAREST_OBJECT,
                       TURN_TO_NEAREST_OBJECT, CALIBRATE_MOTOR_STICTION}; // declare a new type of data, with defined values

directionState_t directionState = STOPPED; // create a variable named directionState of type
                                           // directionState_t, set the variable to a value of stopped.
                       
directionState_t previousDirectionState = -1;
boolean isNewState = true;

const int AB_mtr_INA_PIN = 10;
const int AB_mtr_INB_PIN = 9;
const int CD_mtr_INC_PIN = 8;
const int CD_mtr_IND_PIN = 7;

const int PWM_AB_PIN = 6;  
const int PWM_CD_PIN = 5; 

int MotorABpwmOffset = 0;
int MotorCDpwmOffset = 0;

boolean calibrating_motor_stiction = false;
//================================================================

void setup() {
  BT.begin(9600);  // for bluetooth
  Serial.begin(115200);
  Serial.println("I'm alive with BT");

  StripInit();
  display_color_on_RGB_led(DIM_BLUE_COLOR);

  Serial.println("done with setup");

} //setup

//================================================================
void loop() {

  //Function call to receive data over Bluetooth connection
  get_BT_command_if_any();

  // update state information
  isNewState = ( directionState != previousDirectionState); // if state != prevState, then isNewState

  switch (directionState) {
    case FORWARD:
      //entry housekeeping
      if (isNewState) {
        Serial.println(F("new state is FORWARD"));
        display_color_on_RGB_led(DIM_GREEN_COLOR);
      }
      //state business
      go_forward(120);
      //exit housekeeping
      break;

    case LEFT_TURN:
      //entry housekeeping
      if (isNewState) {
        Serial.println(F("new state is LEFT_TURN")); 
        display_color_on_RGB_led(DIM_GREEN_BLUE_COLOR);
      }
      //state business
      turn_counterclockwise(120);
      //exit housekeeping
      break;

    case RIGHT_TURN:
      //entry housekeeping
      if (isNewState) {
        Serial.println(F("new state is RIGHT_TURN"));
        display_color_on_RGB_led(DIM_RED_GREEN_COLOR);
      }
      //state business
      turn_clockwise(120);
      //exit housekeeping
      break;

    case BACKWARD:
      //entry housekeeping
      if (isNewState) {
        Serial.println(F("new state is BACKWARD"));
        display_color_on_RGB_led(DIM_RED_BLUE_COLOR);
      }
      //state business
      go_backward(120);
      //exit housekeeping
      break;

    case STOPPED:
      //entry housekeeping
      if (isNewState) {
        Serial.println(F("new state is STOPPED")); 
        display_color_on_RGB_led(DIM_RED_COLOR);
      }
      //state business
      stop_motor();
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
        MotorABpwmOffset = find_motorstiction_using_gyro(AB_mtr_INA_PIN, AB_mtr_INB_PIN, PWM_AB_PIN);
        display_color_on_RGB_led(DIM_GREEN_COLOR);
        delay(2000);
        MotorCDpwmOffset = find_motorstiction_using_gyro(CD_mtr_INC_PIN, CD_mtr_IND_PIN, PWM_CD_PIN);
        display_color_on_RGB_led(MED_BLUE_COLOR);
        delay(2000);
      }
      
      //state business

      
      //exit housekeeping
      directionState = STOPPED;
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

//==============================================================
void get_BT_command_if_any()
{
    if (BT.available()) { 
      BT_value = BT.read();
      if (SHOWCODES) Serial.println(BT_value);
    //if (results.value == REPEATCODE) { // key held down, repeat last command
      //results.value=previousIRcommand; 
      //if (SHOWCODES) {
          //Serial.print("repeating command results.value "); Serial.println(results.value, HEX);
      //}
      if (BT_value == LEFT)     directionState = LEFT_TURN;
      if (BT_value == CIRCLE)   directionState = STOPPED;
      if (BT_value == RIGHT)    directionState = RIGHT_TURN;
      if (BT_value == UP)       directionState = FORWARD;
      if (BT_value == DOWN)     directionState = BACKWARD;
      if (BT_value == TRIANGLE) directionState = SCAN_FOR_NEAREST_OBJECT;
      if (BT_value == X_BUTTON) directionState = TURN_TO_NEAREST_OBJECT;
      if (BT_value == SQUARE)   directionState = CALIBRATE_MOTOR_STICTION;   

    if (SHOWCODES) Serial.println(directionState);
    }
}
//================================================================
void go_forward(int rate) {
  digitalWrite(AB_mtr_INA_PIN, 0);
  digitalWrite(AB_mtr_INB_PIN, 1);
  digitalWrite(CD_mtr_INC_PIN, 0);
  digitalWrite(CD_mtr_IND_PIN, 1);
  analogWrite(PWM_AB_PIN, constrain(MotorABpwmOffset + rate, 0, 255));
  analogWrite(PWM_CD_PIN, constrain(MotorCDpwmOffset + rate, 0, 255));
}

void go_backward(int rate) {
  digitalWrite(AB_mtr_INA_PIN, 1);
  digitalWrite(AB_mtr_INB_PIN, 0);
  digitalWrite(CD_mtr_INC_PIN, 1);
  digitalWrite(CD_mtr_IND_PIN, 0);
  analogWrite(PWM_AB_PIN, constrain(MotorABpwmOffset + rate, 0, 255));
  analogWrite(PWM_CD_PIN, constrain(MotorCDpwmOffset + rate, 0, 255));
}

void turn_clockwise(int rate) {
  digitalWrite(AB_mtr_INA_PIN, 0);
  digitalWrite(AB_mtr_INB_PIN, 1);
  digitalWrite(CD_mtr_INC_PIN, 1);
  digitalWrite(CD_mtr_IND_PIN, 0);
  analogWrite(PWM_AB_PIN, constrain(MotorABpwmOffset + rate, 0, 255));
  analogWrite(PWM_CD_PIN, constrain(MotorCDpwmOffset + rate, 0, 255));
}

void turn_counterclockwise(int rate) {
  digitalWrite(AB_mtr_INA_PIN, 1);
  digitalWrite(AB_mtr_INB_PIN, 0);
  digitalWrite(CD_mtr_INC_PIN, 0);
  digitalWrite(CD_mtr_IND_PIN, 1);
  analogWrite(PWM_AB_PIN, constrain(MotorABpwmOffset + rate, 0, 255));
  analogWrite(PWM_CD_PIN, constrain(MotorCDpwmOffset + rate, 0, 255));
}

void stop_motor() {
  digitalWrite(AB_mtr_INA_PIN, 0);
  digitalWrite(AB_mtr_INB_PIN, 0);
  digitalWrite(CD_mtr_INC_PIN, 0);
  digitalWrite(CD_mtr_IND_PIN, 0);
  analogWrite(PWM_AB_PIN, 0);
  analogWrite(PWM_CD_PIN, 0);
}
//================================================
int find_motorstiction_using_gyro(int fwd_pin, int rev_pin, int pwm_pin){
  calibrating_motor_stiction = true;

  int pwm_value_sent_to_motor = 0;
  Serial.println("Calibrating motor....");
//  PORTB |= 0x02;
//  PORTB &= 0xFB;
//  PORTD |= 0x80;
    digitalWrite(fwd_pin,1);
    digitalWrite(rev_pin,0);
  
  while(calibrating_motor_stiction){
    Serial.println(pwm_value_sent_to_motor);
    normalizedGyroDPS = mpu.readNormalizeGyro();
    analogWrite(pwm_pin, pwm_value_sent_to_motor++);
    
    //delay(5);
    
    if(abs(normalizedGyroDPS.ZAxis) > 10){ 
      calibrating_motor_stiction = false;
      analogWrite(pwm_pin, 0);
      //Serial.println(normalizedGyroDPS.ZAxis * timeStepS); 
    }

    if (pwm_value_sent_to_motor > 250) //failed calibration
    { Serial.print(F("Calibration failed. Check if battery is connected/switched on."));
      analogWrite(pwm_pin, 0);
      while (1) {
        display_color_on_RGB_led(DIM_RED_COLOR);
        delay(100);
        display_color_on_RGB_led(DIM_BLUE_COLOR);
        delay(100);
      }
    }
  }
  return pwm_value_sent_to_motor;
}
