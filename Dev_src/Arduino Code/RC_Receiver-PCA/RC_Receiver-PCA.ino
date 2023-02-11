/*************************************************** 
 * File Name: RC_Receiver-PCA-v18.ino
 * Updated: 2023-01-13
 * Usage: Transfer RC Receiver's PWM values into Serial CMD
 * Pins: 
 *  D0-Steering
 *  D1-Throttle
 *  D2-SDA
 *  D3-SCL
 *  D8-Clutch
 *  D9-Mode
 * Functions:
 *  1.Read from RC Receiver
 *  2.Mode 1 driven by racer with RC Controller
 *  3.CH3 enabled to start and cut off Throttle
 *  4.Controller losing protection updated
 *  5.CH4 enabled Mode Switch among User, Auto Angle, Auto Pilot 
 *  6.CH4 added LED blinking for easy understanding
 *  7.Comments added for better understanding
 *  8.Serial Return added for Upper Controller learning 
 *  9.fix Serial Receive bugs to accommodate Serial Sending style
 * 10.Sepreate working role for different Mode 
 * 11.Receive CMD for Steering and throttle at same time
 * 12.Soft-Serial enabled for SerialUSB Debug with PIN 8 & 9
 * 13.Move PIN_CLUTCH and PIN_MODE to Pin 4 & 5
 * 14.Report RC Controller CMD without blocking Serial
 * 15.Fix RC Receive block issue with threading control
 * 16.Keep Throttle lower than 80
 * 17.Exchange between Steering & Throttle for output
 * 18.Adding while(!serial) to waiting for serial port ready.
 * 19.Check sending buffer is available. 
 * 20.Delete SoftwareSerial for stable performance.
 * 21.Move ServoInput to highest loop for better reaction.
 * 22.Add HT_Min and TH_Max especially for Throttle Control.
***************************************************/ 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ServoInput.h>


/* The signal pin for ServoInput MUST be an interrupt-capable pin!
 *     Uno, Nano, Mini (328P): 2, 3
 *     Micro, Leonardo (32U4): 0, 1, 2, 3, 7
 *             Mega, Mega2560: 2, 3, 18, 19, 20, 21
 * Reference: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 */

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ  50 // Analog servos run at ~50 Hz updates
#define SERVO_MIN  200 // 'minimum' pulse length count (out of 4096)
#define SERVO_MAX  400 // 'maximum' pulse length count (out of 4096)
#define SERVO_MID  310 // 'medium' pulse length count (out of 4096)
#define MOTOR_RANGE 60 // Pulse range for Motor Throttle
#define MOTOR_BACKBOOST 0 // Pulse range for Motor Throttle
#define PIN_SERVO      0
#define PIN_MOTOR      1
#define PIN_CLUTCH     4
#define PIN_MODE       5
#define PIN_SSerial_RX 8
#define PIN_SSerial_TX 9
#define PIN_LED      LED_BUILTIN // Default:P13

ServoInputPin<PIN_SERVO> steering;
ServoInputPin<PIN_MOTOR> throttle;

const int ST_Min = 1; // Minimum PWM range for Steering and Throttle
const int ST_Max = 4095; // Maximum PWM range for Steering and Throttle

const int TH_Min = 1; // Minimum PWM range for Steering and Throttle
const int TH_Max = 4095; // Maximum PWM range for Steering and Throttle

//const int TH_Min = 950; // Minimum PWM range for Steering and Throttle
//const int TH_Max = 1950; // Maximum PWM range for Steering and Throttle

const int CM_Min = 1300; // Minimum PWM range for Clutch and Mode
const int CM_Max = 1600; // Maximum PWM range for Clutch and Mode
const int C_Range = 1; // Clutch varies range
const int M_Range = 3; // Mode varies range
const float steering_deadzone = 0.05;  // 5%
const float throttle_deadzone = 0.01;  // 1%

int steering_val_serial;
int throttle_val_serial;
int steering_val_joystick;
int throttle_val_joystick;
int clutch_val;
int clutch_status;
int mode_val;
int mode_status;
int channel;
int steering_pulse;
int throttle_pulse;
int temp_pulse;
int ledState = LOW;           // ledState used to set the LED
long LED_interval = 1000;     // interval at which to blink (milliseconds)
unsigned long previousMillis; // will store last time LED was updated
unsigned long counter;        // count loops for programme running
String CMD = "";
String CMD_steering = "";
String CMD_throttle = "";
String result = "";
String result_temp_TS = "";
String result_temp_T = "";
String result_temp_MC = "";
bool mode_user = false;
bool mode_angle = false;
bool mode_pilot = false;

void setup() {

  delay(500);
  steering.setRange(ST_Min, ST_Max);
  throttle.setRange(TH_Min, TH_Max);

  CMD.reserve(20);
  CMD_steering.reserve(10);
  CMD_throttle.reserve(10);
  
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_CLUTCH, INPUT);
  pinMode(PIN_MODE, INPUT);

  // Open serial communications and wait for port to open
	
  Serial.begin(38400);
  
   // waiting for the serial port up and ready.
  /*
  while (! Serial) {
     ;
   }
 */

  Serial.setTimeout(10);                                                                                                                                                            

  pwm.begin();
  // pwm.setOscillatorFrequency(27000000); // crystal frequency 
  pwm.setOscillatorFrequency(25000000); // crystal frequency 
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(100);
  pwm.setPWM(PIN_SERVO, 0, SERVO_MID);
  delay(100);
  pwm.setPWM(PIN_MOTOR, 0, SERVO_MID);
  delay(100);
}

void LED_Blink(long LED_Delay){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= LED_Delay) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(PIN_LED, ledState);
  }
}

void loop() {
  steering_val_joystick = steering.mapDeadzone(-100, 100, steering_deadzone);
  throttle_val_joystick = throttle.mapDeadzone(-100, 100, throttle_deadzone);
	
  //============================================= Driving CMD from RC Receiver
  if(counter % 5 == 0) // check per 5 loops to save time amonge pulseIn()
  {
    if(mode_status == 1){
      result = "";
      result += "T";
	
      result += (throttle_val_joystick); // Range [-100,100] 
      result += "S";

      result += (steering_val_joystick); // Range [-100,100] 

      if(Serial.availableForWrite() > 8) { 
          Serial.println(result); // Report to Upper Controller
      }

    }
    else if(mode_status == 2){
      // throttle_val_joystick = throttle.mapDeadzone(-100, 100, throttle_deadzone);

      result = "";
      result += "T";
      result += (throttle_val_joystick); // Range [-100,100] 
    
      if(Serial.availableForWrite() > 8) { 
         Serial.println(result); // Report to Upper Controller
      }
    }
  }

  if(counter % 100 == 0) // check per 100 loops to save time amonge pulseIn()
  {
    clutch_val = pulseIn(PIN_CLUTCH, HIGH, 25000);
    clutch_status = map(clutch_val,CM_Min,CM_Max,0,C_Range);
    mode_val = pulseIn(PIN_MODE, HIGH, 25000);
    mode_status = map(mode_val,CM_Min,CM_Max,1,M_Range);
    mode_status = max(1,mode_status);
    mode_status = min(M_Range,mode_status);
    
    result = "";
    result += "M";
    result += mode_status;
    result += "C";
    result += clutch_status;

    if(result_temp_MC != result) {
      // Report to Upper Controller
      if(Serial.availableForWrite() >= 8) 
      {
        Serial.println(result); // Report to Upper Controller
      }
      result_temp_MC = result;
    }
      
  }

  //============================================= Driving CMD from Upper Controller
  if (Serial.available()){
    CMD = Serial.readStringUntil('\n');
  
    int CMD_gap = CMD.indexOf(':');
    int CMD_len = CMD.length();
    CMD_steering = CMD;

    CMD_throttle = CMD;
    CMD_steering.remove(CMD_gap,-1);
    CMD_throttle.remove(0,CMD_gap+1);
  
    steering_val_serial = CMD_steering.toInt();
    steering_val_serial = min(steering_val_serial,80);
    throttle_val_serial = CMD_throttle.toInt();
  
    steering_pulse = map(steering_val_serial,-100,100,SERVO_MIN,SERVO_MAX);
    throttle_pulse = map(throttle_val_serial,-100,100,SERVO_MID-MOTOR_RANGE-MOTOR_BACKBOOST,SERVO_MID+MOTOR_RANGE);
  
    result = "";
    result += " S:";
    result += CMD_steering;
    result += " T:";
    result += CMD_throttle;
  
  }

  //============================================= Executing Driving CMD
  if(mode_pilot == true){  // Stop Once when mode changed from USER
      steering_pulse = SERVO_MID;
      throttle_pulse = SERVO_MID;
      mode_pilot = false;
      
  }
  if (mode_status == 1){ // Mode-1 will take steering and throttle values from RC Receiver
    mode_user = true;
    LED_interval = 1000; // LED blinking fast to represent User Mode
    steering_pulse = map(steering_val_joystick,-100,100,SERVO_MIN,SERVO_MAX);
    throttle_pulse = map(throttle_val_joystick,-100,100,SERVO_MID-MOTOR_RANGE-MOTOR_BACKBOOST,SERVO_MID+MOTOR_RANGE);
  }
  else{
    if(mode_user == true){  // Stop Once when mode changed from USER
        steering_pulse = SERVO_MID;
        throttle_pulse = SERVO_MID;
        mode_user = false;
        
    }
    if (mode_status == 2){
        mode_angle = true;
        LED_interval = 300; // LED blinking fast to represent Angle Mode
        steering_pulse = map(steering_val_serial,-100,100,SERVO_MIN,SERVO_MAX);
        throttle_pulse = map(throttle_val_joystick,-100,100,SERVO_MID-MOTOR_RANGE-MOTOR_BACKBOOST,SERVO_MID+MOTOR_RANGE);
    }
    else{
      if(mode_angle == true){  // Stop Once when mode changed from Auto Angle
        steering_pulse = SERVO_MID;
        throttle_pulse = SERVO_MID;
        mode_angle = false;
      }
      if (mode_status == 3){
        mode_pilot = true;
        LED_interval = 50; // LED blinking very fast to represent Auto Pilot
        steering_pulse = map(steering_val_serial,-100,100,SERVO_MIN,SERVO_MAX);
        throttle_pulse = map(throttle_val_serial,-100,100,SERVO_MID-MOTOR_RANGE-MOTOR_BACKBOOST,SERVO_MID+MOTOR_RANGE);
      }
    }
  }
 

  if(clutch_status != 1){ // Stop Motors if clutch_status is not 1.
    throttle_pulse = SERVO_MID;
  }
  if(counter % 2 == 0) // check per 2 loops to save time amonge pulseIn()
  {
    pwm.setPWM(PIN_SERVO, 0, steering_pulse); // Update Servo Angel every loop
    pwm.setPWM(PIN_MOTOR, 0, throttle_pulse); // Update Motor Throttle every loop
  }

  counter += 1; 

  LED_Blink(LED_interval);
}
