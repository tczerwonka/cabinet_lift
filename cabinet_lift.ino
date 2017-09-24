// cabinet_lift.ino
//
//
// -*- mode: C++ -*-
//
// control a single motor via h-bridge for the cabinet lift

// 24sep2017 added to github
//           git@github.com:tczerwonka/cabinet_lift.git
// 19jul2015 limit switch and stop switch code put in place
// 27jul2015 bluetooth module attached, serial control enabled
// 25jun2016 version 2 in new enclosure
// 26jun2016 -- default motor off is pin LOW
// 09july2015 -- code cleanup, add pwm to display brightness

#include <Arduino.h>
#include <LiquidCrystal.h>
// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//states for the cabinet
#define LIFTED   0
#define LOWERED  1
#define LIFTING  2
#define LOWERING 3
#define UNKNOWN  4
#define STOPPED  5
#define MOVING  6

#define UP   1
#define DOWN 2

#define PWM_OFF 0
#define PWM_LOW 32
#define PWM_MED 128
#define PWM_BRIGHT 255


//Arduino pin definitions
//A0 -- keypad ADC
//4, 5, 6, 7, 8, 9 -- 2-line LCD display
#define DISPLAY_PWM    10
#define AC_POWER       11
#define MOTOR_PWM  12
#define ON_BOARD_LED   13
//14, 15 -- Serial3
#define LOWER_LIMIT    17
#define UPPER_LIMIT    18
#define EMERGENCY_STOP 19
#define H_BRIDGE1 48
#define H_BRIDGE2 53
//20, 21 SDA, SCL -- tied high internally
//48, 49, 50, 51, 52, 53 -- stepper pins

//misc initialization
int incomingByte;
int current_operation = UNKNOWN;
int stop_status = UNKNOWN;


/////////////////////////////////////////////////////////////////////////////////
// setup
/////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);
  //bluetooth port
  Serial3.begin(9600);
  

  lcd.begin(16, 2);
  
  lcd.setCursor(0, 0);
  lcd.print("cabinet_lift_v2 v0.2 ");
  lcd.setCursor(0, 1);
  lcd.print("T Czerwonka 2016 ");
  delay (2000);
  lcd.clear();
  
  
  lcd.setCursor(0, 0);
  lcd.print("Status:");

  pinMode(DISPLAY_PWM, OUTPUT);
  analogWrite(DISPLAY_PWM, PWM_LOW);

  pinMode(UPPER_LIMIT, INPUT);
  pinMode(LOWER_LIMIT, INPUT);
  pinMode(EMERGENCY_STOP, INPUT);

  pinMode(AC_POWER, OUTPUT);
  digitalWrite(AC_POWER, LOW);

  //h-bridge off -- both states the same
  pinMode(H_BRIDGE1, OUTPUT);
  digitalWrite(H_BRIDGE1, LOW);
  pinMode(H_BRIDGE2, OUTPUT);
  digitalWrite(H_BRIDGE2, LOW);

  pinMode(MOTOR_PWM, OUTPUT);
  digitalWrite(MOTOR_PWM, LOW);


  print_status(UNKNOWN);
}



/////////////////////////////////////////////////////////////////////////////////
// check the stop buttons
/////////////////////////////////////////////////////////////////////////////////
int check_stop()
{
  // read the BT/serial port
  //    no -- already being checked in loop
  if (digitalRead(LOWER_LIMIT)) {
    //if at lower limit and lifting -- no stop because there is some 
    //mechanical hysteresis in the switching
    if (stop_status != LIFTING) {
      motor_stop();
      stop_status = LOWER_LIMIT;
      return (LOWER_LIMIT);
    } //if stopstatus
  }
  if (digitalRead(UPPER_LIMIT)) {
    if (stop_status != LOWERING) {
      motor_stop();
      stop_status = UPPER_LIMIT;
      return (UPPER_LIMIT);
    } //if stopstatus
  }
  if (digitalRead(EMERGENCY_STOP)) {
    motor_stop();
    stop_status = EMERGENCY_STOP;
    return (EMERGENCY_STOP);
  }
  //nothing to see here
  return (0);
}



/////////////////////////////////////////////////////////////////////////////////
// read the analog buttons
/////////////////////////////////////////////////////////////////////////////////
int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the sensor
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  // For V1.1 us this threshold
  if (adc_key_in < 50)   return btnRIGHT;
  if (adc_key_in < 195)  return btnUP;
  if (adc_key_in < 380)  return btnDOWN;
  if (adc_key_in < 555)  return btnLEFT;
  if (adc_key_in < 790)  return btnSELECT;


  return btnNONE;  // when all others fail, return this...
}



/////////////////////////////////////////////////////////////////////////////////
// stop the motor
/////////////////////////////////////////////////////////////////////////////////
int motor_stop()
{
  digitalWrite(H_BRIDGE1, LOW);
  digitalWrite(H_BRIDGE2, LOW);
  digitalWrite(AC_POWER, LOW);
  stop_status = STOPPED;
  print_status(STOPPED);
  analogWrite(DISPLAY_PWM, PWM_LOW);
}
int motor_up()
{
  analogWrite(DISPLAY_PWM, PWM_BRIGHT);
  digitalWrite(AC_POWER, HIGH);
  //wait for a moment after powering up
  delay(1500);
  digitalWrite(H_BRIDGE1, HIGH);
  digitalWrite(H_BRIDGE2, LOW);
  stop_status = LIFTING;
  print_status(LIFTING);
}
int motor_down()
{
  analogWrite(DISPLAY_PWM, PWM_BRIGHT);
  digitalWrite(AC_POWER, HIGH);
  //wait for a moment after powering up
  delay(1500);
  digitalWrite(H_BRIDGE1, LOW);
  digitalWrite(H_BRIDGE2, HIGH);
  stop_status = LOWERING;
  print_status(LOWERING);
}



/////////////////////////////////////////////////////////////////////////////////
// run motor
/////////////////////////////////////////////////////////////////////////////////
int move_fullstroke(int direction)
{
  lcd.setCursor(0, 1);



  switch (direction)     {
    case UP:
      {
        //if the opposite button is hit -- stop
        if (current_operation == LOWERING) {
          motor_stop();
          delay(1000);
          return (1);
        } //if
        if (stop_status == UPPER_LIMIT) {
          //if we're here, we've been commanded to go up
          //but are already at the upper stop, so do nothing
          delay(1000);
          return (1);
        } //if
        motor_up();
      }
      break;

    case DOWN:
      {

        if (current_operation == LIFTING) {
          motor_stop();
          delay(1000);
          return (1);
        } //if
        if (stop_status == LOWER_LIMIT) {
          //if we're here, we've been commanded to go down
          //but are already at the lower stop, so do nothing
          delay(1000);
          return (1);
        } //if
        motor_down();
      }
      break;

    default:
      {
        return (1);
      }
  }


  Serial.println("=================");
}



/////////////////////////////////////////////////////////////////////////////////
// modify status line on lcd
/////////////////////////////////////////////////////////////////////////////////
int print_status(int status)
{
  lcd.setCursor(8, 0);           // move to position on first line
  current_operation = status;
  switch (status)
  {
    case LIFTED:
      {
        lcd.print("at top  ");
        Serial3.write("TOP\n");
        break;
      }
    case LOWERED:
      {
        lcd.print("lowered ");
        Serial3.write("BOTTOM\n");
        break;
      }
    case LIFTING:
      {
        lcd.print("lifting");
        Serial3.write("LIFTING\n");
        break;
      }
    case LOWERING:
      {
        lcd.print("lowering");
        Serial3.write("LOWERING\n");
        break;
      }
    case EMERGENCY_STOP:
      {
        lcd.print("USR HALT");
        Serial3.write("USER HALT\n");
        break;
      }
    case LOWER_LIMIT:
      {
        lcd.print("LO LIMIT");
        Serial3.write("LOWER LIMIT\n");
        break;
      }
    case UPPER_LIMIT:
      {
        lcd.print("HI LIMIT");
        Serial3.write("UPPER LIMIT\n");
        break;
      }
    case STOPPED:
      {
        lcd.print("STOPPED ");
        Serial3.write("STOPPED\n");
        break;
      }
    case UNKNOWN:
      {
        lcd.print("unknown");
        Serial3.write("UNKNOWN STATE\n");
        break;
      }
  }
}



/////////////////////////////////////////////////////////////////////////////////
// modify status line on lcd
/////////////////////////////////////////////////////////////////////////////////
int print_motion(int status)
{
  lcd.setCursor(0, 1);           // move to the begining of the second line
  switch (status)
  {
    case btnRIGHT:
      {
        lcd.print("RIGHT         ");
        break;
      }
    case btnLEFT:
      {
        lcd.print("LEFT            ");
        break;
      }
    case btnUP:
      {
        lcd.print("Moving UP    ");
        break;
      }
    case btnDOWN:
      {
        lcd.print("Moving DOWN  ");
        break;
      }
    case btnSELECT:
      {
        lcd.print("SELECT          ");
        break;
      }
    case btnNONE:
      {
        lcd.print("Select motion ");
        break;
      }
  }
}



/////////////////////////////////////////////////////////////////////////////////
// loop
/////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // read the BT/serial port
  if (Serial3.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial3.read();

    // say what you got:
    // Serial.print("I received: ");
    // Serial.println(incomingByte, DEC);

    //U
    if (incomingByte == 85) {
      print_motion(btnUP);
      move_fullstroke(UP);
    }
    //L
    if (incomingByte == 76) {
      print_motion(btnUP);
      move_fullstroke(UP);
    }

    //D
    if (incomingByte == 68) {
      print_motion(btnDOWN);
      move_fullstroke(DOWN);
    }
    //R
    if (incomingByte == 83) {
      print_motion(btnDOWN);
      move_fullstroke(DOWN);
    }
    
    //C -- stop
    if (incomingByte == 67) {
      motor_stop();
    }

  }

 //check for any stop conditions
  check_stop();
  lcd_key = read_LCD_buttons();  // read the buttons

  switch (lcd_key)               // depending on which button was pushed, we perform an action
  {
    case btnRIGHT:
      {
        print_motion(btnRIGHT);
        break;
      }
    case btnLEFT:
      {
        print_motion(btnLEFT);
        break;
      }
    case btnUP:
      {
        print_motion(btnUP);
        move_fullstroke(UP);
        break;
      }
    case btnDOWN:
      {
        print_motion(btnDOWN);
        move_fullstroke(DOWN);
        break;
      }
    case btnSELECT:
      {
        print_motion(btnSELECT);
        break;
      }
    case btnNONE:
      {
        print_motion(btnNONE);
        break;
      }
  }



}

