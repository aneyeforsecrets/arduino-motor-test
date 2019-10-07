#include <Arduino.h>
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>

enum MotorMode
{
  BRAKEVCC,
  CW,
  CCW,
  BRAKEGND,
};

enum Motor
{
  MOTOR_A,
  MOTOR_B,
};

const int kCurrentSensingThreshhold = 1000;

/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
const int kInAPin[2] = {2, 0}; // INA: Clockwise input
const int kInBPin[2] = {3, 0}; // INB: Counter-clockwise input
const int kPwmPin[2] = {5, 0}; // PWM input
const int kCspPin[2] = {0, 0}; // CS: Current sense ANALOG input
const int kEnPin[2] = {1, 0};  // EN: Status of switches output (Analog pin)

const int kStatPin = 13;

// LCD
const int kLcdAddress = 0x27;
LiquidCrystal_PCF8574 lcd(kLcdAddress);

// Joystick
const uint8_t kXPin = 7;
const uint8_t kYPin = 6;

void lcdSetup(int address)
{
  Wire.beginTransmission(kLcdAddress);
  lcd.clear();
  lcd.setBacklight(255);
}

void motorSetup()
{
  pinMode(kStatPin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(kInAPin[i], OUTPUT);
    pinMode(kInBPin[i], OUTPUT);
    pinMode(kPwmPin[i], OUTPUT);
  }
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(kInAPin[i], LOW);
    digitalWrite(kInBPin[i], LOW);
  }
}
void motorOff(int motor)
{
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(kInAPin[i], LOW);
    digitalWrite(kInBPin[i], LOW);
  }
  analogWrite(kPwmPin[motor], 0);
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled
 
 mode: Should be one of the following values
  BRAKEVCC: Brake to VCC
  CW: Turn Clockwise
  CCW: Turn Counter-Clockwise
  BRAKEGND: Brake to GND
 
 speed: should be a value between 0 and 255, higher the number, the faster
 */
void motorGo(Motor motor, MotorMode mode, uint8_t speed)
{

  if (motor == MOTOR_A || motor == MOTOR_B)
  {
    switch (mode)
    {
    case BRAKEVCC:
      digitalWrite(kInAPin[motor], HIGH);
      digitalWrite(kInBPin[motor], HIGH);
      break;
    case CW:
      digitalWrite(kInAPin[motor], HIGH);
      digitalWrite(kInBPin[motor], LOW);
      break;
    case CCW:
      digitalWrite(kInAPin[motor], LOW);
      digitalWrite(kInBPin[motor], HIGH);
      break;
    case BRAKEGND:
      digitalWrite(kInAPin[motor], LOW);
      digitalWrite(kInBPin[motor], LOW);
      break;

    default:
      // Invalid mode skips changing the PWM signal
      return;
    }
    analogWrite(kPwmPin[motor], speed);
  }
  return;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  motorSetup();
  delay(100);
  lcdSetup(kLcdAddress);
  delay(100);
} // setup()

void loop()
{
  int joyX;
  int joyY;
  int deadZoneX = abs(analogRead(kXPin) - 512) + 5;
  uint8_t speed;
  MotorMode mode;
  int time = 0;

  String line0;
  String line1;

  float motorCurrent;
  int state = 0;

  while (state == 0)
  {

    joyX = analogRead(kXPin);
    joyY = analogRead(kYPin);

    speed = abs(joyX - 512) / 2;
    if (joyX - 512 < -deadZoneX)
    {
      mode = CCW;
    }
    else if (joyX - 512 > deadZoneX)
    {
      mode = CW;
    }
    else
    {
      mode = BRAKEVCC;
      speed = 0;
    }
    
    if (time % 100 == 0)
    {
      line0 = String("");
      line1 = String("");

      line0 = String(line0 + "Speed: " + String(speed, DEC));
      line1 = String(line1 + "Mode: " + String(mode, DEC));

      lcd.clear();
      lcd.print(line0);
      // Serial.println(speed);
      lcd.setCursor(0, 1);
      lcd.print(line1);
      time = 0;
    }

    motorGo(MOTOR_A, mode, speed);

    // if (analogRead(kCspPin[MOTOR_A]) > kCurrentSensingThreshhold)
    // { // If the motor locks, it will shutdown and Resets the process of increasing the PWM
    //   state = -1;
    //   continue;
    // }
    time++;
    delay(1);
  }
  motorOff(MOTOR_A);
}
