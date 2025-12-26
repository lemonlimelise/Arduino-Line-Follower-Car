#include <LiquidCrystal.h>

//LCD PINS
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

//MOTORS PINS
#define IN1 13   
#define IN2 12   
#define IN3 A4   
#define IN4 A5   
#define ENA 11 
#define ENB A1 

//IR SENSORS
#define LEFT_IR  A2
#define RIGHT_IR A3

//ENCODERS
#define LEFT_ENCODER 2
#define RIGHT_ENCODER 3

int baseSpeed = 120;
int turnSpeed = 150;
int recoverySpeed = 45;

int threshold = 500; //Black line threshold
int lostThreshold = 300; 

//DISTANCE
volatile long leftCount = 0;
volatile long rightCount = 0;
long prevLeft = 0;
long prevRight = 0;
float distance = 0;
float cmPerPulse = 0.25;

//STATES
bool recovering = false;
bool pivoting = false;

void leftISR()  { leftCount++; }
void rightISR() { rightCount++; }

//MOTOR CONTROL
void setMotor(int left, int right) {

  // LEFT MOTOR
  if (left >= 0) {
    analogWrite(ENB, left);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    analogWrite(ENB, -left);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  // RIGHT MOTOR
  if (right >= 0) {
    analogWrite(ENA, right);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    analogWrite(ENA, -right);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

//SETUP
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);

  pinMode(LEFT_ENCODER, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), rightISR, RISING);

  lcd.begin(16, 2);
  lcd.print("Distance:");
  lcd.setCursor(0, 1);
  lcd.print("0 cm");
}

//MAIN LOOP
void loop() {

  int leftV  = analogRead(LEFT_IR);
  int rightV = analogRead(RIGHT_IR);

  bool leftBlack  = (leftV  >= threshold);
  bool rightBlack = (rightV >= threshold);

  //SHARP RIGHT TURN 90 DEGREES
  if (!pivoting && !recovering &&
      leftV < lostThreshold && rightV < lostThreshold) {

    pivoting = true;

    setMotor(0, 0);
    delay(60);

    //CORRECT PIVOT DIRECTION
    setMotor(-turnSpeed, turnSpeed);  // left backward, right forward
    delay(220);

    setMotor(0, 0);
    delay(60);

    pivoting = false;
    recovering = true;
    return;
  }

  //RECOVERY MODE
  if (recovering) {

    //IF SENSOR SEES BLACK AGAIN
    if (leftBlack || rightBlack) {
      setMotor(0, 0);
      delay(40);

      //CORRECT LEFT SLIGHTLY
      setMotor(30, 60);
      delay(100);

      recovering = false;
      return;
    }

    //DRIVE SLOWLY UNTIL LINE FOUND
    setMotor(recoverySpeed, recoverySpeed);
    return;
  }

  //NORMAL LINE FOLLOWING
  int leftSpeed = baseSpeed;
  int rightSpeed = baseSpeed;

  int leftErr  = max(0, leftV - threshold);
  int rightErr = max(0, rightV - threshold);

  int gain = 80;  // steering power

  if (rightBlack && !leftBlack) {
    leftSpeed  += gain;
    rightSpeed -= gain;
  }
  else if (leftBlack && !rightBlack) {
    leftSpeed  -= gain;
    rightSpeed += gain;
  }
  else {
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed;
  }

  setMotor(leftSpeed, rightSpeed);

  //TRACK DISTANCE W/ENCODERS
  long dl = leftCount - prevLeft;
  long dr = rightCount - prevRight;

  prevLeft = leftCount;
  prevRight = rightCount;

  if (leftSpeed < 0) dl = 0;
  if (rightSpeed < 0) dr = 0;

  distance += ((dl + dr) / 2.0) * cmPerPulse;

  lcd.setCursor(0, 1);
  lcd.print(distance);
  lcd.print(" cm   ");

  delay(5);
}