#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

//MOTOR PINS
#define ENA 11
#define IN1 13
#define IN2 12
#define ENB 3       
#define IN3 A4
#define IN4 A5

//IR SENSORS
#define LEFT_IR  A2
#define RIGHT_IR A3

//ENCODERS
#define LEFT_ENCODER  2   
#define RIGHT_ENCODER A1  
volatile long leftTicks = 0;
volatile long rightTicks = 0;
long prevLeftTicks = 0;
long prevRightTicks = 0;
float distance = 0.0;
float cmPerTick = 0.43; 

int baseSpeed = 110; 
int blackThreshold = 550; //threshold for black line

//PID TUNING
float Kp = 0.25; 
float Kd = 20.0; 
float Ki = 0.00;

//STATES
float lastError = 0;
unsigned long prevLCDTime = 0;
volatile byte lastA1State = 0; 
bool finished = false;
bool hasStoppedAt10 = false; //FOR 10CM STOP


//INTERRUPT ROUTINES (ISR)
void leftISR() {
  leftTicks++;
}

ISR(PCINT1_vect) {
  byte currentA1 = digitalRead(RIGHT_ENCODER);
  if (currentA1 == HIGH && lastA1State == LOW) {
    rightTicks++;
  }
  lastA1State = currentA1;
}

//MOTOR CONTROL
void setMotors(int leftSpd, int rightSpd) {
  leftSpd = constrain(leftSpd, -255, 255);
  rightSpd = constrain(rightSpd, -255, 255);

  //LEFT MOTORS
  if (leftSpd >= 0) {
    analogWrite(ENB, leftSpd);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    analogWrite(ENB, abs(leftSpd));
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  //RIGHT MOTORS
  if (rightSpd >= 0) {
    analogWrite(ENA, rightSpd);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    analogWrite(ENA, abs(rightSpd));
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

// SETUP
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
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), leftISR, RISING);

  pinMode(RIGHT_ENCODER, INPUT_PULLUP);
  
  cli(); 
  PCICR |= (1 << PCIE1);    
  PCMSK1 |= (1 << PCINT9);  
  sei(); 

  lcd.begin(16, 2);
  lcd.print("Calibrated Ready");
  delay(1000);
  lcd.clear();
}

//MAIN LOOP
void loop() {
  if (finished) return; // Do nothing if finished

  //READ SENSORS
  int leftVal = analogRead(LEFT_IR);   
  int rightVal = analogRead(RIGHT_IR);

  //THRESHOLD CHECK
  bool isLeftBlack = (leftVal > blackThreshold);
  bool isRightBlack = (rightVal > blackThreshold);

  //STOP AT 10CM
  if (distance >= 10.0 && !hasStoppedAt10) {
    // 1. Stop Motors
    setMotors(0, 0);
    
    //WAIT 2 SECONDS
    unsigned long pauseStart = millis();
    while (millis() - pauseStart < 2000) {
       //UPDATE LCD TIMER
       unsigned long currentM = millis();
       if (currentM - prevLCDTime > 200) {
          prevLCDTime = currentM;
          lcd.setCursor(0, 0);
          lcd.print("Time:");
          lcd.print(currentM / 1000);
          lcd.print("s (WAIT)");
          
          lcd.setCursor(0, 1);
          lcd.print("Dist:");
          lcd.print(distance, 0);
          lcd.print("cm ");
       }
    }
    
    //RESUME LINE FOLLOWING
    hasStoppedAt10 = true; // Mark done so it doesn't happen again
    // Loop continues below to normal PID...
  }

  //FINISH LINE CHECK
  if (isLeftBlack && isRightBlack) {
  setMotors(0, 0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("FINISH LINE!");
  lcd.setCursor(0, 1);
  lcd.print(distance);
  finished = true;
  while(1); // Stop execution permanently
  }

  //ERROR CALC
  float error = 0;

  //CORNER MEMORY
  if (isLeftBlack && isRightBlack) {
    if (lastError > 0) error = 800;      // Keep Hard Right
    else if (lastError < 0) error = -800; // Keep Hard Left
  }
  else {
    //DIFFERENTIAL CALC
    error = rightVal - leftVal;

    // DEADBAND FILTER 
    if (abs(error) < 30) error = 0; 
  }

  //PID CALC
  float P = error;
  float D = error - lastError; 
  lastError = error;

  float correction = (Kp * P) + (Kd * D);

  //CORRECTION CALC
  int speedReduction = abs(correction) * 0.15; 
  int currentBaseSpeed = baseSpeed - speedReduction;
  currentBaseSpeed = max(currentBaseSpeed, 60); 

  int leftSpeed = currentBaseSpeed + correction;
  int rightSpeed = currentBaseSpeed - correction;

  setMotors(leftSpeed, rightSpeed);

  //LCD DISPLAY 
  unsigned long currentMillis = millis();
  
  if (currentMillis - prevLCDTime > 200) {
    prevLCDTime = currentMillis;

    long dLeft = leftTicks - prevLeftTicks;
    long dRight = rightTicks - prevRightTicks;
    prevLeftTicks = leftTicks;
    prevRightTicks = rightTicks;

    float ticksAverage = (dLeft + dRight) / 2.0;
    distance += ticksAverage * cmPerTick;

    lcd.setCursor(0, 0);
    lcd.print("Time:");
    lcd.print(currentMillis / 1000);
    lcd.print("s ");

    lcd.setCursor(0, 1);
    lcd.print("Dist:");
    lcd.print(distance, 0); // Removed decimal for cleaner look at high numbers
    lcd.print("cm ");
  }

  delay(2);
}