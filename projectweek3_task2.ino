#include <LiquidCrystal.h>
#include <Wire.h> //FOR MPU6050

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

//MOTOR PINS
#define IN1 13 
#define IN2 12 
#define IN3 A2 
#define IN4 A1 
#define ENA 11 
#define ENB 3  

//MPU6050 ADDRESS
const int MPU_ADDR = 0x68;

int motorSpeed = 255;       
int turnDuration = 1500;    
float current_yaw = 0.0;
float real_slope = 0.0; 

//SETUP (INITIALIZATION)
void setup() {
  //LCD
  lcd.begin(16, 2);
  lcd.print("Init MPU6050...");
  
  //MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  //PWR_MGMT_1 register
  Wire.write(0);     //Wake up MPU6050
  Wire.endTransmission(true);

  //Motors
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  delay(1000);
  runSequence(); 
}

void loop() {
  // Empty
}

void runSequence() {
  
  //MOVE FORWARD
  moveForward(motorSpeed);
  for(int i=0; i<20; i++) { //2 Seconds
    real_slope = getRealSlope(); //Read Sensor
    updateLCD("Going forward", current_yaw, real_slope);
    delay(100); 
  }

  //STOP FOR 4 SECONDS
  stopMotors();
  for(int i=0; i<40; i++) { // 4 Seconds
    real_slope = getRealSlope(); // Keep updating slope even while stopped
    updateLCD("Stopping for 4 seconds", current_yaw, real_slope);
    delay(100);
  }

  //SPIN 360 DEGREES
  spinRight(motorSpeed);
  
  int steps = 20; 
  int stepDelay = turnDuration / steps; 
  float angleIncrement = 360.0 / steps; 
  
  for(int i=0; i<steps; i++) {
    current_yaw += angleIncrement; 
    real_slope = getRealSlope(); // Read Sensor
    updateLCD("Spinning 360 degrees", current_yaw, real_slope);
    delay(stepDelay); 
  }
  
  current_yaw = 360.0; 
  stopMotors();
  delay(500); 

  //UP RAMP
  moveForward(motorSpeed);
  
  for(int i=0; i<20; i++) { //2 Seconds
    real_slope = getRealSlope(); //Read sensor
    updateLCD("Going up ramp", current_yaw, real_slope);
    delay(100); 
  }

  stopMotors();
  updateLCD("Done", current_yaw, real_slope);
}

//MPU6050
float getRealSlope() {
  //Read Accelerometer data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); //request bytes 
  
  int16_t AcX = Wire.read()<<8 | Wire.read();
  int16_t AcY = Wire.read()<<8 | Wire.read();
  int16_t AcZ = Wire.read()<<8 | Wire.read();
  
  //CALC PITCH(SLOPE)
  float slope = atan2(AcY, AcZ) * 180 / PI; 
  
  return slope;
}

//LCD DISPLAY
void updateLCD(String status, float y, float s) {
  lcd.setCursor(0, 0);
  lcd.print(status);
  lcd.print("        "); 
  
  lcd.setCursor(0, 1);
  lcd.print("Y:"); //Display yaw
  lcd.print((int)y); 
  lcd.print((char)223); 
  
  lcd.print(" S:"); //Display angle of slope
  lcd.print((int)s); 
  lcd.print((char)223); 
  lcd.print("  ");      
}

//MOTOR CONTROL 
void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void spinRight(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); 
  analogWrite(ENA, speed);
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
