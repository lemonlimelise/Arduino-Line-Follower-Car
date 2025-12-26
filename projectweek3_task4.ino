#include <SoftwareSerial.h>

SoftwareSerial BT(3, 2); //RX, TX

//MOTOR PINS
#define IN1 13
#define IN2 12
#define IN3 A4
#define IN4 A5
#define ENA 11  
#define ENB A1  

int speedVal = 200;         
char currentCommand = 'S'; //current movement command
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 200; //in milliseconds

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopMotors();
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

//MAIN LOOP
void loop() {
  //CHECK FOR BLUETOOTH
  if (BT.available()) {
    char c = BT.read();
    Serial.print("BT: "); Serial.println(c);

    lastCommandTime = millis();  

    //ADJUST SPEED
    if (c >= '0' && c <= '9') {
      setSpeed(c);
    } 
    //MOVEMENT COMMANDS
    else if (c == 'F' || c == 'B' || c == 'L' || c == 'R') {
      currentCommand = c; //change this to current command
    }
  }

  //TIMEOUT
  if (millis() - lastCommandTime > commandTimeout) {
    currentCommand = 'S';
  }

  //EXECUTE CURRENT COMMAND
  handleCommand(currentCommand);
}

//CHANGE SPEED
void setSpeed(char digitChar) {
  int digit = digitChar - '0';
  speedVal = map(digit, 0, 9, 0, 255);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
  Serial.print("Speed set to: "); Serial.println(speedVal);
}

void handleCommand(char c) {
  switch (c) {
    case 'F': forward(); break;
    case 'B': backward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    case 'S': stopMotors(); break;
  }
}

//MOTOR CONTROL 
void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void backward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}