// PWM capable pins for the nano are 3, 5, 6, 9, 10, and 11. 

int motorA_speed = 9;  // ENA
int motorA_in1 = 12;    // IN1
int motorA_in2 = 13;    // IN2

int motorB_speed = 11;  // ENB  
int motorB_in1 = 7;    // IN3
int motorB_in2 = 8;    // IN4

// Change your pin assignments to match your actual wiring:
int right_encoder_phaseA = 3;  // Right Green (was expecting yellow)
int right_encoder_phaseB = 5;  // Right Yellow (was expecting green)
int left_encoder_phaseA = 2;   // Left Green (was expecting yellow)  
int left_encoder_phaseB = 4;   // Left Yellow (was expecting green)

int currentLeftSpeed = 0;
int currentRightSpeed = 0;

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 500;

void setup() {
  Serial.begin(9600);

  // Motor A Setup
  pinMode(motorA_speed, OUTPUT);
  pinMode(motorA_in1, OUTPUT);
  pinMode(motorA_in2, OUTPUT);

  // Motor B Setup
  pinMode(motorB_speed, OUTPUT);
  pinMode(motorB_in1, OUTPUT);
  pinMode(motorB_in2, OUTPUT);

  // Encoder Setup
  pinMode(right_encoder_phaseA, INPUT_PULLUP);
  pinMode(right_encoder_phaseB, INPUT_PULLUP);
  pinMode(left_encoder_phaseA, INPUT_PULLUP);
  pinMode(left_encoder_phaseB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(2), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(3), rightEncoderISR, RISING);

  delay(1000);

  Serial.println("READY");
}

void setMotor(int in1_pin, int in2_pin, int speed_pin, int speed) {
  if(speed > 0){
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
    analogWrite(speed_pin, constrain(speed, 0, 255));
  } else if (speed < 0) {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
    analogWrite(speed_pin, constrain(abs(speed), 0, 255));
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
    analogWrite(speed_pin, 0);
  }
}

void moveRobot(int leftSpeed, int rightSpeed) {
  setMotor(motorA_in1, motorA_in2, motorA_speed, leftSpeed);
  setMotor(motorB_in1, motorB_in2, motorB_speed, rightSpeed);
  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;
}

// Add interrupt functions:
void rightEncoderISR() {
  if(digitalRead(right_encoder_phaseB) == HIGH) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}

void leftEncoderISR() {
  if(digitalRead(left_encoder_phaseB) == HIGH) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void parseCommand(String input) {

  lastCommandTime = millis();

  input.trim();

  int firstSpace = input.indexOf(' ');
  int secondSpace = input.indexOf(' ', firstSpace + 1);

  if(firstSpace == -1 || secondSpace == -1){
    Serial.println("ERROR Invalid format");
    return;
  }

  String command = input.substring(0, firstSpace);
  int value1 = input.substring(firstSpace + 1, secondSpace).toInt();
  int value2 = input.substring(secondSpace + 1).toInt();

  if (command == "MOVE") {
    moveRobot(value1, value2);
    Serial.println("OK MOVE " + String(value1) + " " + String(value2));
  } else if (command == "STOP") {
    moveRobot(0, 0);
    Serial.println("OK STOP");
  } else if (command == "STATUS") {
    Serial.println("STATUS " + String(currentLeftSpeed) + " " + String(currentRightSpeed));
  } else if (command == "GET_ENC") {
    Serial.println("ENCODERS " + String(leftEncoderCount) + " " + String(rightEncoderCount));
  } else if (command == "RST_ENC") {
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    Serial.println("OK RESET"); 
  }
  else {
    Serial.println("ERROR Unknown command: " + command);
  }

}

void loop() {

  if(millis() - lastCommandTime > COMMAND_TIMEOUT &&
    (currentLeftSpeed != 0 || currentRightSpeed != 0)) {
    moveRobot(0, 0);
  }

  if(Serial.available() > 0){
    String input = Serial.readStringUntil('\n');
    parseCommand(input);
  }

  // Serial.println("L2:" + String(digitalRead(2)) + " L4:" + String(digitalRead(4)) + 
  //              " R3:" + String(digitalRead(3)) + " R5:" + String(digitalRead(5)));
  // delay(500);
}
