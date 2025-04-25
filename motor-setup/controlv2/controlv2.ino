#include <Wire.h>
#include <Servo.h>

#define I2C_ADDR 0x34


// Motor control addresses
#define MOTOR_TYPE_ADDR 20
#define MOTOR_ENCODER_POLARITY_ADDR 21
#define MOTOR_FIXED_SPEED_ADDR 51

// Motor types
#define MOTOR_TYPE_JGB37_520_12V_110RPM 3

// Servo parameters
Servo myServo;
const int servoPin = 12;
const int angleStep = 10;
const int angleDelay = 500;

// Motor commands
const int8_t SPEED_FORWARD = 16;
const int8_t SPEED_BACKWARD = -16;
const int8_t SPEED_STOP = 0;

void WireWriteDataArray(uint8_t reg, uint8_t *val, unsigned int len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  for (unsigned int i = 0; i < len; i++) {
    Wire.write(val[i]);
  }
  Wire.endTransmission();
}

void setMotorSpeed(int8_t speedLeft, int8_t speedRight) {
  int8_t data[4] = {speedLeft, 0, speedRight, 0};
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, (uint8_t *)data, 4);
}

void moveCar(int8_t speedLeft, int8_t speedRight, unsigned long duration) {
  setMotorSpeed(speedLeft, speedRight);
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    // Non-blocking delay
  }
  setMotorSpeed(SPEED_STOP, SPEED_STOP);
}

void moveServo(int angle) {
  myServo.write(angle);
  unsigned long startTime = millis();
  while (millis() - startTime < angleDelay) {
    // Non-blocking delay
  }
}

void setup() {
  Wire.begin();
  myServo.attach(servoPin);
  delay(200);

  uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
  uint8_t MotorEncoderPolarity = 0;
  WireWriteDataArray(MOTOR_TYPE_ADDR, &MotorType, 1);
  delay(5);
  WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1);

  for (int angle = 0; angle <= 180; angle += angleStep) {
    moveServo(angle);
  }
  moveServo(90);
}

void loop() {
  moveCar(SPEED_BACKWARD, SPEED_FORWARD, 4000);  // Move forward
  delay(1000);
  moveCar(SPEED_FORWARD, SPEED_BACKWARD, 4000);  // Move backward
  delay(1000);

  moveServo(45);
  moveCar(SPEED_BACKWARD, SPEED_FORWARD, 4000);  // Move forward
  delay(1000);
  moveCar(SPEED_FORWARD, SPEED_BACKWARD, 4000);  // Move backward
  moveServo(90);
  delay(1000);

  moveServo(145);
  moveCar(SPEED_BACKWARD, SPEED_FORWARD, 4000);  // Move forward
  delay(1000);
  moveCar(SPEED_FORWARD, SPEED_BACKWARD, 4000);  // Move backward
  moveServo(90);
  delay(1000);

  while (1);  // Stop the loop
}
