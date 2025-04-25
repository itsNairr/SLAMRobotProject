#include <Wire.h>
#include <Servo.h>

#define I2C_ADDR 0x34

// Motor control addresses
#define MOTOR_TYPE_ADDR 20
#define MOTOR_ENCODER_POLARITY_ADDR 21
#define MOTOR_FIXED_SPEED_ADDR 51
#define MOTOR_ENCODER_TOTAL_ADDR 60

// Motor types
#define MOTOR_TYPE_JGB37_520_12V_110RPM 3

// Servo parameters
Servo myServo;
const int servoPin = 12;
const int angleStep = 10;
const int angleDelay = 500;

// Motor commands
const int8_t SPEED_STOP = 0;

// Speed range
const int8_t MAX_SPEED = 255; // Max speed due to PWM limit

const float wheelDiameter = 0.065; // 65 mm in meters
const float pi = 3.1416;
const int TPR = 1000; // 44 pulses per revolution

// Conversion factors
// Calculate ticks per meter
const float wheelCircumference = pi * wheelDiameter; // in meters
const float ticksPerMeter = TPR / wheelCircumference; // ticks per meter

int8_t  currentSpeed = 0

// PID control variables
float kp = 5.0; // Increased proportional gain
float ki = 0.01; // Decreased integral gain
float integral = 0;
float previousError = 0;
unsigned long previousTime = 0;

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

void resetEncoders() {
  int32_t resetValue[4] = {0, 0, 0, 0};
  WireWriteDataArray(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t *)resetValue, 16);
}

int WireReadDataArray(uint8_t reg, uint8_t *val, unsigned int len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDR, (uint8_t)len);
  unsigned int i = 0;
  while (Wire.available() && i < len) {
    val[i++] = Wire.read();
  }
  return i;
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

  Serial.println("Setup Complete");
  Serial.println("Target Ticks:");
  Serial.println(setpoint);
  delay(5000);
  previousTime = millis();
}

void loop() {;
  int32_t EncodeTotal[4];

  WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t*)EncodeTotal, 16);

  Serial.print("Encode 1 = ");
  Serial.print(EncodeTotal[0]);
  Serial.print(" Encode 2 = ");
  Serial.println(EncodeTotal[2]);
  Serial.println(currentSpeed);

  if (EncodeTotal[0] < TPR) {
    currentSpeed += 1;
    if (currentSpeed > MAX_SPEED) {
      currentSpeed = MAX_SPEED;
  }
      setMotorSpeed(currentSpeed, currentSpeed);
  }

  resetEncoders();

  delay(100); // Adjust as needed for control loop timing
}
