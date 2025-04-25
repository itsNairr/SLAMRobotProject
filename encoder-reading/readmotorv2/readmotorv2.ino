#include <Wire.h>
#include <Servo.h>

#define I2C_ADDR 0x34

// Motor control addresses
#define MOTOR_TYPE_ADDR 20
#define MOTOR_ENCODER_POLARITY_ADDR 21
#define MOTOR_FIXED_PWM_ADDR 31
#define MOTOR_FIXED_SPEED_ADDR 51
#define MOTOR_ENCODER_TOTAL_ADDR 60

// Motor types
#define MOTOR_TYPE_WITHOUT_ENCODER 0
#define MOTOR_TYPE_TT 1
#define MOTOR_TYPE_N20 2
#define MOTOR_TYPE_JGB37_520_12V_110RPM 3

// Servo parameters
Servo myServo;
const int servoPin = 12;
const int angleStep = 10;
const int angleDelay = 500;

// Speed range
const int8_t MAX_SPEED = 50; // Reduced max speed for slower movement
const int8_t SPEED_STEP = 10;

const float wheelDiameter = 0.065; // 65 mm in meters
const float pi = 3.1416;
const int pulsesPerRevolution = 44; // 44 pulses per revolution

bool WireWriteDataArray(uint8_t reg, uint8_t *val, unsigned int len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  for (unsigned int i = 0; i < len; i++) {
    Wire.write(val[i]);
  }
  return Wire.endTransmission() == 0;
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

void resetEncoders() {
  int32_t resetValue[4] = {0, 0, 0, 0};
  WireWriteDataArray(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t *)resetValue, 16);
}

void setup() {
  Wire.begin();
  Serial.begin(9600); // Initialize serial communication for debugging

  // Wait for Serial Monitor to open
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }

  delay(200);

  // Set the motor type
  uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
  WireWriteDataArray(MOTOR_TYPE_ADDR, &MotorType, 1);
  delay(5);

  // Set the encoder polarity
  uint8_t MotorEncoderPolarity = 0;
  WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1);

  // Servo setup
  myServo.attach(servoPin);
  for (int angle = 0; angle <= 180; angle += angleStep) {
    myServo.write(angle);
    delay(angleDelay);
  }
  myServo.write(90); // Center position
}

  int8_t forwardSpeed[4] = {25, 25, 25, 25}; // Reduced speed values
  int8_t backwardSpeed[4] = {-25, -25, -25, -25}; // Reduced speed values
  uint8_t rawData[16];
  int32_t EncodeTotal[4];


void loop() {

  // Move the car forward
  // WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, forwardSpeed, 4);
  // delay(2000); // Move forward for 2 seconds

  WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR,(uint8_t*)EncodeTotal, 16);//

  Serial.print("Encode 1 = ");
  Serial.print(EncodeTotal[0]);
  Serial.print(" Encode 2 = ");
  Serial.println(EncodeTotal[2]);

  delay(500); // Delay to allow time to read the serial output
}
