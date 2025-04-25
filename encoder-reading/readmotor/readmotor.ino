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
const int8_t MAX_SPEED = 100;
const int8_t SPEED_STEP = 10;

const float wheelDiameter = 0.065; // 65 mm in meters
const float pi = 3.1416;
const int pulsesPerRevolution = 44; // 44 pulses per revolution

bool WireWriteByte(uint8_t val) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool WireWriteDataArray(uint8_t reg, uint8_t *val, unsigned int len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  for (unsigned int i = 0; i < len; i++) {
    Wire.write(val[i]);
  }
  return Wire.endTransmission() == 0;
}

bool WireReadDataByte(uint8_t reg, uint8_t &val) {
  if (!WireWriteByte(reg)) return false;
  Wire.requestFrom(I2C_ADDR, (uint8_t)1);
  if (Wire.available()) {
    val = Wire.read();
    return true;
  }
  return false;
}

int WireReadDataArray(uint8_t reg, uint8_t *val, unsigned int len) {
  if (!WireWriteByte(reg)) return -1;
  Wire.requestFrom(I2C_ADDR, (uint8_t)len);
  unsigned int i = 0;
  while (Wire.available() && i < len) {
    val[i++] = Wire.read();
  }
  return i;
}

int serial_putc(char c, struct __file *) {
  Serial.write(c);
  return c;
}

void printf_begin(void) {
  fdevopen(&serial_putc, 0);
}

void setup() {
  Wire.begin();
  Serial.begin(9600); // Initialize serial communication for debugging

  // Wait for Serial Monitor to open
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }

  printf_begin(); // Initialize printf output
  delay(200);

  // Set the motor type
  uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
  WireWriteDataArray(MOTOR_TYPE_ADDR, &MotorType, 1);
  delay(5);

  // Set the encoder polarity
  uint8_t MotorEncoderPolarity = 0;
  WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1);
}

void loop() {
  // Reset the encoder values
  uint8_t resetValue = 0;
  WireWriteDataArray(MOTOR_ENCODER_TOTAL_ADDR, &resetValue, 1);
  
  // Read the accumulated rotation value of the motors
  uint8_t EncodeTotal[16];
  WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR, EncodeTotal, 16);
  Serial.print("Encode1 = ");
  Serial.print((long)EncodeTotal[0]);
  Serial.print(" Encode2 = ");
  Serial.print((long)EncodeTotal[1]);
  Serial.print(" Encode3 = ");
  Serial.print((long)EncodeTotal[2]);
  Serial.print(" Encode4 = ");
  Serial.println((long)EncodeTotal[3]);
}
