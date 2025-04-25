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
const int32_t SPEED_STOP = 0;

// Speed range
const int32_t MAX_SPEED = 255; // Max speed due to PWM limit

const float wheelDiameter = 0.065; // 65 mm in meters
const int TPR = 1320; // 44 pulses per revolution

// Conversion factors
const float wheelCircumference = PI * wheelDiameter; // in meters
const float ticksPerMeter = TPR / wheelCircumference; // ticks per meter

// Sampling interval for measurements in milliseconds
const int T = 100; // Smaller sampling interval for faster updates

// Desired distance in meters
float desiredDistance = 1.0; 
float desiredTicks = desiredDistance * ticksPerMeter;

// PID control variables
float kp = 1.0;
float ki = 0.005; // Reduced integral gain
float kd = 1; // Adding derivative term to reduce overshoot
float integral = 0;
float previousError = 0;
float error = 0;
float currentDistance = 0;
float output = 0;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

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

  Serial.begin(9600);
  Serial.println("Setup Complete");
  Serial.println("Target Ticks:");
  Serial.println(desiredTicks);
  resetEncoders();
  delay(5000);
  t_last = millis();
}

void loop() {
  int8_t forwardSpeed[4] = {0, 0, 0, 0};
  int32_t EncodeTotal[4];

  // Get the elapsed time [ms]
  t_now = millis();

  if (t_now - t_last >= T) {
    WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t*)EncodeTotal, 16);

    Serial.print("Encode 1 (L-) = ");
    Serial.print(EncodeTotal[0]);
    Serial.print(" Encode 2 (R+) = ");
    Serial.println(EncodeTotal[2]);
    Serial.print("Time = ");
    Serial.println(t_now - t_last);
   
    // Calculate the current distance traveled in meters
    currentDistance = EncodeTotal[2] / ticksPerMeter;

    // Debugging: Print calculated current distance
    Serial.print("Current Distance: ");
    Serial.println(currentDistance);

    error = desiredDistance - currentDistance;

    // Debugging: Print error value
    Serial.print("Error: ");
    Serial.println(error);

    // Reset integral windup when the error changes sign
    if ((previousError < 0 && error > 0) || (previousError > 0 && error < 0)) {
      integral = 0;
    }

    integral += error * (t_now - t_last);

    // Compute the derivative of the error
    float derivative = (error - previousError) / (t_now - t_last);

    // Debugging: Print integral and derivative values
    Serial.print("Integral: ");
    Serial.println(integral);
    Serial.print("Derivative: ");
    Serial.println(derivative);

    output = (kp * error) + (ki * integral) + (kd * derivative); // PID controller output
    
    Serial.print("Output= ");
    Serial.println(output); // Print the output value for debugging

    // Limit output to max speed range and prevent integral windup
    if (output > MAX_SPEED) {
      output = MAX_SPEED;
      integral -= error * (t_now - t_last); // Prevent integral windup
    } else if (output < -MAX_SPEED) {
      output = -MAX_SPEED;
      integral -= error * (t_now - t_last); // Prevent integral windup
    }

    // Update previous error
    previousError = error;

    // Record the current time [ms]
    t_last = t_now;

    // Set motor speed based on PID output (adjusting for motor direction)
    forwardSpeed[0] = -(int8_t)output; // Reverse direction for motor 1
    forwardSpeed[2] = (int8_t)output;  // Normal direction for motor 2
    setMotorSpeed(forwardSpeed[0], forwardSpeed[2]);

    // Stop the motors if the target distance is reached
    // if (error < 0.01) {
    //   setMotorSpeed(SPEED_STOP, SPEED_STOP);
    //   Serial.println("Target distance reached. Motors stopped.");
    //   while (1); // Stop the loop
    // }
  }
}
