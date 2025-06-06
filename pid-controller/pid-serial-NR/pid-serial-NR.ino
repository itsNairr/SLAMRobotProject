#include <Wire.h>
#include <Servo.h>

#define I2C_ADDR 0x34 // I2C address of the motor controller

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
const float SERVO_THRESHOLD = 1;

// Sampling interval for measurements in milliseconds
const int T = 250; // milliseconds

// Car parameters
const float L_diff = 0.180; // Distance between the back left and right wheels in meters
const float L_steering = 0.170; // Distance between the front and rear wheels in meters
const float wheelDiameter = 0.065; // 65 mm in meters
const int TPR = 1320; // 44 pulses per revolution
const int32_t MAX_SPEED = 255; // Max speed due to PWM limit
const int32_t SPEED_STOP = 0;// Motor stop

// Conversion factors
const float wheelCircumference = PI * wheelDiameter; // in meters
const float ticksPerMeter = TPR / wheelCircumference; // ticks per meter

// Desired speed in m/s
float desiredSpeed = 0; //  desired speed in meters per second
float angularVel = 0; //  desired omega in rad/s
float desiredTicksPerInterval = desiredSpeed * ticksPerMeter * T / 1000.0; // Desired ticks per interval

// Filtering variables
const int FILTER_SIZE = 5;
int bufferIndex = 0;
int32_t encoderBuffer[4][FILTER_SIZE] = {0};
int8_t forwardSpeed[4] = {0, 0, 0, 0};
int32_t EncodeTotal[4];
int32_t prevEncoder[4] = {0};


// PID control variables
float kp = 4;
float ki = 0.025;
float integral = 0;
float error = 0;
float currentspeed = 0;
float output = 0;
float delta = 0;
float prevDelta = 0;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

void WireWriteDataArray(uint8_t reg, uint8_t *val, unsigned int len) { // Write data to the I2C bus
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  for (unsigned int i = 0; i < len; i++) {
    Wire.write(val[i]);
  }
  Wire.endTransmission();
}

void setMotorSpeed(int8_t speedLeft, int8_t speedRight) { // Set the motor speed
  int8_t data[4] = {speedLeft, 0, speedRight, 0};
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, (uint8_t *)data, 4);
}

void resetEncoders() { // Reset the encoder values
  int32_t resetValue[4] = {0, 0, 0, 0};
  WireWriteDataArray(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t *)resetValue, 16);
}

int WireReadDataArray(uint8_t reg, uint8_t *val, unsigned int len) { // Read data from the I2C bus
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

void moveCar(int8_t speedLeft, int8_t speedRight, unsigned long duration) { // Move the car for a specified duration
  setMotorSpeed(speedLeft, speedRight); 
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    // Non-blocking delay
  }
  setMotorSpeed(SPEED_STOP, SPEED_STOP);
}

// void moveServo(int angle) {
//   myServo.write(angle);
//   unsigned long startTime = millis();
//   while (millis() - startTime < angleDelay) {
//     // Non-blocking delay
//   }
// }

void setup() {
  Wire.begin();
  myServo.attach(servoPin);
  delay(200);
  myServo.write(90);
  uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
  uint8_t MotorEncoderPolarity = 0;
  WireWriteDataArray(MOTOR_TYPE_ADDR, &MotorType, 1); // Set the motor type
  delay(5);
  WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1); // Set the encoder polarity

  Serial.begin(9600);
  Serial1.begin(9600);
  // Serial1.println("Speed Serial communication started");
  // Serial.println("Setup Complete");
  // Serial.println("Target Ticks:");
  // Serial.println(desiredTicksPerInterval);
  resetEncoders();
  t_last = millis();
}

void loop() {
  // Check if data is available on the serial port
  if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');  // Read the data until newline character
    int commaIndex = data.indexOf(',');          // Find the comma that separates v and omega

    if (commaIndex != -1) {
      // Extract v (linear velocity) and omega (angular velocity)
      String vString = data.substring(0, commaIndex);
      String omegaString = data.substring(commaIndex + 1);

      // Convert the extracted values to floats
      desiredSpeed = vString.toFloat();
      angularVel = omegaString.toFloat();

      // Serial1.print("Received v: ");
      // Serial1.print(desiredSpeed);
      // Serial1.print(", omega: ");
      // Serial1.println(angularVel);
    } else {
      Serial1.println("Invalid data format. Expected 'v,omega'");
    }
  }

  // Get the elapsed time [ms]
  t_now = millis();

  if (t_now - t_last >= T) {
    WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t*)EncodeTotal, 16); // Read the current encoder values

    
    // Serial.print("Encode 1 (L-) Before = ");
    // Serial.println(-EncodeTotal[0]);

    // Sanity check on encoder values
    if (EncodeTotal[0] > desiredTicksPerInterval+1000 || EncodeTotal[0] < (desiredTicksPerInterval+1000)*-1) { // Reduced threshold for quicker spike detection
        // Serial.println("Encoder value exceeded threshold!");
        EncodeTotal[0] = prevEncoder[0]; // Ignore the value if it exceeds the threshold
      } 
    prevEncoder[0] = EncodeTotal[0];

    // Debugging: Print encoder values
    // Serial.print("Pre = ");
    // Serial.println(prevEncoder[0]);

    // Update the buffer with new readings
    for (int i = 0; i < 4; i++) {
      encoderBuffer[i][bufferIndex] = EncodeTotal[i];
    }
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;

    // Calculate the moving average
    for (int i = 0; i < 4; i++) {
      int32_t sum = 0;
      for (int j = 0; j < FILTER_SIZE; j++) {
        sum += encoderBuffer[i][j];
      }
      EncodeTotal[i] = sum / FILTER_SIZE;
    }
   
    // Serial.print("Encode 1 (L-) After = ");
    // Serial.println(-EncodeTotal[0]);
    // Serial.print(" Encode 2 (R+) = ");
    // Serial.println(EncodeTotal[2]);
    // Serial.print("Time = ");
    // Serial.println(t_now - t_last);

     // Estimate the rotational speed of the wheels [rad/s]
    float omegaR = 2.0 * PI * ((double)EncodeTotal[2] / (double)TPR) * 1000.0 / (double)(t_now - t_last); // Right wheel
    float omegaL = 2.0 * PI * (-(double)EncodeTotal[0] / (double)TPR) * 1000.0 / (double)(t_now - t_last); // Left wheel
    float omega = (omegaR + omegaL) / 2.0; // Average speed of the wheels

    // Debugging: Print intermediate speed calculation values
    // Serial.print("Intermediate Omega: ");
    // Serial.println(omega);

    currentspeed = omega * (wheelDiameter / 2.0); // Current speed in m/s
    float currentAngularVel = (omegaR - omegaL) / L_diff; // Current angular velocity in rad/s


    // Debugging: Print calculated current speed
     Serial1.print("Current Speed: ");
     Serial1.println(currentspeed);

    error = desiredSpeed - currentspeed; // Error term

    // Debugging: Print error value
    // Serial.print("Error: ");
    // Serial.println(error);

    integral += error * (t_now - t_last); // Integral term

    // Debugging: Print integral value
    // Serial.print("Integral: ");
    // Serial.println(integral);

    output = (kp * error) + (ki * integral); // PI controller output

    if (angularVel == 0) {
      currentAngularVel = 0;
      delta = 90;
      myServo.write(delta);
    } else if (currentspeed != 0) {
      float angle = atan((angularVel * L_steering) / currentspeed) * (180 / PI); // Servo angle in degrees
      delta = 90 - angle; // Adjust the angle relative to the straight position
    } else {
      delta = 90; // Straight if current speed is zero
      myServo.write(delta);
    }

    //Delta angle
    // Serial.print("Delta= ");
    // Serial.println(delta);

    // Serial.print(" Output= ");
    // Serial.println(output); // Print the output value for debugging

    // Limit output to max speed range and prevent integral windup
    if (output > MAX_SPEED) {
      output = MAX_SPEED;
      integral -= error * (t_now - t_last); // Prevent integral windup
    } else if (output < -MAX_SPEED) {
      output = -MAX_SPEED;
      integral -= error * (t_now - t_last); // Prevent integral windup
    }

    // Limit delta to valid servo angles
    if (delta < 0) {
      delta = 0;
    } else if (delta > 180) {
      delta = 180;
    }

    if (abs(delta - prevDelta) > SERVO_THRESHOLD) {
      myServo.write(delta);
      
    }

    prevDelta = delta; // Record the current delta value

    // Reset the encoder ticks counter
    resetEncoders();

    unsigned long dt_ms = t_now - t_last;
    float dt_s = dt_ms / 1000.0;

    // Set motor speed based on PI output (adjusting for motor direction)
    forwardSpeed[0] = -(int32_t)output; // Reverse direction for motor 1
    forwardSpeed[2] = (int32_t)output;  // Normal direction for motor 2
    setMotorSpeed(forwardSpeed[0], forwardSpeed[2]); // Set the motor speed
    Serial.print(currentspeed,6);
    Serial.print(",");
    Serial.print(currentAngularVel,6);
    Serial.print(",");
    Serial.println(dt_s,6);
  
    // Record the current time [ms]
    t_last = t_now;
  }
}
