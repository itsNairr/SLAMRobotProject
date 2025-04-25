#include <Wire.h>
#include <Servo.h>


#define I2C_ADDR        0x34

#define GET_LOW_BYTE(A) (uint8_t)((A))
// Macro function to get the low byte of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
// Macro function to get the high byte of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
// Macro function to merge A as the high byte and B as the low byte into a 16-bit integer

#define ADC_BAT_ADDR                  0
#define MOTOR_TYPE_ADDR               20 // Motor type setting for encoder motor
#define MOTOR_ENCODER_POLARITY_ADDR   21 // Set the encoder direction polarity,
// If the motor speed is not controlled at all, either rotating at the fastest speed or stopped,
// you can reset the value of this address
// Range: 0 or 1, default: 0
#define MOTOR_FIXED_PWM_ADDR      31 // Fixed PWM control, open-loop control, range: (-100~100)
//#define SERVOS_ADDR_CMD 40        
#define MOTOR_FIXED_SPEED_ADDR    51 // Fixed speed control, closed-loop control,
// Unit: pulse count per 10 milliseconds, range: (depending on the specific encoder motor, influenced by the number of encoder lines, voltage, load, etc., generally around ±50)

#define MOTOR_ENCODER_TOTAL_ADDR  60 // Total pulse count for each of the 4 encoder motors
// If the pulse count per revolution of the motor is known as U, and the diameter of the wheel is known as D,
// then the distance traveled by each wheel can be obtained through pulse counting
// For example, if the total pulse count of motor 1 is P, then the distance traveled is (P/U) * (3.14159*D)
// For different motors, you can test the pulse count per revolution U manually by rotating the wheel 10 times and reading the pulse count, then take the average


// Motor type specific values
#define MOTOR_TYPE_WITHOUT_ENCODER        0
#define MOTOR_TYPE_TT                     1
#define MOTOR_TYPE_N20                    2
#define MOTOR_TYPE_JGB37_520_12V_110RPM   3 // 44 pulses per revolution of the magnetic ring, gear ratio: 90, default


Servo myServo;
const int servoPin = 12;


uint8_t data[20];
bool WireWriteByte(uint8_t val)   // Send byte data via I2C
{
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(val);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    return true;
}
bool WireWriteDataArray(uint8_t reg, uint8_t *val, unsigned int len)    // Send data via I2C
{
    unsigned int i;

    Wire.beginTransmission(I2C_ADDR);   // Set the starting address
    Wire.write(reg);    // Write the header address content
    for (i = 0; i < len; i++) {
        Wire.write(val[i]);   // Write the content passed in to I2C
    }
    if (Wire.endTransmission() != 0) {     // Check if the write is successful
        return false;
    }

    return true;
}

uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;    // Motor mode setting
uint8_t MotorEncoderPolarity = 0;     // Motor polarity setting
void setup()
{
  Wire.begin();
  myServo.attach(servoPin);
  delay(200);
  WireWriteDataArray(MOTOR_TYPE_ADDR, &MotorType, 1);
  delay(5);
  WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1);
  for (int angle = 0; angle <= 180; angle += 10) {
    myServo.write(angle); // Set servo to current angle
    delay(500); // Wait 0.5 seconds
  }
  myServo.write(90);
  delay(500);
}
int8_t car_forward[4] = {-16, 0, 16, 0};   // Forward
int8_t car_retreat[4] = {16, 0, -16, 0};   // Backward
int8_t car_stop[4] = {0, 0, 0, 0};

void loop()
{
  /* Car moves forward */
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_forward, 4);
  delay(4000);
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
  delay(1000);
  /* Car moves backward */
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_retreat, 4);
  delay(4000);
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
  delay(1000);
  myServo.write(45);  // Rotate to 90 degrees    
  /* Car moves to the left front, maintains the same direction after 4 seconds, and returns to the starting point. */
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_forward, 4);
  delay(4000);
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
  delay(1000); 
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_retreat, 4);
  delay(4000);
  myServo.write(90);
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
  delay(1000);
  myServo.write(145);
  /* Car moves to the right front, maintains the same direction after 4 seconds, and returns to the starting point. */
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_forward, 4);
  delay(4000);
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
  delay(1000);
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_retreat, 4);
  delay(4000);
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
  myServo.write(90);
  delay(1000); 
  
  while (1);
  
}
