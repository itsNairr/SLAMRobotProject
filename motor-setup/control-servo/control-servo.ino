// Include the servo library
#include <Servo.h>

// Create a Servo object
Servo myServo;

// Define the servo pin
const int servoPin = 12;

void setup() {
  // Attach the servo to a pin
  myServo.attach(servoPin);
  
  // Rotate the servo across a range of angles
  for (int angle = 0; angle <= 180; angle += 10) {
    myServo.write(angle); // Set servo to current angle
    delay(500); // Wait 0.5 seconds
  }
  
  // Return the servo to 90 degrees (neutral position)
  myServo.write(90);
  delay(500);
}

void loop() {
  // The setup routine runs once and the loop routine does nothing in this test
}