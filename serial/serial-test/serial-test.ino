void setup() {
  Serial.begin(9600);
}

void loop() {
  float v = 1.0;    // Example linear velocity
  float omega = 0;  // Example angular velocity

  // Send the velocities as comma-separated values
  Serial.print(v);
  Serial.print(",");
  Serial.println(omega);

  delay(100); // Send data every 100 ms
}
