#include <Servo.h>

// Receives newline-terminated integer angles from the Python application.
// Adjust SERVO_PIN and angle limits to match your mechanical setup.

constexpr uint8_t SERVO_PIN = 9;
constexpr int START_ANGLE = 90;
constexpr int MIN_ANGLE = 0;
constexpr int MAX_ANGLE = 180;

Servo panServo;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(30);
  panServo.attach(SERVO_PIN);
  panServo.write(START_ANGLE);
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  const int requestedAngle = Serial.parseInt();
  while (Serial.available() > 0) {
    Serial.read();  // Discard the newline and any stale bytes.
  }

  if (requestedAngle >= MIN_ANGLE && requestedAngle <= MAX_ANGLE) {
    panServo.write(requestedAngle);
  }
}
