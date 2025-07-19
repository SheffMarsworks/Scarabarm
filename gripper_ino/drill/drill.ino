/*
| Command      | Description                      |
| ------------ | -------------------------------- |
| `MOTOR 200`  | Spin drill forward at 200/255    |
| `MOTOR -150` | Spin drill in reverse at 150/255 |
| `MOTOR 0`    | Stop the motor                   |
| `GRIP_OPEN`  | Open gripper (servo = 0°)        |
| `GRIP_CLOSE` | Close gripper (servo = 90°)      |
*/

// Includes
#include <Servo.h>

// Motor + Encoder Pins
#define PWM_PIN     6
#define DIR_PIN     8

// Servo
#define SERVO_PIN 5

Servo gripper;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  gripper.attach(SERVO_PIN);

  Serial.begin(115200);
  Serial.println("End-effector ready (Serial)");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // Remove whitespace
    handleCommand(cmd);
  }
}

void handleCommand(String cmd) {
  if (cmd.startsWith("MOTOR ")) {
    int speed = cmd.substring(6).toInt();  // Extract speed
    setMotor(speed);
  } else if (cmd == "GRIP_OPEN") {
    gripper.write(0);   // Adjust angle as needed
    Serial.println("Gripper opened");
  } else if (cmd == "GRIP_CLOSE") {
    gripper.write(90);  // Adjust angle as needed
    Serial.println("Gripper closed");
  } else if (cmd == "STOP") {
    setMotor(0);
    Serial.println("Motor stopped");
  } else {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
  }
}

void setMotor(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(DIR_PIN, speed >= 0 ? HIGH : LOW);
  analogWrite(PWM_PIN, abs(speed));
  Serial.print("Motor power: "); Serial.println(speed);
}