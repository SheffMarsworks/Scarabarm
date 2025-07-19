#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

// Dynamixel setup: OpenRB-150, TX pin 2
Dynamixel2Arduino dxl(Serial1, 2);

#define DXL_ID_RIGHT 1
#define DXL_ID_LEFT  2

// Gripper angle positions (in degrees)
float open_pos_right = 90.0;
float closed_pos_right = 180.0;
float open_pos_left = 90.0;
float closed_pos_left = 0.0;

// Current limits (in mA)
int min_current = 100;
int max_current = 900;

void setup() {
  Serial.begin(115200);    // USB serial for command input
  while (!Serial);         // Wait for serial to be ready

  dxl.begin(57600);        // Start Dynamixel communication
  dxl.setPortProtocolVersion(2.0);

  Serial.println("Gripper setup starting...");

  // Ping both motors
  if (dxl.ping(DXL_ID_RIGHT)) {
    Serial.println("Right motor connected.");
  } else {
    Serial.println("Right motor NOT found.");
    while (1);
  }

  if (dxl.ping(DXL_ID_LEFT)) {
    Serial.println("Left motor connected.");
  } else {
    Serial.println("Left motor NOT found.");
    while (1);
  }

  // Torque off before changing modes
  dxl.torqueOff(DXL_ID_RIGHT);
  dxl.torqueOff(DXL_ID_LEFT);

  // Set control mode
  dxl.setOperatingMode(DXL_ID_RIGHT, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(DXL_ID_LEFT, OP_CURRENT_BASED_POSITION);

  // Enable torque
  bool resultR = dxl.torqueOn(DXL_ID_RIGHT);
  bool resultL = dxl.torqueOn(DXL_ID_LEFT);

  Serial.print("Torque Right: ");
  Serial.println(resultR ? "Success" : "Failed");

  Serial.print("Torque Left: ");
  Serial.println(resultL ? "Success" : "Failed");

  Serial.println("Gripper ready. Send command like: GRIP 0.0 (open) to GRIP 1.0 (close)");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    if (input.startsWith("GRIP")) {
      float val = input.substring(5).toFloat();
      val = constrain(val, 0.0, 1.0);

      float pos_r = open_pos_right + val * (closed_pos_right - open_pos_right);
      float pos_l = open_pos_left + val * (closed_pos_left - open_pos_left);
      int force = min_current + val * (max_current - min_current);

      dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID_RIGHT, force);
      dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID_LEFT, force);
      dxl.setGoalPosition(DXL_ID_RIGHT, pos_r, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_LEFT, pos_l, UNIT_DEGREE);

      Serial.print("Received grip ");
      Serial.println(val);
    } else {
      Serial.println("Unknown command.");
    }
  }
}

