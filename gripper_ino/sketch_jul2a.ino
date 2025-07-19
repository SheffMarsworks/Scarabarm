#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <Dynamixel2Arduino.h>


#define MICRO_ROS_TRANSPORT_SERIAL
#define DXL_ID_RIGHT 1
#define DXL_ID_LEFT  2

// Dynamixel communication on Serial1 (OpenRB-150)
Dynamixel2Arduino dxl(Serial1);
using namespace ControlTableItem;

// ROS 2 variables
rcl_subscription_t gripper_sub;
std_msgs__msg__Float32 gripper_msg;
rclc_executor_t executor;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;

// Gripper angle limits
float open_pos_right   = 90.0;
float closed_pos_right = 180.0;
float open_pos_left    = 90.0;
float closed_pos_left  = 0.0;

void gripper_callback(const std_msgs__msg__Float32 *msg) {
  float cmd = msg->data;
  if (cmd < 0.0) cmd = 0.0;
  if (cmd > 1.0) cmd = 1.0;

  if (cmd == 0.0) {
    // Fully open
    dxl.setGoalPosition(DXL_ID_RIGHT, 90.0, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_LEFT, 90.0, UNIT_DEGREE);
    Serial.println("Gripper OPEN");
  } else {
    // Closed with force mapping
    int min_force = 100;  // Safe minimum
    int max_force = 900;  // XM430 max safe range
    int goal_current = min_force + (cmd * (max_force - min_force));

    dxl.writeControlTableItem(GOAL_CURRENT, DXL_ID_RIGHT, goal_current);
    dxl.writeControlTableItem(GOAL_CURRENT, DXL_ID_LEFT, goal_current);

    dxl.setGoalPosition(DXL_ID_RIGHT, 180.0, UNIT_DEGREE);  // Close
    dxl.setGoalPosition(DXL_ID_LEFT, 0.0, UNIT_DEGREE);
    
    Serial.print("Gripper CLOSED with force: ");
    Serial.println(goal_current);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Starting micro-ROS gripper...");

  set_microros_transports();
  delay(2000);  // Wait for agent

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "gripper_node", "", &support);

  rclc_subscription_init_default(
    &gripper_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/gripper_command"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor, &gripper_sub, &gripper_msg,
    reinterpret_cast<rclc_callback_t>(gripper_callback), ON_NEW_DATA
  );

  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  if (!dxl.ping(DXL_ID_RIGHT) || !dxl.ping(DXL_ID_LEFT)) {
    Serial.println("Gripper motors not found!");
    while (1);
  }

  dxl.torqueOff(DXL_ID_RIGHT);
  dxl.torqueOff(DXL_ID_LEFT);
  dxl.setOperatingMode(DXL_ID_RIGHT, OP_CURRENT_BASED_POSITION);
  dxl.setOperatingMode(DXL_ID_LEFT,  OP_CURRENT_BASED_POSITION);
  dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID_RIGHT, 350);
  dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID_LEFT, 350);
  dxl.torqueOn(DXL_ID_RIGHT);
  dxl.torqueOn(DXL_ID_LEFT);

  Serial.println("Gripper ready.");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
