// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from odrive_can:msg/ControllerStatus.idl
// generated code does not contain a copyright notice
#include "odrive_can/msg/detail/controller_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "odrive_can/msg/detail/controller_status__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace odrive_can
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
cdr_serialize(
  const odrive_can::msg::ControllerStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: pos_estimate
  cdr << ros_message.pos_estimate;
  // Member: vel_estimate
  cdr << ros_message.vel_estimate;
  // Member: torque_target
  cdr << ros_message.torque_target;
  // Member: torque_estimate
  cdr << ros_message.torque_estimate;
  // Member: iq_setpoint
  cdr << ros_message.iq_setpoint;
  // Member: iq_measured
  cdr << ros_message.iq_measured;
  // Member: active_errors
  cdr << ros_message.active_errors;
  // Member: axis_state
  cdr << ros_message.axis_state;
  // Member: procedure_result
  cdr << ros_message.procedure_result;
  // Member: trajectory_done_flag
  cdr << (ros_message.trajectory_done_flag ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  odrive_can::msg::ControllerStatus & ros_message)
{
  // Member: pos_estimate
  cdr >> ros_message.pos_estimate;

  // Member: vel_estimate
  cdr >> ros_message.vel_estimate;

  // Member: torque_target
  cdr >> ros_message.torque_target;

  // Member: torque_estimate
  cdr >> ros_message.torque_estimate;

  // Member: iq_setpoint
  cdr >> ros_message.iq_setpoint;

  // Member: iq_measured
  cdr >> ros_message.iq_measured;

  // Member: active_errors
  cdr >> ros_message.active_errors;

  // Member: axis_state
  cdr >> ros_message.axis_state;

  // Member: procedure_result
  cdr >> ros_message.procedure_result;

  // Member: trajectory_done_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.trajectory_done_flag = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
get_serialized_size(
  const odrive_can::msg::ControllerStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: pos_estimate
  {
    size_t item_size = sizeof(ros_message.pos_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: vel_estimate
  {
    size_t item_size = sizeof(ros_message.vel_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: torque_target
  {
    size_t item_size = sizeof(ros_message.torque_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: torque_estimate
  {
    size_t item_size = sizeof(ros_message.torque_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: iq_setpoint
  {
    size_t item_size = sizeof(ros_message.iq_setpoint);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: iq_measured
  {
    size_t item_size = sizeof(ros_message.iq_measured);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: active_errors
  {
    size_t item_size = sizeof(ros_message.active_errors);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: axis_state
  {
    size_t item_size = sizeof(ros_message.axis_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: procedure_result
  {
    size_t item_size = sizeof(ros_message.procedure_result);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: trajectory_done_flag
  {
    size_t item_size = sizeof(ros_message.trajectory_done_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
max_serialized_size_ControllerStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: pos_estimate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vel_estimate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: torque_target
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: torque_estimate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: iq_setpoint
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: iq_measured
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: active_errors
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: axis_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: procedure_result
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: trajectory_done_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = odrive_can::msg::ControllerStatus;
    is_plain =
      (
      offsetof(DataType, trajectory_done_flag) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ControllerStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const odrive_can::msg::ControllerStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ControllerStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<odrive_can::msg::ControllerStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ControllerStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const odrive_can::msg::ControllerStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ControllerStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ControllerStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ControllerStatus__callbacks = {
  "odrive_can::msg",
  "ControllerStatus",
  _ControllerStatus__cdr_serialize,
  _ControllerStatus__cdr_deserialize,
  _ControllerStatus__get_serialized_size,
  _ControllerStatus__max_serialized_size
};

static rosidl_message_type_support_t _ControllerStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ControllerStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace odrive_can

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_odrive_can
const rosidl_message_type_support_t *
get_message_type_support_handle<odrive_can::msg::ControllerStatus>()
{
  return &odrive_can::msg::typesupport_fastrtps_cpp::_ControllerStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, odrive_can, msg, ControllerStatus)() {
  return &odrive_can::msg::typesupport_fastrtps_cpp::_ControllerStatus__handle;
}

#ifdef __cplusplus
}
#endif
