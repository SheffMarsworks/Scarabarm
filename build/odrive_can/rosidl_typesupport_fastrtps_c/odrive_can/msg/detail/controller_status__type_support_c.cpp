// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from odrive_can:msg/ControllerStatus.idl
// generated code does not contain a copyright notice
#include "odrive_can/msg/detail/controller_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "odrive_can/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "odrive_can/msg/detail/controller_status__struct.h"
#include "odrive_can/msg/detail/controller_status__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _ControllerStatus__ros_msg_type = odrive_can__msg__ControllerStatus;

static bool _ControllerStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ControllerStatus__ros_msg_type * ros_message = static_cast<const _ControllerStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: pos_estimate
  {
    cdr << ros_message->pos_estimate;
  }

  // Field name: vel_estimate
  {
    cdr << ros_message->vel_estimate;
  }

  // Field name: torque_target
  {
    cdr << ros_message->torque_target;
  }

  // Field name: torque_estimate
  {
    cdr << ros_message->torque_estimate;
  }

  // Field name: iq_setpoint
  {
    cdr << ros_message->iq_setpoint;
  }

  // Field name: iq_measured
  {
    cdr << ros_message->iq_measured;
  }

  // Field name: active_errors
  {
    cdr << ros_message->active_errors;
  }

  // Field name: axis_state
  {
    cdr << ros_message->axis_state;
  }

  // Field name: procedure_result
  {
    cdr << ros_message->procedure_result;
  }

  // Field name: trajectory_done_flag
  {
    cdr << (ros_message->trajectory_done_flag ? true : false);
  }

  return true;
}

static bool _ControllerStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ControllerStatus__ros_msg_type * ros_message = static_cast<_ControllerStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: pos_estimate
  {
    cdr >> ros_message->pos_estimate;
  }

  // Field name: vel_estimate
  {
    cdr >> ros_message->vel_estimate;
  }

  // Field name: torque_target
  {
    cdr >> ros_message->torque_target;
  }

  // Field name: torque_estimate
  {
    cdr >> ros_message->torque_estimate;
  }

  // Field name: iq_setpoint
  {
    cdr >> ros_message->iq_setpoint;
  }

  // Field name: iq_measured
  {
    cdr >> ros_message->iq_measured;
  }

  // Field name: active_errors
  {
    cdr >> ros_message->active_errors;
  }

  // Field name: axis_state
  {
    cdr >> ros_message->axis_state;
  }

  // Field name: procedure_result
  {
    cdr >> ros_message->procedure_result;
  }

  // Field name: trajectory_done_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->trajectory_done_flag = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_odrive_can
size_t get_serialized_size_odrive_can__msg__ControllerStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ControllerStatus__ros_msg_type * ros_message = static_cast<const _ControllerStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name pos_estimate
  {
    size_t item_size = sizeof(ros_message->pos_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vel_estimate
  {
    size_t item_size = sizeof(ros_message->vel_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name torque_target
  {
    size_t item_size = sizeof(ros_message->torque_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name torque_estimate
  {
    size_t item_size = sizeof(ros_message->torque_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name iq_setpoint
  {
    size_t item_size = sizeof(ros_message->iq_setpoint);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name iq_measured
  {
    size_t item_size = sizeof(ros_message->iq_measured);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name active_errors
  {
    size_t item_size = sizeof(ros_message->active_errors);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name axis_state
  {
    size_t item_size = sizeof(ros_message->axis_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name procedure_result
  {
    size_t item_size = sizeof(ros_message->procedure_result);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name trajectory_done_flag
  {
    size_t item_size = sizeof(ros_message->trajectory_done_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ControllerStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_odrive_can__msg__ControllerStatus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_odrive_can
size_t max_serialized_size_odrive_can__msg__ControllerStatus(
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

  // member: pos_estimate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: vel_estimate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: torque_target
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: torque_estimate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: iq_setpoint
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: iq_measured
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: active_errors
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: axis_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: procedure_result
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: trajectory_done_flag
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
    using DataType = odrive_can__msg__ControllerStatus;
    is_plain =
      (
      offsetof(DataType, trajectory_done_flag) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ControllerStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_odrive_can__msg__ControllerStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ControllerStatus = {
  "odrive_can::msg",
  "ControllerStatus",
  _ControllerStatus__cdr_serialize,
  _ControllerStatus__cdr_deserialize,
  _ControllerStatus__get_serialized_size,
  _ControllerStatus__max_serialized_size
};

static rosidl_message_type_support_t _ControllerStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ControllerStatus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, odrive_can, msg, ControllerStatus)() {
  return &_ControllerStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
