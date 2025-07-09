// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from odrive_can:msg/ODriveStatus.idl
// generated code does not contain a copyright notice
#include "odrive_can/msg/detail/o_drive_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "odrive_can/msg/detail/o_drive_status__struct.hpp"

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
  const odrive_can::msg::ODriveStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: bus_voltage
  cdr << ros_message.bus_voltage;
  // Member: bus_current
  cdr << ros_message.bus_current;
  // Member: fet_temperature
  cdr << ros_message.fet_temperature;
  // Member: motor_temperature
  cdr << ros_message.motor_temperature;
  // Member: active_errors
  cdr << ros_message.active_errors;
  // Member: disarm_reason
  cdr << ros_message.disarm_reason;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  odrive_can::msg::ODriveStatus & ros_message)
{
  // Member: bus_voltage
  cdr >> ros_message.bus_voltage;

  // Member: bus_current
  cdr >> ros_message.bus_current;

  // Member: fet_temperature
  cdr >> ros_message.fet_temperature;

  // Member: motor_temperature
  cdr >> ros_message.motor_temperature;

  // Member: active_errors
  cdr >> ros_message.active_errors;

  // Member: disarm_reason
  cdr >> ros_message.disarm_reason;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
get_serialized_size(
  const odrive_can::msg::ODriveStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: bus_voltage
  {
    size_t item_size = sizeof(ros_message.bus_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: bus_current
  {
    size_t item_size = sizeof(ros_message.bus_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fet_temperature
  {
    size_t item_size = sizeof(ros_message.fet_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor_temperature
  {
    size_t item_size = sizeof(ros_message.motor_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: active_errors
  {
    size_t item_size = sizeof(ros_message.active_errors);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: disarm_reason
  {
    size_t item_size = sizeof(ros_message.disarm_reason);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
max_serialized_size_ODriveStatus(
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


  // Member: bus_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: bus_current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: fet_temperature
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: motor_temperature
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

  // Member: disarm_reason
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = odrive_can::msg::ODriveStatus;
    is_plain =
      (
      offsetof(DataType, disarm_reason) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ODriveStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const odrive_can::msg::ODriveStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ODriveStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<odrive_can::msg::ODriveStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ODriveStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const odrive_can::msg::ODriveStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ODriveStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ODriveStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ODriveStatus__callbacks = {
  "odrive_can::msg",
  "ODriveStatus",
  _ODriveStatus__cdr_serialize,
  _ODriveStatus__cdr_deserialize,
  _ODriveStatus__get_serialized_size,
  _ODriveStatus__max_serialized_size
};

static rosidl_message_type_support_t _ODriveStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ODriveStatus__callbacks,
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
get_message_type_support_handle<odrive_can::msg::ODriveStatus>()
{
  return &odrive_can::msg::typesupport_fastrtps_cpp::_ODriveStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, odrive_can, msg, ODriveStatus)() {
  return &odrive_can::msg::typesupport_fastrtps_cpp::_ODriveStatus__handle;
}

#ifdef __cplusplus
}
#endif
