// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from odrive_can:srv/AxisState.idl
// generated code does not contain a copyright notice
#include "odrive_can/srv/detail/axis_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "odrive_can/srv/detail/axis_state__struct.hpp"

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

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
cdr_serialize(
  const odrive_can::srv::AxisState_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: axis_requested_state
  cdr << ros_message.axis_requested_state;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  odrive_can::srv::AxisState_Request & ros_message)
{
  // Member: axis_requested_state
  cdr >> ros_message.axis_requested_state;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
get_serialized_size(
  const odrive_can::srv::AxisState_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: axis_requested_state
  {
    size_t item_size = sizeof(ros_message.axis_requested_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
max_serialized_size_AxisState_Request(
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


  // Member: axis_requested_state
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
    using DataType = odrive_can::srv::AxisState_Request;
    is_plain =
      (
      offsetof(DataType, axis_requested_state) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _AxisState_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const odrive_can::srv::AxisState_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _AxisState_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<odrive_can::srv::AxisState_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _AxisState_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const odrive_can::srv::AxisState_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _AxisState_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_AxisState_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _AxisState_Request__callbacks = {
  "odrive_can::srv",
  "AxisState_Request",
  _AxisState_Request__cdr_serialize,
  _AxisState_Request__cdr_deserialize,
  _AxisState_Request__get_serialized_size,
  _AxisState_Request__max_serialized_size
};

static rosidl_message_type_support_t _AxisState_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_AxisState_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace odrive_can

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_odrive_can
const rosidl_message_type_support_t *
get_message_type_support_handle<odrive_can::srv::AxisState_Request>()
{
  return &odrive_can::srv::typesupport_fastrtps_cpp::_AxisState_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, odrive_can, srv, AxisState_Request)() {
  return &odrive_can::srv::typesupport_fastrtps_cpp::_AxisState_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace odrive_can
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
cdr_serialize(
  const odrive_can::srv::AxisState_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: active_errors
  cdr << ros_message.active_errors;
  // Member: axis_state
  cdr << ros_message.axis_state;
  // Member: procedure_result
  cdr << ros_message.procedure_result;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  odrive_can::srv::AxisState_Response & ros_message)
{
  // Member: active_errors
  cdr >> ros_message.active_errors;

  // Member: axis_state
  cdr >> ros_message.axis_state;

  // Member: procedure_result
  cdr >> ros_message.procedure_result;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
get_serialized_size(
  const odrive_can::srv::AxisState_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

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

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_odrive_can
max_serialized_size_AxisState_Response(
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

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = odrive_can::srv::AxisState_Response;
    is_plain =
      (
      offsetof(DataType, procedure_result) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _AxisState_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const odrive_can::srv::AxisState_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _AxisState_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<odrive_can::srv::AxisState_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _AxisState_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const odrive_can::srv::AxisState_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _AxisState_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_AxisState_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _AxisState_Response__callbacks = {
  "odrive_can::srv",
  "AxisState_Response",
  _AxisState_Response__cdr_serialize,
  _AxisState_Response__cdr_deserialize,
  _AxisState_Response__get_serialized_size,
  _AxisState_Response__max_serialized_size
};

static rosidl_message_type_support_t _AxisState_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_AxisState_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace odrive_can

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_odrive_can
const rosidl_message_type_support_t *
get_message_type_support_handle<odrive_can::srv::AxisState_Response>()
{
  return &odrive_can::srv::typesupport_fastrtps_cpp::_AxisState_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, odrive_can, srv, AxisState_Response)() {
  return &odrive_can::srv::typesupport_fastrtps_cpp::_AxisState_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace odrive_can
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _AxisState__callbacks = {
  "odrive_can::srv",
  "AxisState",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, odrive_can, srv, AxisState_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, odrive_can, srv, AxisState_Response)(),
};

static rosidl_service_type_support_t _AxisState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_AxisState__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace odrive_can

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_odrive_can
const rosidl_service_type_support_t *
get_service_type_support_handle<odrive_can::srv::AxisState>()
{
  return &odrive_can::srv::typesupport_fastrtps_cpp::_AxisState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, odrive_can, srv, AxisState)() {
  return &odrive_can::srv::typesupport_fastrtps_cpp::_AxisState__handle;
}

#ifdef __cplusplus
}
#endif
