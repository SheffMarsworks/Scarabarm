// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from odrive_can:srv/AxisState.idl
// generated code does not contain a copyright notice
#include "odrive_can/srv/detail/axis_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "odrive_can/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "odrive_can/srv/detail/axis_state__struct.h"
#include "odrive_can/srv/detail/axis_state__functions.h"
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


using _AxisState_Request__ros_msg_type = odrive_can__srv__AxisState_Request;

static bool _AxisState_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _AxisState_Request__ros_msg_type * ros_message = static_cast<const _AxisState_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: axis_requested_state
  {
    cdr << ros_message->axis_requested_state;
  }

  return true;
}

static bool _AxisState_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _AxisState_Request__ros_msg_type * ros_message = static_cast<_AxisState_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: axis_requested_state
  {
    cdr >> ros_message->axis_requested_state;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_odrive_can
size_t get_serialized_size_odrive_can__srv__AxisState_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _AxisState_Request__ros_msg_type * ros_message = static_cast<const _AxisState_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name axis_requested_state
  {
    size_t item_size = sizeof(ros_message->axis_requested_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _AxisState_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_odrive_can__srv__AxisState_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_odrive_can
size_t max_serialized_size_odrive_can__srv__AxisState_Request(
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

  // member: axis_requested_state
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
    using DataType = odrive_can__srv__AxisState_Request;
    is_plain =
      (
      offsetof(DataType, axis_requested_state) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _AxisState_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_odrive_can__srv__AxisState_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_AxisState_Request = {
  "odrive_can::srv",
  "AxisState_Request",
  _AxisState_Request__cdr_serialize,
  _AxisState_Request__cdr_deserialize,
  _AxisState_Request__get_serialized_size,
  _AxisState_Request__max_serialized_size
};

static rosidl_message_type_support_t _AxisState_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_AxisState_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, odrive_can, srv, AxisState_Request)() {
  return &_AxisState_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "odrive_can/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "odrive_can/srv/detail/axis_state__struct.h"
// already included above
// #include "odrive_can/srv/detail/axis_state__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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


using _AxisState_Response__ros_msg_type = odrive_can__srv__AxisState_Response;

static bool _AxisState_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _AxisState_Response__ros_msg_type * ros_message = static_cast<const _AxisState_Response__ros_msg_type *>(untyped_ros_message);
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

  return true;
}

static bool _AxisState_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _AxisState_Response__ros_msg_type * ros_message = static_cast<_AxisState_Response__ros_msg_type *>(untyped_ros_message);
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

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_odrive_can
size_t get_serialized_size_odrive_can__srv__AxisState_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _AxisState_Response__ros_msg_type * ros_message = static_cast<const _AxisState_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

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

  return current_alignment - initial_alignment;
}

static uint32_t _AxisState_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_odrive_can__srv__AxisState_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_odrive_can
size_t max_serialized_size_odrive_can__srv__AxisState_Response(
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

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = odrive_can__srv__AxisState_Response;
    is_plain =
      (
      offsetof(DataType, procedure_result) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _AxisState_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_odrive_can__srv__AxisState_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_AxisState_Response = {
  "odrive_can::srv",
  "AxisState_Response",
  _AxisState_Response__cdr_serialize,
  _AxisState_Response__cdr_deserialize,
  _AxisState_Response__get_serialized_size,
  _AxisState_Response__max_serialized_size
};

static rosidl_message_type_support_t _AxisState_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_AxisState_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, odrive_can, srv, AxisState_Response)() {
  return &_AxisState_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "odrive_can/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "odrive_can/srv/axis_state.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t AxisState__callbacks = {
  "odrive_can::srv",
  "AxisState",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, odrive_can, srv, AxisState_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, odrive_can, srv, AxisState_Response)(),
};

static rosidl_service_type_support_t AxisState__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &AxisState__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, odrive_can, srv, AxisState)() {
  return &AxisState__handle;
}

#if defined(__cplusplus)
}
#endif
