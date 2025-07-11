// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ros2_benchmark_interfaces:srv/StopMonitoring.idl
// generated code does not contain a copyright notice
#include "ros2_benchmark_interfaces/srv/detail/stop_monitoring__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ros2_benchmark_interfaces/srv/detail/stop_monitoring__struct.hpp"

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

namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
cdr_serialize(
  const ros2_benchmark_interfaces::srv::StopMonitoring_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: structure_needs_at_least_one_member
  cdr << ros_message.structure_needs_at_least_one_member;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros2_benchmark_interfaces::srv::StopMonitoring_Request & ros_message)
{
  // Member: structure_needs_at_least_one_member
  cdr >> ros_message.structure_needs_at_least_one_member;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
get_serialized_size(
  const ros2_benchmark_interfaces::srv::StopMonitoring_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message.structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
max_serialized_size_StopMonitoring_Request(
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


  // Member: structure_needs_at_least_one_member
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
    using DataType = ros2_benchmark_interfaces::srv::StopMonitoring_Request;
    is_plain =
      (
      offsetof(DataType, structure_needs_at_least_one_member) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _StopMonitoring_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ros2_benchmark_interfaces::srv::StopMonitoring_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _StopMonitoring_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ros2_benchmark_interfaces::srv::StopMonitoring_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _StopMonitoring_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ros2_benchmark_interfaces::srv::StopMonitoring_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _StopMonitoring_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_StopMonitoring_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _StopMonitoring_Request__callbacks = {
  "ros2_benchmark_interfaces::srv",
  "StopMonitoring_Request",
  _StopMonitoring_Request__cdr_serialize,
  _StopMonitoring_Request__cdr_deserialize,
  _StopMonitoring_Request__get_serialized_size,
  _StopMonitoring_Request__max_serialized_size
};

static rosidl_message_type_support_t _StopMonitoring_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_StopMonitoring_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ros2_benchmark_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2_benchmark_interfaces::srv::StopMonitoring_Request>()
{
  return &ros2_benchmark_interfaces::srv::typesupport_fastrtps_cpp::_StopMonitoring_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2_benchmark_interfaces, srv, StopMonitoring_Request)() {
  return &ros2_benchmark_interfaces::srv::typesupport_fastrtps_cpp::_StopMonitoring_Request__handle;
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
namespace ros2_benchmark_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const ros2_benchmark_interfaces::msg::TimestampedMessageArray &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ros2_benchmark_interfaces::msg::TimestampedMessageArray &);
size_t get_serialized_size(
  const ros2_benchmark_interfaces::msg::TimestampedMessageArray &,
  size_t current_alignment);
size_t
max_serialized_size_TimestampedMessageArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace ros2_benchmark_interfaces

// functions for ros2_benchmark_interfaces::msg::TimestampedMessageArray already declared above


namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
cdr_serialize(
  const ros2_benchmark_interfaces::srv::StopMonitoring_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: start_timestamps
  ros2_benchmark_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.start_timestamps,
    cdr);
  // Member: end_timestamps
  ros2_benchmark_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.end_timestamps,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros2_benchmark_interfaces::srv::StopMonitoring_Response & ros_message)
{
  // Member: start_timestamps
  ros2_benchmark_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.start_timestamps);

  // Member: end_timestamps
  ros2_benchmark_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.end_timestamps);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
get_serialized_size(
  const ros2_benchmark_interfaces::srv::StopMonitoring_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: start_timestamps

  current_alignment +=
    ros2_benchmark_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.start_timestamps, current_alignment);
  // Member: end_timestamps

  current_alignment +=
    ros2_benchmark_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.end_timestamps, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
max_serialized_size_StopMonitoring_Response(
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


  // Member: start_timestamps
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ros2_benchmark_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_TimestampedMessageArray(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: end_timestamps
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ros2_benchmark_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_TimestampedMessageArray(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2_benchmark_interfaces::srv::StopMonitoring_Response;
    is_plain =
      (
      offsetof(DataType, end_timestamps) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _StopMonitoring_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ros2_benchmark_interfaces::srv::StopMonitoring_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _StopMonitoring_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ros2_benchmark_interfaces::srv::StopMonitoring_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _StopMonitoring_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ros2_benchmark_interfaces::srv::StopMonitoring_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _StopMonitoring_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_StopMonitoring_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _StopMonitoring_Response__callbacks = {
  "ros2_benchmark_interfaces::srv",
  "StopMonitoring_Response",
  _StopMonitoring_Response__cdr_serialize,
  _StopMonitoring_Response__cdr_deserialize,
  _StopMonitoring_Response__get_serialized_size,
  _StopMonitoring_Response__max_serialized_size
};

static rosidl_message_type_support_t _StopMonitoring_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_StopMonitoring_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ros2_benchmark_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2_benchmark_interfaces::srv::StopMonitoring_Response>()
{
  return &ros2_benchmark_interfaces::srv::typesupport_fastrtps_cpp::_StopMonitoring_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2_benchmark_interfaces, srv, StopMonitoring_Response)() {
  return &ros2_benchmark_interfaces::srv::typesupport_fastrtps_cpp::_StopMonitoring_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _StopMonitoring__callbacks = {
  "ros2_benchmark_interfaces::srv",
  "StopMonitoring",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2_benchmark_interfaces, srv, StopMonitoring_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2_benchmark_interfaces, srv, StopMonitoring_Response)(),
};

static rosidl_service_type_support_t _StopMonitoring__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_StopMonitoring__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ros2_benchmark_interfaces
const rosidl_service_type_support_t *
get_service_type_support_handle<ros2_benchmark_interfaces::srv::StopMonitoring>()
{
  return &ros2_benchmark_interfaces::srv::typesupport_fastrtps_cpp::_StopMonitoring__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2_benchmark_interfaces, srv, StopMonitoring)() {
  return &ros2_benchmark_interfaces::srv::typesupport_fastrtps_cpp::_StopMonitoring__handle;
}

#ifdef __cplusplus
}
#endif
