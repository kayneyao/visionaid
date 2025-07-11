// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ros2_benchmark_interfaces:srv/StartLoading.idl
// generated code does not contain a copyright notice
#include "ros2_benchmark_interfaces/srv/detail/start_loading__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ros2_benchmark_interfaces/srv/detail/start_loading__struct.hpp"

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
  const ros2_benchmark_interfaces::srv::StartLoading_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: start_time_offset_ns
  cdr << ros_message.start_time_offset_ns;
  // Member: end_time_offset_ns
  cdr << ros_message.end_time_offset_ns;
  // Member: repeat_data
  cdr << (ros_message.repeat_data ? true : false);
  // Member: publish_in_real_time
  cdr << (ros_message.publish_in_real_time ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros2_benchmark_interfaces::srv::StartLoading_Request & ros_message)
{
  // Member: start_time_offset_ns
  cdr >> ros_message.start_time_offset_ns;

  // Member: end_time_offset_ns
  cdr >> ros_message.end_time_offset_ns;

  // Member: repeat_data
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.repeat_data = tmp ? true : false;
  }

  // Member: publish_in_real_time
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.publish_in_real_time = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
get_serialized_size(
  const ros2_benchmark_interfaces::srv::StartLoading_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: start_time_offset_ns
  {
    size_t item_size = sizeof(ros_message.start_time_offset_ns);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: end_time_offset_ns
  {
    size_t item_size = sizeof(ros_message.end_time_offset_ns);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: repeat_data
  {
    size_t item_size = sizeof(ros_message.repeat_data);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: publish_in_real_time
  {
    size_t item_size = sizeof(ros_message.publish_in_real_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
max_serialized_size_StartLoading_Request(
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


  // Member: start_time_offset_ns
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: end_time_offset_ns
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: repeat_data
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: publish_in_real_time
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
    using DataType = ros2_benchmark_interfaces::srv::StartLoading_Request;
    is_plain =
      (
      offsetof(DataType, publish_in_real_time) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _StartLoading_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ros2_benchmark_interfaces::srv::StartLoading_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _StartLoading_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ros2_benchmark_interfaces::srv::StartLoading_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _StartLoading_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ros2_benchmark_interfaces::srv::StartLoading_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _StartLoading_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_StartLoading_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _StartLoading_Request__callbacks = {
  "ros2_benchmark_interfaces::srv",
  "StartLoading_Request",
  _StartLoading_Request__cdr_serialize,
  _StartLoading_Request__cdr_deserialize,
  _StartLoading_Request__get_serialized_size,
  _StartLoading_Request__max_serialized_size
};

static rosidl_message_type_support_t _StartLoading_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_StartLoading_Request__callbacks,
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
get_message_type_support_handle<ros2_benchmark_interfaces::srv::StartLoading_Request>()
{
  return &ros2_benchmark_interfaces::srv::typesupport_fastrtps_cpp::_StartLoading_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2_benchmark_interfaces, srv, StartLoading_Request)() {
  return &ros2_benchmark_interfaces::srv::typesupport_fastrtps_cpp::_StartLoading_Request__handle;
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
  const ros2_benchmark_interfaces::msg::TopicMessageTimestampArray &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ros2_benchmark_interfaces::msg::TopicMessageTimestampArray &);
size_t get_serialized_size(
  const ros2_benchmark_interfaces::msg::TopicMessageTimestampArray &,
  size_t current_alignment);
size_t
max_serialized_size_TopicMessageTimestampArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace ros2_benchmark_interfaces


namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
cdr_serialize(
  const ros2_benchmark_interfaces::srv::StartLoading_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: topic_message_timestamps
  {
    size_t size = ros_message.topic_message_timestamps.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      ros2_benchmark_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.topic_message_timestamps[i],
        cdr);
    }
  }
  // Member: played_message_count
  cdr << ros_message.played_message_count;
  // Member: success
  cdr << (ros_message.success ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros2_benchmark_interfaces::srv::StartLoading_Response & ros_message)
{
  // Member: topic_message_timestamps
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.topic_message_timestamps.resize(size);
    for (size_t i = 0; i < size; i++) {
      ros2_benchmark_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.topic_message_timestamps[i]);
    }
  }

  // Member: played_message_count
  cdr >> ros_message.played_message_count;

  // Member: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.success = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
get_serialized_size(
  const ros2_benchmark_interfaces::srv::StartLoading_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: topic_message_timestamps
  {
    size_t array_size = ros_message.topic_message_timestamps.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        ros2_benchmark_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.topic_message_timestamps[index], current_alignment);
    }
  }
  // Member: played_message_count
  {
    size_t item_size = sizeof(ros_message.played_message_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: success
  {
    size_t item_size = sizeof(ros_message.success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_benchmark_interfaces
max_serialized_size_StartLoading_Response(
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


  // Member: topic_message_timestamps
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ros2_benchmark_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_TopicMessageTimestampArray(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: played_message_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: success
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
    using DataType = ros2_benchmark_interfaces::srv::StartLoading_Response;
    is_plain =
      (
      offsetof(DataType, success) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _StartLoading_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ros2_benchmark_interfaces::srv::StartLoading_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _StartLoading_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ros2_benchmark_interfaces::srv::StartLoading_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _StartLoading_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ros2_benchmark_interfaces::srv::StartLoading_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _StartLoading_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_StartLoading_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _StartLoading_Response__callbacks = {
  "ros2_benchmark_interfaces::srv",
  "StartLoading_Response",
  _StartLoading_Response__cdr_serialize,
  _StartLoading_Response__cdr_deserialize,
  _StartLoading_Response__get_serialized_size,
  _StartLoading_Response__max_serialized_size
};

static rosidl_message_type_support_t _StartLoading_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_StartLoading_Response__callbacks,
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
get_message_type_support_handle<ros2_benchmark_interfaces::srv::StartLoading_Response>()
{
  return &ros2_benchmark_interfaces::srv::typesupport_fastrtps_cpp::_StartLoading_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2_benchmark_interfaces, srv, StartLoading_Response)() {
  return &ros2_benchmark_interfaces::srv::typesupport_fastrtps_cpp::_StartLoading_Response__handle;
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

static service_type_support_callbacks_t _StartLoading__callbacks = {
  "ros2_benchmark_interfaces::srv",
  "StartLoading",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2_benchmark_interfaces, srv, StartLoading_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2_benchmark_interfaces, srv, StartLoading_Response)(),
};

static rosidl_service_type_support_t _StartLoading__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_StartLoading__callbacks,
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
get_service_type_support_handle<ros2_benchmark_interfaces::srv::StartLoading>()
{
  return &ros2_benchmark_interfaces::srv::typesupport_fastrtps_cpp::_StartLoading__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2_benchmark_interfaces, srv, StartLoading)() {
  return &ros2_benchmark_interfaces::srv::typesupport_fastrtps_cpp::_StartLoading__handle;
}

#ifdef __cplusplus
}
#endif
