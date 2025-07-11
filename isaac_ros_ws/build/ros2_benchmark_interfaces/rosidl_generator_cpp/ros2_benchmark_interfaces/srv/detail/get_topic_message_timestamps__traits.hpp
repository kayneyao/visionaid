// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_benchmark_interfaces:srv/GetTopicMessageTimestamps.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__GET_TOPIC_MESSAGE_TIMESTAMPS__TRAITS_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__GET_TOPIC_MESSAGE_TIMESTAMPS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_benchmark_interfaces/srv/detail/get_topic_message_timestamps__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_benchmark_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetTopicMessageTimestamps_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: start_time_offset_ns
  {
    out << "start_time_offset_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.start_time_offset_ns, out);
    out << ", ";
  }

  // member: end_time_offset_ns
  {
    out << "end_time_offset_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.end_time_offset_ns, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetTopicMessageTimestamps_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: start_time_offset_ns
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_time_offset_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.start_time_offset_ns, out);
    out << "\n";
  }

  // member: end_time_offset_ns
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_time_offset_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.end_time_offset_ns, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetTopicMessageTimestamps_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ros2_benchmark_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_benchmark_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_benchmark_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request & msg)
{
  return ros2_benchmark_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request>()
{
  return "ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request>()
{
  return "ros2_benchmark_interfaces/srv/GetTopicMessageTimestamps_Request";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'topic_message_timestamps'
#include "ros2_benchmark_interfaces/msg/detail/topic_message_timestamp_array__traits.hpp"

namespace ros2_benchmark_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetTopicMessageTimestamps_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: topic_message_timestamps
  {
    if (msg.topic_message_timestamps.size() == 0) {
      out << "topic_message_timestamps: []";
    } else {
      out << "topic_message_timestamps: [";
      size_t pending_items = msg.topic_message_timestamps.size();
      for (auto item : msg.topic_message_timestamps) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetTopicMessageTimestamps_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: topic_message_timestamps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.topic_message_timestamps.size() == 0) {
      out << "topic_message_timestamps: []\n";
    } else {
      out << "topic_message_timestamps:\n";
      for (auto item : msg.topic_message_timestamps) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetTopicMessageTimestamps_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ros2_benchmark_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_benchmark_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_benchmark_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response & msg)
{
  return ros2_benchmark_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response>()
{
  return "ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response>()
{
  return "ros2_benchmark_interfaces/srv/GetTopicMessageTimestamps_Response";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps>()
{
  return "ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps>()
{
  return "ros2_benchmark_interfaces/srv/GetTopicMessageTimestamps";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps>
  : std::integral_constant<
    bool,
    has_fixed_size<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request>::value &&
    has_fixed_size<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps>
  : std::integral_constant<
    bool,
    has_bounded_size<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request>::value &&
    has_bounded_size<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response>::value
  >
{
};

template<>
struct is_service<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps>
  : std::true_type
{
};

template<>
struct is_service_request<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__GET_TOPIC_MESSAGE_TIMESTAMPS__TRAITS_HPP_
