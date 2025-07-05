// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_benchmark_interfaces:srv/StartRecording.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_RECORDING__TRAITS_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_RECORDING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_benchmark_interfaces/srv/detail/start_recording__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'topic_message_timestamps'
#include "ros2_benchmark_interfaces/msg/detail/topic_message_timestamp_array__traits.hpp"

namespace ros2_benchmark_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const StartRecording_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: buffer_length
  {
    out << "buffer_length: ";
    rosidl_generator_traits::value_to_yaml(msg.buffer_length, out);
    out << ", ";
  }

  // member: timeout
  {
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << ", ";
  }

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

  // member: record_data_timeline
  {
    out << "record_data_timeline: ";
    rosidl_generator_traits::value_to_yaml(msg.record_data_timeline, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StartRecording_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: buffer_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "buffer_length: ";
    rosidl_generator_traits::value_to_yaml(msg.buffer_length, out);
    out << "\n";
  }

  // member: timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << "\n";
  }

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

  // member: record_data_timeline
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "record_data_timeline: ";
    rosidl_generator_traits::value_to_yaml(msg.record_data_timeline, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StartRecording_Request & msg, bool use_flow_style = false)
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
  const ros2_benchmark_interfaces::srv::StartRecording_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_benchmark_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_benchmark_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_benchmark_interfaces::srv::StartRecording_Request & msg)
{
  return ros2_benchmark_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::StartRecording_Request>()
{
  return "ros2_benchmark_interfaces::srv::StartRecording_Request";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::StartRecording_Request>()
{
  return "ros2_benchmark_interfaces/srv/StartRecording_Request";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::StartRecording_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::StartRecording_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_benchmark_interfaces::srv::StartRecording_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'recorded_topic_message_counts'
#include "ros2_benchmark_interfaces/msg/detail/topic_message_count__traits.hpp"

namespace ros2_benchmark_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const StartRecording_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: recorded_message_count
  {
    out << "recorded_message_count: ";
    rosidl_generator_traits::value_to_yaml(msg.recorded_message_count, out);
    out << ", ";
  }

  // member: recorded_topic_message_counts
  {
    if (msg.recorded_topic_message_counts.size() == 0) {
      out << "recorded_topic_message_counts: []";
    } else {
      out << "recorded_topic_message_counts: [";
      size_t pending_items = msg.recorded_topic_message_counts.size();
      for (auto item : msg.recorded_topic_message_counts) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StartRecording_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: recorded_message_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "recorded_message_count: ";
    rosidl_generator_traits::value_to_yaml(msg.recorded_message_count, out);
    out << "\n";
  }

  // member: recorded_topic_message_counts
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.recorded_topic_message_counts.size() == 0) {
      out << "recorded_topic_message_counts: []\n";
    } else {
      out << "recorded_topic_message_counts:\n";
      for (auto item : msg.recorded_topic_message_counts) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StartRecording_Response & msg, bool use_flow_style = false)
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
  const ros2_benchmark_interfaces::srv::StartRecording_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_benchmark_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_benchmark_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_benchmark_interfaces::srv::StartRecording_Response & msg)
{
  return ros2_benchmark_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::StartRecording_Response>()
{
  return "ros2_benchmark_interfaces::srv::StartRecording_Response";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::StartRecording_Response>()
{
  return "ros2_benchmark_interfaces/srv/StartRecording_Response";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::StartRecording_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::StartRecording_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_benchmark_interfaces::srv::StartRecording_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::StartRecording>()
{
  return "ros2_benchmark_interfaces::srv::StartRecording";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::StartRecording>()
{
  return "ros2_benchmark_interfaces/srv/StartRecording";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::StartRecording>
  : std::integral_constant<
    bool,
    has_fixed_size<ros2_benchmark_interfaces::srv::StartRecording_Request>::value &&
    has_fixed_size<ros2_benchmark_interfaces::srv::StartRecording_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::StartRecording>
  : std::integral_constant<
    bool,
    has_bounded_size<ros2_benchmark_interfaces::srv::StartRecording_Request>::value &&
    has_bounded_size<ros2_benchmark_interfaces::srv::StartRecording_Response>::value
  >
{
};

template<>
struct is_service<ros2_benchmark_interfaces::srv::StartRecording>
  : std::true_type
{
};

template<>
struct is_service_request<ros2_benchmark_interfaces::srv::StartRecording_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros2_benchmark_interfaces::srv::StartRecording_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_RECORDING__TRAITS_HPP_
