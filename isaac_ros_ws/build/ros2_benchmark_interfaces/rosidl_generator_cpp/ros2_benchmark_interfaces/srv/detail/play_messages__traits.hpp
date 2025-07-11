// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_benchmark_interfaces:srv/PlayMessages.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__PLAY_MESSAGES__TRAITS_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__PLAY_MESSAGES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_benchmark_interfaces/srv/detail/play_messages__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_benchmark_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlayMessages_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: playback_mode
  {
    out << "playback_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.playback_mode, out);
    out << ", ";
  }

  // member: message_count
  {
    out << "message_count: ";
    rosidl_generator_traits::value_to_yaml(msg.message_count, out);
    out << ", ";
  }

  // member: target_publisher_rate
  {
    out << "target_publisher_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.target_publisher_rate, out);
    out << ", ";
  }

  // member: enforce_publisher_rate
  {
    out << "enforce_publisher_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.enforce_publisher_rate, out);
    out << ", ";
  }

  // member: revise_timestamps_as_message_ids
  {
    out << "revise_timestamps_as_message_ids: ";
    rosidl_generator_traits::value_to_yaml(msg.revise_timestamps_as_message_ids, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlayMessages_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: playback_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "playback_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.playback_mode, out);
    out << "\n";
  }

  // member: message_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message_count: ";
    rosidl_generator_traits::value_to_yaml(msg.message_count, out);
    out << "\n";
  }

  // member: target_publisher_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_publisher_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.target_publisher_rate, out);
    out << "\n";
  }

  // member: enforce_publisher_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enforce_publisher_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.enforce_publisher_rate, out);
    out << "\n";
  }

  // member: revise_timestamps_as_message_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "revise_timestamps_as_message_ids: ";
    rosidl_generator_traits::value_to_yaml(msg.revise_timestamps_as_message_ids, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlayMessages_Request & msg, bool use_flow_style = false)
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
  const ros2_benchmark_interfaces::srv::PlayMessages_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_benchmark_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_benchmark_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_benchmark_interfaces::srv::PlayMessages_Request & msg)
{
  return ros2_benchmark_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::PlayMessages_Request>()
{
  return "ros2_benchmark_interfaces::srv::PlayMessages_Request";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::PlayMessages_Request>()
{
  return "ros2_benchmark_interfaces/srv/PlayMessages_Request";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::PlayMessages_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::PlayMessages_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2_benchmark_interfaces::srv::PlayMessages_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'timestamps'
#include "ros2_benchmark_interfaces/msg/detail/timestamped_message_array__traits.hpp"

namespace ros2_benchmark_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlayMessages_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: timestamps
  {
    out << "timestamps: ";
    to_flow_style_yaml(msg.timestamps, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlayMessages_Response & msg,
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

  // member: timestamps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamps:\n";
    to_block_style_yaml(msg.timestamps, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlayMessages_Response & msg, bool use_flow_style = false)
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
  const ros2_benchmark_interfaces::srv::PlayMessages_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_benchmark_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_benchmark_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_benchmark_interfaces::srv::PlayMessages_Response & msg)
{
  return ros2_benchmark_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::PlayMessages_Response>()
{
  return "ros2_benchmark_interfaces::srv::PlayMessages_Response";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::PlayMessages_Response>()
{
  return "ros2_benchmark_interfaces/srv/PlayMessages_Response";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::PlayMessages_Response>
  : std::integral_constant<bool, has_fixed_size<ros2_benchmark_interfaces::msg::TimestampedMessageArray>::value> {};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::PlayMessages_Response>
  : std::integral_constant<bool, has_bounded_size<ros2_benchmark_interfaces::msg::TimestampedMessageArray>::value> {};

template<>
struct is_message<ros2_benchmark_interfaces::srv::PlayMessages_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::PlayMessages>()
{
  return "ros2_benchmark_interfaces::srv::PlayMessages";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::PlayMessages>()
{
  return "ros2_benchmark_interfaces/srv/PlayMessages";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::PlayMessages>
  : std::integral_constant<
    bool,
    has_fixed_size<ros2_benchmark_interfaces::srv::PlayMessages_Request>::value &&
    has_fixed_size<ros2_benchmark_interfaces::srv::PlayMessages_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::PlayMessages>
  : std::integral_constant<
    bool,
    has_bounded_size<ros2_benchmark_interfaces::srv::PlayMessages_Request>::value &&
    has_bounded_size<ros2_benchmark_interfaces::srv::PlayMessages_Response>::value
  >
{
};

template<>
struct is_service<ros2_benchmark_interfaces::srv::PlayMessages>
  : std::true_type
{
};

template<>
struct is_service_request<ros2_benchmark_interfaces::srv::PlayMessages_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros2_benchmark_interfaces::srv::PlayMessages_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__PLAY_MESSAGES__TRAITS_HPP_
