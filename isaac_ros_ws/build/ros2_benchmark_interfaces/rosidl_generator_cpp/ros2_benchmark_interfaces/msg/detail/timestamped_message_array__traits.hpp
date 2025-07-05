// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_benchmark_interfaces:msg/TimestampedMessageArray.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TIMESTAMPED_MESSAGE_ARRAY__TRAITS_HPP_
#define ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TIMESTAMPED_MESSAGE_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_benchmark_interfaces/msg/detail/timestamped_message_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_benchmark_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TimestampedMessageArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: keys
  {
    if (msg.keys.size() == 0) {
      out << "keys: []";
    } else {
      out << "keys: [";
      size_t pending_items = msg.keys.size();
      for (auto item : msg.keys) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: timestamps_ns
  {
    if (msg.timestamps_ns.size() == 0) {
      out << "timestamps_ns: []";
    } else {
      out << "timestamps_ns: [";
      size_t pending_items = msg.timestamps_ns.size();
      for (auto item : msg.timestamps_ns) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const TimestampedMessageArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: keys
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.keys.size() == 0) {
      out << "keys: []\n";
    } else {
      out << "keys:\n";
      for (auto item : msg.keys) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: timestamps_ns
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.timestamps_ns.size() == 0) {
      out << "timestamps_ns: []\n";
    } else {
      out << "timestamps_ns:\n";
      for (auto item : msg.timestamps_ns) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TimestampedMessageArray & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ros2_benchmark_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ros2_benchmark_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_benchmark_interfaces::msg::TimestampedMessageArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_benchmark_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_benchmark_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros2_benchmark_interfaces::msg::TimestampedMessageArray & msg)
{
  return ros2_benchmark_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_benchmark_interfaces::msg::TimestampedMessageArray>()
{
  return "ros2_benchmark_interfaces::msg::TimestampedMessageArray";
}

template<>
inline const char * name<ros2_benchmark_interfaces::msg::TimestampedMessageArray>()
{
  return "ros2_benchmark_interfaces/msg/TimestampedMessageArray";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::msg::TimestampedMessageArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::msg::TimestampedMessageArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_benchmark_interfaces::msg::TimestampedMessageArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TIMESTAMPED_MESSAGE_ARRAY__TRAITS_HPP_
