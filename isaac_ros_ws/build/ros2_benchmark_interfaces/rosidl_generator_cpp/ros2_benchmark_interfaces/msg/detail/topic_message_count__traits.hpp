// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_benchmark_interfaces:msg/TopicMessageCount.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_COUNT__TRAITS_HPP_
#define ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_COUNT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_benchmark_interfaces/msg/detail/topic_message_count__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_benchmark_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TopicMessageCount & msg,
  std::ostream & out)
{
  out << "{";
  // member: topic_name
  {
    out << "topic_name: ";
    rosidl_generator_traits::value_to_yaml(msg.topic_name, out);
    out << ", ";
  }

  // member: message_count
  {
    out << "message_count: ";
    rosidl_generator_traits::value_to_yaml(msg.message_count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TopicMessageCount & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: topic_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "topic_name: ";
    rosidl_generator_traits::value_to_yaml(msg.topic_name, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TopicMessageCount & msg, bool use_flow_style = false)
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
  const ros2_benchmark_interfaces::msg::TopicMessageCount & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_benchmark_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_benchmark_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros2_benchmark_interfaces::msg::TopicMessageCount & msg)
{
  return ros2_benchmark_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_benchmark_interfaces::msg::TopicMessageCount>()
{
  return "ros2_benchmark_interfaces::msg::TopicMessageCount";
}

template<>
inline const char * name<ros2_benchmark_interfaces::msg::TopicMessageCount>()
{
  return "ros2_benchmark_interfaces/msg/TopicMessageCount";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::msg::TopicMessageCount>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::msg::TopicMessageCount>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_benchmark_interfaces::msg::TopicMessageCount>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_COUNT__TRAITS_HPP_
