// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_benchmark_interfaces:msg/TopicMessageCount.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_COUNT__BUILDER_HPP_
#define ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_COUNT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_benchmark_interfaces/msg/detail/topic_message_count__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_benchmark_interfaces
{

namespace msg
{

namespace builder
{

class Init_TopicMessageCount_message_count
{
public:
  explicit Init_TopicMessageCount_message_count(::ros2_benchmark_interfaces::msg::TopicMessageCount & msg)
  : msg_(msg)
  {}
  ::ros2_benchmark_interfaces::msg::TopicMessageCount message_count(::ros2_benchmark_interfaces::msg::TopicMessageCount::_message_count_type arg)
  {
    msg_.message_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::msg::TopicMessageCount msg_;
};

class Init_TopicMessageCount_topic_name
{
public:
  Init_TopicMessageCount_topic_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TopicMessageCount_message_count topic_name(::ros2_benchmark_interfaces::msg::TopicMessageCount::_topic_name_type arg)
  {
    msg_.topic_name = std::move(arg);
    return Init_TopicMessageCount_message_count(msg_);
  }

private:
  ::ros2_benchmark_interfaces::msg::TopicMessageCount msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::msg::TopicMessageCount>()
{
  return ros2_benchmark_interfaces::msg::builder::Init_TopicMessageCount_topic_name();
}

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_COUNT__BUILDER_HPP_
