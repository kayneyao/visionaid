// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_benchmark_interfaces:msg/TopicMessageCount.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_COUNT__STRUCT_HPP_
#define ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_COUNT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2_benchmark_interfaces__msg__TopicMessageCount __attribute__((deprecated))
#else
# define DEPRECATED__ros2_benchmark_interfaces__msg__TopicMessageCount __declspec(deprecated)
#endif

namespace ros2_benchmark_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TopicMessageCount_
{
  using Type = TopicMessageCount_<ContainerAllocator>;

  explicit TopicMessageCount_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->topic_name = "";
      this->message_count = 0ull;
    }
  }

  explicit TopicMessageCount_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : topic_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->topic_name = "";
      this->message_count = 0ull;
    }
  }

  // field types and members
  using _topic_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _topic_name_type topic_name;
  using _message_count_type =
    uint64_t;
  _message_count_type message_count;

  // setters for named parameter idiom
  Type & set__topic_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->topic_name = _arg;
    return *this;
  }
  Type & set__message_count(
    const uint64_t & _arg)
  {
    this->message_count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_benchmark_interfaces__msg__TopicMessageCount
    std::shared_ptr<ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_benchmark_interfaces__msg__TopicMessageCount
    std::shared_ptr<ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TopicMessageCount_ & other) const
  {
    if (this->topic_name != other.topic_name) {
      return false;
    }
    if (this->message_count != other.message_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const TopicMessageCount_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TopicMessageCount_

// alias to use template instance with default allocator
using TopicMessageCount =
  ros2_benchmark_interfaces::msg::TopicMessageCount_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_COUNT__STRUCT_HPP_
