// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_benchmark_interfaces:srv/StartRecording.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_RECORDING__STRUCT_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_RECORDING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'topic_message_timestamps'
#include "ros2_benchmark_interfaces/msg/detail/topic_message_timestamp_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2_benchmark_interfaces__srv__StartRecording_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros2_benchmark_interfaces__srv__StartRecording_Request __declspec(deprecated)
#endif

namespace ros2_benchmark_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartRecording_Request_
{
  using Type = StartRecording_Request_<ContainerAllocator>;

  explicit StartRecording_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->record_data_timeline = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->buffer_length = 0ull;
      this->timeout = 0ll;
      this->record_data_timeline = false;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->buffer_length = 0ull;
      this->timeout = 0ll;
    }
  }

  explicit StartRecording_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->record_data_timeline = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->buffer_length = 0ull;
      this->timeout = 0ll;
      this->record_data_timeline = false;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->buffer_length = 0ull;
      this->timeout = 0ll;
    }
  }

  // field types and members
  using _buffer_length_type =
    uint64_t;
  _buffer_length_type buffer_length;
  using _timeout_type =
    int64_t;
  _timeout_type timeout;
  using _topic_message_timestamps_type =
    std::vector<ros2_benchmark_interfaces::msg::TopicMessageTimestampArray_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2_benchmark_interfaces::msg::TopicMessageTimestampArray_<ContainerAllocator>>>;
  _topic_message_timestamps_type topic_message_timestamps;
  using _record_data_timeline_type =
    bool;
  _record_data_timeline_type record_data_timeline;

  // setters for named parameter idiom
  Type & set__buffer_length(
    const uint64_t & _arg)
  {
    this->buffer_length = _arg;
    return *this;
  }
  Type & set__timeout(
    const int64_t & _arg)
  {
    this->timeout = _arg;
    return *this;
  }
  Type & set__topic_message_timestamps(
    const std::vector<ros2_benchmark_interfaces::msg::TopicMessageTimestampArray_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2_benchmark_interfaces::msg::TopicMessageTimestampArray_<ContainerAllocator>>> & _arg)
  {
    this->topic_message_timestamps = _arg;
    return *this;
  }
  Type & set__record_data_timeline(
    const bool & _arg)
  {
    this->record_data_timeline = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_benchmark_interfaces::srv::StartRecording_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_benchmark_interfaces::srv::StartRecording_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartRecording_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartRecording_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StartRecording_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StartRecording_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StartRecording_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StartRecording_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StartRecording_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StartRecording_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StartRecording_Request
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartRecording_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StartRecording_Request
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartRecording_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartRecording_Request_ & other) const
  {
    if (this->buffer_length != other.buffer_length) {
      return false;
    }
    if (this->timeout != other.timeout) {
      return false;
    }
    if (this->topic_message_timestamps != other.topic_message_timestamps) {
      return false;
    }
    if (this->record_data_timeline != other.record_data_timeline) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartRecording_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartRecording_Request_

// alias to use template instance with default allocator
using StartRecording_Request =
  ros2_benchmark_interfaces::srv::StartRecording_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_benchmark_interfaces


// Include directives for member types
// Member 'recorded_topic_message_counts'
#include "ros2_benchmark_interfaces/msg/detail/topic_message_count__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2_benchmark_interfaces__srv__StartRecording_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros2_benchmark_interfaces__srv__StartRecording_Response __declspec(deprecated)
#endif

namespace ros2_benchmark_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartRecording_Response_
{
  using Type = StartRecording_Response_<ContainerAllocator>;

  explicit StartRecording_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->success = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->success = false;
      this->recorded_message_count = 0ull;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->recorded_message_count = 0ull;
    }
  }

  explicit StartRecording_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->success = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->success = false;
      this->recorded_message_count = 0ull;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->recorded_message_count = 0ull;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _recorded_message_count_type =
    uint64_t;
  _recorded_message_count_type recorded_message_count;
  using _recorded_topic_message_counts_type =
    std::vector<ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator>>>;
  _recorded_topic_message_counts_type recorded_topic_message_counts;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__recorded_message_count(
    const uint64_t & _arg)
  {
    this->recorded_message_count = _arg;
    return *this;
  }
  Type & set__recorded_topic_message_counts(
    const std::vector<ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2_benchmark_interfaces::msg::TopicMessageCount_<ContainerAllocator>>> & _arg)
  {
    this->recorded_topic_message_counts = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_benchmark_interfaces::srv::StartRecording_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_benchmark_interfaces::srv::StartRecording_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartRecording_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartRecording_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StartRecording_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StartRecording_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StartRecording_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StartRecording_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StartRecording_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StartRecording_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StartRecording_Response
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartRecording_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StartRecording_Response
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartRecording_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartRecording_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->recorded_message_count != other.recorded_message_count) {
      return false;
    }
    if (this->recorded_topic_message_counts != other.recorded_topic_message_counts) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartRecording_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartRecording_Response_

// alias to use template instance with default allocator
using StartRecording_Response =
  ros2_benchmark_interfaces::srv::StartRecording_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

namespace ros2_benchmark_interfaces
{

namespace srv
{

struct StartRecording
{
  using Request = ros2_benchmark_interfaces::srv::StartRecording_Request;
  using Response = ros2_benchmark_interfaces::srv::StartRecording_Response;
};

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_RECORDING__STRUCT_HPP_
