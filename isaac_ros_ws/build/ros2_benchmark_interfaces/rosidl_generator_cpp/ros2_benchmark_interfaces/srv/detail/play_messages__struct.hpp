// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_benchmark_interfaces:srv/PlayMessages.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__PLAY_MESSAGES__STRUCT_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__PLAY_MESSAGES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2_benchmark_interfaces__srv__PlayMessages_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros2_benchmark_interfaces__srv__PlayMessages_Request __declspec(deprecated)
#endif

namespace ros2_benchmark_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlayMessages_Request_
{
  using Type = PlayMessages_Request_<ContainerAllocator>;

  explicit PlayMessages_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->playback_mode = 1;
      this->message_count = 0ull;
      this->enforce_publisher_rate = false;
      this->revise_timestamps_as_message_ids = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->playback_mode = 0;
      this->message_count = 0ull;
      this->target_publisher_rate = 0.0;
      this->enforce_publisher_rate = false;
      this->revise_timestamps_as_message_ids = false;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_publisher_rate = 0.0;
    }
  }

  explicit PlayMessages_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->playback_mode = 1;
      this->message_count = 0ull;
      this->enforce_publisher_rate = false;
      this->revise_timestamps_as_message_ids = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->playback_mode = 0;
      this->message_count = 0ull;
      this->target_publisher_rate = 0.0;
      this->enforce_publisher_rate = false;
      this->revise_timestamps_as_message_ids = false;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_publisher_rate = 0.0;
    }
  }

  // field types and members
  using _playback_mode_type =
    uint8_t;
  _playback_mode_type playback_mode;
  using _message_count_type =
    uint64_t;
  _message_count_type message_count;
  using _target_publisher_rate_type =
    double;
  _target_publisher_rate_type target_publisher_rate;
  using _enforce_publisher_rate_type =
    bool;
  _enforce_publisher_rate_type enforce_publisher_rate;
  using _revise_timestamps_as_message_ids_type =
    bool;
  _revise_timestamps_as_message_ids_type revise_timestamps_as_message_ids;

  // setters for named parameter idiom
  Type & set__playback_mode(
    const uint8_t & _arg)
  {
    this->playback_mode = _arg;
    return *this;
  }
  Type & set__message_count(
    const uint64_t & _arg)
  {
    this->message_count = _arg;
    return *this;
  }
  Type & set__target_publisher_rate(
    const double & _arg)
  {
    this->target_publisher_rate = _arg;
    return *this;
  }
  Type & set__enforce_publisher_rate(
    const bool & _arg)
  {
    this->enforce_publisher_rate = _arg;
    return *this;
  }
  Type & set__revise_timestamps_as_message_ids(
    const bool & _arg)
  {
    this->revise_timestamps_as_message_ids = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t PLAYBACK_MODE_TIMELINE =
    0u;
  static constexpr uint8_t PLAYBACK_MODE_LOOPING =
    1u;
  static constexpr uint8_t PLAYBACK_MODE_SWEEPING =
    2u;

  // pointer types
  using RawPtr =
    ros2_benchmark_interfaces::srv::PlayMessages_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_benchmark_interfaces::srv::PlayMessages_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::PlayMessages_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::PlayMessages_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__PlayMessages_Request
    std::shared_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__PlayMessages_Request
    std::shared_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlayMessages_Request_ & other) const
  {
    if (this->playback_mode != other.playback_mode) {
      return false;
    }
    if (this->message_count != other.message_count) {
      return false;
    }
    if (this->target_publisher_rate != other.target_publisher_rate) {
      return false;
    }
    if (this->enforce_publisher_rate != other.enforce_publisher_rate) {
      return false;
    }
    if (this->revise_timestamps_as_message_ids != other.revise_timestamps_as_message_ids) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlayMessages_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlayMessages_Request_

// alias to use template instance with default allocator
using PlayMessages_Request =
  ros2_benchmark_interfaces::srv::PlayMessages_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PlayMessages_Request_<ContainerAllocator>::PLAYBACK_MODE_TIMELINE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PlayMessages_Request_<ContainerAllocator>::PLAYBACK_MODE_LOOPING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PlayMessages_Request_<ContainerAllocator>::PLAYBACK_MODE_SWEEPING;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace ros2_benchmark_interfaces


// Include directives for member types
// Member 'timestamps'
#include "ros2_benchmark_interfaces/msg/detail/timestamped_message_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2_benchmark_interfaces__srv__PlayMessages_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros2_benchmark_interfaces__srv__PlayMessages_Response __declspec(deprecated)
#endif

namespace ros2_benchmark_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlayMessages_Response_
{
  using Type = PlayMessages_Response_<ContainerAllocator>;

  explicit PlayMessages_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamps(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->success = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->success = false;
    }
  }

  explicit PlayMessages_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamps(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->success = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _timestamps_type =
    ros2_benchmark_interfaces::msg::TimestampedMessageArray_<ContainerAllocator>;
  _timestamps_type timestamps;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__timestamps(
    const ros2_benchmark_interfaces::msg::TimestampedMessageArray_<ContainerAllocator> & _arg)
  {
    this->timestamps = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_benchmark_interfaces::srv::PlayMessages_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_benchmark_interfaces::srv::PlayMessages_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::PlayMessages_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::PlayMessages_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__PlayMessages_Response
    std::shared_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__PlayMessages_Response
    std::shared_ptr<ros2_benchmark_interfaces::srv::PlayMessages_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlayMessages_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->timestamps != other.timestamps) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlayMessages_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlayMessages_Response_

// alias to use template instance with default allocator
using PlayMessages_Response =
  ros2_benchmark_interfaces::srv::PlayMessages_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

namespace ros2_benchmark_interfaces
{

namespace srv
{

struct PlayMessages
{
  using Request = ros2_benchmark_interfaces::srv::PlayMessages_Request;
  using Response = ros2_benchmark_interfaces::srv::PlayMessages_Response;
};

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__PLAY_MESSAGES__STRUCT_HPP_
