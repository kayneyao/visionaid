// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_benchmark_interfaces:srv/SetData.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__SET_DATA__STRUCT_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__SET_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2_benchmark_interfaces__srv__SetData_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros2_benchmark_interfaces__srv__SetData_Request __declspec(deprecated)
#endif

namespace ros2_benchmark_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetData_Request_
{
  using Type = SetData_Request_<ContainerAllocator>;

  explicit SetData_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->publish_tf_messages = false;
      this->publish_tf_static_messages = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->data_path = "";
      this->publish_tf_messages = false;
      this->publish_tf_static_messages = false;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data_path = "";
    }
  }

  explicit SetData_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->publish_tf_messages = false;
      this->publish_tf_static_messages = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->data_path = "";
      this->publish_tf_messages = false;
      this->publish_tf_static_messages = false;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data_path = "";
    }
  }

  // field types and members
  using _data_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _data_path_type data_path;
  using _publish_tf_messages_type =
    bool;
  _publish_tf_messages_type publish_tf_messages;
  using _publish_tf_static_messages_type =
    bool;
  _publish_tf_static_messages_type publish_tf_static_messages;

  // setters for named parameter idiom
  Type & set__data_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->data_path = _arg;
    return *this;
  }
  Type & set__publish_tf_messages(
    const bool & _arg)
  {
    this->publish_tf_messages = _arg;
    return *this;
  }
  Type & set__publish_tf_static_messages(
    const bool & _arg)
  {
    this->publish_tf_static_messages = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_benchmark_interfaces::srv::SetData_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_benchmark_interfaces::srv::SetData_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::SetData_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::SetData_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::SetData_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::SetData_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::SetData_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::SetData_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::SetData_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::SetData_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__SetData_Request
    std::shared_ptr<ros2_benchmark_interfaces::srv::SetData_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__SetData_Request
    std::shared_ptr<ros2_benchmark_interfaces::srv::SetData_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetData_Request_ & other) const
  {
    if (this->data_path != other.data_path) {
      return false;
    }
    if (this->publish_tf_messages != other.publish_tf_messages) {
      return false;
    }
    if (this->publish_tf_static_messages != other.publish_tf_static_messages) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetData_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetData_Request_

// alias to use template instance with default allocator
using SetData_Request =
  ros2_benchmark_interfaces::srv::SetData_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_benchmark_interfaces


#ifndef _WIN32
# define DEPRECATED__ros2_benchmark_interfaces__srv__SetData_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros2_benchmark_interfaces__srv__SetData_Response __declspec(deprecated)
#endif

namespace ros2_benchmark_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetData_Response_
{
  using Type = SetData_Response_<ContainerAllocator>;

  explicit SetData_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->success = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->success = false;
    }
  }

  explicit SetData_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
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

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_benchmark_interfaces::srv::SetData_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_benchmark_interfaces::srv::SetData_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::SetData_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::SetData_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::SetData_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::SetData_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::SetData_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::SetData_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::SetData_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::SetData_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__SetData_Response
    std::shared_ptr<ros2_benchmark_interfaces::srv::SetData_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__SetData_Response
    std::shared_ptr<ros2_benchmark_interfaces::srv::SetData_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetData_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetData_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetData_Response_

// alias to use template instance with default allocator
using SetData_Response =
  ros2_benchmark_interfaces::srv::SetData_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

namespace ros2_benchmark_interfaces
{

namespace srv
{

struct SetData
{
  using Request = ros2_benchmark_interfaces::srv::SetData_Request;
  using Response = ros2_benchmark_interfaces::srv::SetData_Response;
};

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__SET_DATA__STRUCT_HPP_
