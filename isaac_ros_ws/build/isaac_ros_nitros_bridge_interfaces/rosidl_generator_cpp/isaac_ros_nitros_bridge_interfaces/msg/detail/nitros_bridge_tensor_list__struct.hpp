// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from isaac_ros_nitros_bridge_interfaces:msg/NitrosBridgeTensorList.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS_NITROS_BRIDGE_INTERFACES__MSG__DETAIL__NITROS_BRIDGE_TENSOR_LIST__STRUCT_HPP_
#define ISAAC_ROS_NITROS_BRIDGE_INTERFACES__MSG__DETAIL__NITROS_BRIDGE_TENSOR_LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'tensors'
#include "isaac_ros_tensor_list_interfaces/msg/detail/tensor__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList __attribute__((deprecated))
#else
# define DEPRECATED__isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList __declspec(deprecated)
#endif

namespace isaac_ros_nitros_bridge_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NitrosBridgeTensorList_
{
  using Type = NitrosBridgeTensorList_<ContainerAllocator>;

  explicit NitrosBridgeTensorList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pid = 0l;
      this->fd = 0l;
      this->uid = "";
      this->device_id = 0ul;
    }
  }

  explicit NitrosBridgeTensorList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    uid(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pid = 0l;
      this->fd = 0l;
      this->uid = "";
      this->device_id = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _tensors_type =
    std::vector<isaac_ros_tensor_list_interfaces::msg::Tensor_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<isaac_ros_tensor_list_interfaces::msg::Tensor_<ContainerAllocator>>>;
  _tensors_type tensors;
  using _pid_type =
    int32_t;
  _pid_type pid;
  using _fd_type =
    int32_t;
  _fd_type fd;
  using _cuda_event_handle_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _cuda_event_handle_type cuda_event_handle;
  using _uid_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _uid_type uid;
  using _device_id_type =
    uint32_t;
  _device_id_type device_id;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__tensors(
    const std::vector<isaac_ros_tensor_list_interfaces::msg::Tensor_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<isaac_ros_tensor_list_interfaces::msg::Tensor_<ContainerAllocator>>> & _arg)
  {
    this->tensors = _arg;
    return *this;
  }
  Type & set__pid(
    const int32_t & _arg)
  {
    this->pid = _arg;
    return *this;
  }
  Type & set__fd(
    const int32_t & _arg)
  {
    this->fd = _arg;
    return *this;
  }
  Type & set__cuda_event_handle(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->cuda_event_handle = _arg;
    return *this;
  }
  Type & set__uid(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->uid = _arg;
    return *this;
  }
  Type & set__device_id(
    const uint32_t & _arg)
  {
    this->device_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList_<ContainerAllocator> *;
  using ConstRawPtr =
    const isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList
    std::shared_ptr<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList
    std::shared_ptr<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NitrosBridgeTensorList_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->tensors != other.tensors) {
      return false;
    }
    if (this->pid != other.pid) {
      return false;
    }
    if (this->fd != other.fd) {
      return false;
    }
    if (this->cuda_event_handle != other.cuda_event_handle) {
      return false;
    }
    if (this->uid != other.uid) {
      return false;
    }
    if (this->device_id != other.device_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const NitrosBridgeTensorList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NitrosBridgeTensorList_

// alias to use template instance with default allocator
using NitrosBridgeTensorList =
  isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace isaac_ros_nitros_bridge_interfaces

#endif  // ISAAC_ROS_NITROS_BRIDGE_INTERFACES__MSG__DETAIL__NITROS_BRIDGE_TENSOR_LIST__STRUCT_HPP_
