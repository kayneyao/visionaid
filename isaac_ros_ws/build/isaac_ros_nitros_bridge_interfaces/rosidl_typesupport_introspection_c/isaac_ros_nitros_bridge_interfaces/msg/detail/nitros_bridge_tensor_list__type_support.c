// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from isaac_ros_nitros_bridge_interfaces:msg/NitrosBridgeTensorList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "isaac_ros_nitros_bridge_interfaces/msg/detail/nitros_bridge_tensor_list__rosidl_typesupport_introspection_c.h"
#include "isaac_ros_nitros_bridge_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "isaac_ros_nitros_bridge_interfaces/msg/detail/nitros_bridge_tensor_list__functions.h"
#include "isaac_ros_nitros_bridge_interfaces/msg/detail/nitros_bridge_tensor_list__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `tensors`
#include "isaac_ros_tensor_list_interfaces/msg/tensor.h"
// Member `tensors`
#include "isaac_ros_tensor_list_interfaces/msg/detail/tensor__rosidl_typesupport_introspection_c.h"
// Member `cuda_event_handle`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `uid`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__NitrosBridgeTensorList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__init(message_memory);
}

void isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__NitrosBridgeTensorList_fini_function(void * message_memory)
{
  isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__fini(message_memory);
}

size_t isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__size_function__NitrosBridgeTensorList__tensors(
  const void * untyped_member)
{
  const isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence * member =
    (const isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence *)(untyped_member);
  return member->size;
}

const void * isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__get_const_function__NitrosBridgeTensorList__tensors(
  const void * untyped_member, size_t index)
{
  const isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence * member =
    (const isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence *)(untyped_member);
  return &member->data[index];
}

void * isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__get_function__NitrosBridgeTensorList__tensors(
  void * untyped_member, size_t index)
{
  isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence * member =
    (isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence *)(untyped_member);
  return &member->data[index];
}

void isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__fetch_function__NitrosBridgeTensorList__tensors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const isaac_ros_tensor_list_interfaces__msg__Tensor * item =
    ((const isaac_ros_tensor_list_interfaces__msg__Tensor *)
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__get_const_function__NitrosBridgeTensorList__tensors(untyped_member, index));
  isaac_ros_tensor_list_interfaces__msg__Tensor * value =
    (isaac_ros_tensor_list_interfaces__msg__Tensor *)(untyped_value);
  *value = *item;
}

void isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__assign_function__NitrosBridgeTensorList__tensors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  isaac_ros_tensor_list_interfaces__msg__Tensor * item =
    ((isaac_ros_tensor_list_interfaces__msg__Tensor *)
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__get_function__NitrosBridgeTensorList__tensors(untyped_member, index));
  const isaac_ros_tensor_list_interfaces__msg__Tensor * value =
    (const isaac_ros_tensor_list_interfaces__msg__Tensor *)(untyped_value);
  *item = *value;
}

bool isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__resize_function__NitrosBridgeTensorList__tensors(
  void * untyped_member, size_t size)
{
  isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence * member =
    (isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence *)(untyped_member);
  isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence__fini(member);
  return isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence__init(member, size);
}

size_t isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__size_function__NitrosBridgeTensorList__cuda_event_handle(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__get_const_function__NitrosBridgeTensorList__cuda_event_handle(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__get_function__NitrosBridgeTensorList__cuda_event_handle(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__fetch_function__NitrosBridgeTensorList__cuda_event_handle(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__get_const_function__NitrosBridgeTensorList__cuda_event_handle(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__assign_function__NitrosBridgeTensorList__cuda_event_handle(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__get_function__NitrosBridgeTensorList__cuda_event_handle(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__resize_function__NitrosBridgeTensorList__cuda_event_handle(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__NitrosBridgeTensorList_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tensors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList, tensors),  // bytes offset in struct
    NULL,  // default value
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__size_function__NitrosBridgeTensorList__tensors,  // size() function pointer
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__get_const_function__NitrosBridgeTensorList__tensors,  // get_const(index) function pointer
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__get_function__NitrosBridgeTensorList__tensors,  // get(index) function pointer
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__fetch_function__NitrosBridgeTensorList__tensors,  // fetch(index, &value) function pointer
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__assign_function__NitrosBridgeTensorList__tensors,  // assign(index, value) function pointer
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__resize_function__NitrosBridgeTensorList__tensors  // resize(index) function pointer
  },
  {
    "pid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList, pid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList, fd),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cuda_event_handle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList, cuda_event_handle),  // bytes offset in struct
    NULL,  // default value
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__size_function__NitrosBridgeTensorList__cuda_event_handle,  // size() function pointer
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__get_const_function__NitrosBridgeTensorList__cuda_event_handle,  // get_const(index) function pointer
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__get_function__NitrosBridgeTensorList__cuda_event_handle,  // get(index) function pointer
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__fetch_function__NitrosBridgeTensorList__cuda_event_handle,  // fetch(index, &value) function pointer
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__assign_function__NitrosBridgeTensorList__cuda_event_handle,  // assign(index, value) function pointer
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__resize_function__NitrosBridgeTensorList__cuda_event_handle  // resize(index) function pointer
  },
  {
    "uid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList, uid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "device_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList, device_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__NitrosBridgeTensorList_message_members = {
  "isaac_ros_nitros_bridge_interfaces__msg",  // message namespace
  "NitrosBridgeTensorList",  // message name
  7,  // number of fields
  sizeof(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList),
  isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__NitrosBridgeTensorList_message_member_array,  // message members
  isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__NitrosBridgeTensorList_init_function,  // function to initialize message memory (memory has to be allocated)
  isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__NitrosBridgeTensorList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__NitrosBridgeTensorList_message_type_support_handle = {
  0,
  &isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__NitrosBridgeTensorList_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_isaac_ros_nitros_bridge_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, isaac_ros_nitros_bridge_interfaces, msg, NitrosBridgeTensorList)() {
  isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__NitrosBridgeTensorList_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__NitrosBridgeTensorList_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, isaac_ros_tensor_list_interfaces, msg, Tensor)();
  if (!isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__NitrosBridgeTensorList_message_type_support_handle.typesupport_identifier) {
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__NitrosBridgeTensorList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__rosidl_typesupport_introspection_c__NitrosBridgeTensorList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
