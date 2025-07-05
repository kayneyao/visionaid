// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from isaac_ros_nitros_bridge_interfaces:msg/NitrosBridgeTensorList.idl
// generated code does not contain a copyright notice
#include "isaac_ros_nitros_bridge_interfaces/msg/detail/nitros_bridge_tensor_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `tensors`
#include "isaac_ros_tensor_list_interfaces/msg/detail/tensor__functions.h"
// Member `cuda_event_handle`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `uid`
#include "rosidl_runtime_c/string_functions.h"

bool
isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__init(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__fini(msg);
    return false;
  }
  // tensors
  if (!isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence__init(&msg->tensors, 0)) {
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__fini(msg);
    return false;
  }
  // pid
  // fd
  // cuda_event_handle
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->cuda_event_handle, 0)) {
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__fini(msg);
    return false;
  }
  // uid
  if (!rosidl_runtime_c__String__init(&msg->uid)) {
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__fini(msg);
    return false;
  }
  // device_id
  return true;
}

void
isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__fini(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // tensors
  isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence__fini(&msg->tensors);
  // pid
  // fd
  // cuda_event_handle
  rosidl_runtime_c__uint8__Sequence__fini(&msg->cuda_event_handle);
  // uid
  rosidl_runtime_c__String__fini(&msg->uid);
  // device_id
}

bool
isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__are_equal(const isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList * lhs, const isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // tensors
  if (!isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence__are_equal(
      &(lhs->tensors), &(rhs->tensors)))
  {
    return false;
  }
  // pid
  if (lhs->pid != rhs->pid) {
    return false;
  }
  // fd
  if (lhs->fd != rhs->fd) {
    return false;
  }
  // cuda_event_handle
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->cuda_event_handle), &(rhs->cuda_event_handle)))
  {
    return false;
  }
  // uid
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->uid), &(rhs->uid)))
  {
    return false;
  }
  // device_id
  if (lhs->device_id != rhs->device_id) {
    return false;
  }
  return true;
}

bool
isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__copy(
  const isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList * input,
  isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // tensors
  if (!isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence__copy(
      &(input->tensors), &(output->tensors)))
  {
    return false;
  }
  // pid
  output->pid = input->pid;
  // fd
  output->fd = input->fd;
  // cuda_event_handle
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->cuda_event_handle), &(output->cuda_event_handle)))
  {
    return false;
  }
  // uid
  if (!rosidl_runtime_c__String__copy(
      &(input->uid), &(output->uid)))
  {
    return false;
  }
  // device_id
  output->device_id = input->device_id;
  return true;
}

isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList *
isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList * msg = (isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList *)allocator.allocate(sizeof(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList));
  bool success = isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__destroy(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence__init(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList * data = NULL;

  if (size) {
    data = (isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList *)allocator.zero_allocate(size, sizeof(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence__fini(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence *
isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence * array = (isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence *)allocator.allocate(sizeof(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence__destroy(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence__are_equal(const isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence * lhs, const isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence__copy(
  const isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence * input,
  isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList * data =
      (isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
