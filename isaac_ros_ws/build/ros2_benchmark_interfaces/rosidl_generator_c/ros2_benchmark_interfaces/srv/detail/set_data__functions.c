// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_benchmark_interfaces:srv/SetData.idl
// generated code does not contain a copyright notice
#include "ros2_benchmark_interfaces/srv/detail/set_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `data_path`
#include "rosidl_runtime_c/string_functions.h"

bool
ros2_benchmark_interfaces__srv__SetData_Request__init(ros2_benchmark_interfaces__srv__SetData_Request * msg)
{
  if (!msg) {
    return false;
  }
  // data_path
  if (!rosidl_runtime_c__String__init(&msg->data_path)) {
    ros2_benchmark_interfaces__srv__SetData_Request__fini(msg);
    return false;
  }
  // publish_tf_messages
  msg->publish_tf_messages = false;
  // publish_tf_static_messages
  msg->publish_tf_static_messages = false;
  return true;
}

void
ros2_benchmark_interfaces__srv__SetData_Request__fini(ros2_benchmark_interfaces__srv__SetData_Request * msg)
{
  if (!msg) {
    return;
  }
  // data_path
  rosidl_runtime_c__String__fini(&msg->data_path);
  // publish_tf_messages
  // publish_tf_static_messages
}

bool
ros2_benchmark_interfaces__srv__SetData_Request__are_equal(const ros2_benchmark_interfaces__srv__SetData_Request * lhs, const ros2_benchmark_interfaces__srv__SetData_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->data_path), &(rhs->data_path)))
  {
    return false;
  }
  // publish_tf_messages
  if (lhs->publish_tf_messages != rhs->publish_tf_messages) {
    return false;
  }
  // publish_tf_static_messages
  if (lhs->publish_tf_static_messages != rhs->publish_tf_static_messages) {
    return false;
  }
  return true;
}

bool
ros2_benchmark_interfaces__srv__SetData_Request__copy(
  const ros2_benchmark_interfaces__srv__SetData_Request * input,
  ros2_benchmark_interfaces__srv__SetData_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // data_path
  if (!rosidl_runtime_c__String__copy(
      &(input->data_path), &(output->data_path)))
  {
    return false;
  }
  // publish_tf_messages
  output->publish_tf_messages = input->publish_tf_messages;
  // publish_tf_static_messages
  output->publish_tf_static_messages = input->publish_tf_static_messages;
  return true;
}

ros2_benchmark_interfaces__srv__SetData_Request *
ros2_benchmark_interfaces__srv__SetData_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__srv__SetData_Request * msg = (ros2_benchmark_interfaces__srv__SetData_Request *)allocator.allocate(sizeof(ros2_benchmark_interfaces__srv__SetData_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_benchmark_interfaces__srv__SetData_Request));
  bool success = ros2_benchmark_interfaces__srv__SetData_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_benchmark_interfaces__srv__SetData_Request__destroy(ros2_benchmark_interfaces__srv__SetData_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_benchmark_interfaces__srv__SetData_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_benchmark_interfaces__srv__SetData_Request__Sequence__init(ros2_benchmark_interfaces__srv__SetData_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__srv__SetData_Request * data = NULL;

  if (size) {
    data = (ros2_benchmark_interfaces__srv__SetData_Request *)allocator.zero_allocate(size, sizeof(ros2_benchmark_interfaces__srv__SetData_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_benchmark_interfaces__srv__SetData_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_benchmark_interfaces__srv__SetData_Request__fini(&data[i - 1]);
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
ros2_benchmark_interfaces__srv__SetData_Request__Sequence__fini(ros2_benchmark_interfaces__srv__SetData_Request__Sequence * array)
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
      ros2_benchmark_interfaces__srv__SetData_Request__fini(&array->data[i]);
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

ros2_benchmark_interfaces__srv__SetData_Request__Sequence *
ros2_benchmark_interfaces__srv__SetData_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__srv__SetData_Request__Sequence * array = (ros2_benchmark_interfaces__srv__SetData_Request__Sequence *)allocator.allocate(sizeof(ros2_benchmark_interfaces__srv__SetData_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_benchmark_interfaces__srv__SetData_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_benchmark_interfaces__srv__SetData_Request__Sequence__destroy(ros2_benchmark_interfaces__srv__SetData_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_benchmark_interfaces__srv__SetData_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_benchmark_interfaces__srv__SetData_Request__Sequence__are_equal(const ros2_benchmark_interfaces__srv__SetData_Request__Sequence * lhs, const ros2_benchmark_interfaces__srv__SetData_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_benchmark_interfaces__srv__SetData_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_benchmark_interfaces__srv__SetData_Request__Sequence__copy(
  const ros2_benchmark_interfaces__srv__SetData_Request__Sequence * input,
  ros2_benchmark_interfaces__srv__SetData_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_benchmark_interfaces__srv__SetData_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_benchmark_interfaces__srv__SetData_Request * data =
      (ros2_benchmark_interfaces__srv__SetData_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_benchmark_interfaces__srv__SetData_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_benchmark_interfaces__srv__SetData_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_benchmark_interfaces__srv__SetData_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
ros2_benchmark_interfaces__srv__SetData_Response__init(ros2_benchmark_interfaces__srv__SetData_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  msg->success = false;
  return true;
}

void
ros2_benchmark_interfaces__srv__SetData_Response__fini(ros2_benchmark_interfaces__srv__SetData_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
ros2_benchmark_interfaces__srv__SetData_Response__are_equal(const ros2_benchmark_interfaces__srv__SetData_Response * lhs, const ros2_benchmark_interfaces__srv__SetData_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
ros2_benchmark_interfaces__srv__SetData_Response__copy(
  const ros2_benchmark_interfaces__srv__SetData_Response * input,
  ros2_benchmark_interfaces__srv__SetData_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

ros2_benchmark_interfaces__srv__SetData_Response *
ros2_benchmark_interfaces__srv__SetData_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__srv__SetData_Response * msg = (ros2_benchmark_interfaces__srv__SetData_Response *)allocator.allocate(sizeof(ros2_benchmark_interfaces__srv__SetData_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_benchmark_interfaces__srv__SetData_Response));
  bool success = ros2_benchmark_interfaces__srv__SetData_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_benchmark_interfaces__srv__SetData_Response__destroy(ros2_benchmark_interfaces__srv__SetData_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_benchmark_interfaces__srv__SetData_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_benchmark_interfaces__srv__SetData_Response__Sequence__init(ros2_benchmark_interfaces__srv__SetData_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__srv__SetData_Response * data = NULL;

  if (size) {
    data = (ros2_benchmark_interfaces__srv__SetData_Response *)allocator.zero_allocate(size, sizeof(ros2_benchmark_interfaces__srv__SetData_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_benchmark_interfaces__srv__SetData_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_benchmark_interfaces__srv__SetData_Response__fini(&data[i - 1]);
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
ros2_benchmark_interfaces__srv__SetData_Response__Sequence__fini(ros2_benchmark_interfaces__srv__SetData_Response__Sequence * array)
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
      ros2_benchmark_interfaces__srv__SetData_Response__fini(&array->data[i]);
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

ros2_benchmark_interfaces__srv__SetData_Response__Sequence *
ros2_benchmark_interfaces__srv__SetData_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__srv__SetData_Response__Sequence * array = (ros2_benchmark_interfaces__srv__SetData_Response__Sequence *)allocator.allocate(sizeof(ros2_benchmark_interfaces__srv__SetData_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_benchmark_interfaces__srv__SetData_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_benchmark_interfaces__srv__SetData_Response__Sequence__destroy(ros2_benchmark_interfaces__srv__SetData_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_benchmark_interfaces__srv__SetData_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_benchmark_interfaces__srv__SetData_Response__Sequence__are_equal(const ros2_benchmark_interfaces__srv__SetData_Response__Sequence * lhs, const ros2_benchmark_interfaces__srv__SetData_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_benchmark_interfaces__srv__SetData_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_benchmark_interfaces__srv__SetData_Response__Sequence__copy(
  const ros2_benchmark_interfaces__srv__SetData_Response__Sequence * input,
  ros2_benchmark_interfaces__srv__SetData_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_benchmark_interfaces__srv__SetData_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_benchmark_interfaces__srv__SetData_Response * data =
      (ros2_benchmark_interfaces__srv__SetData_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_benchmark_interfaces__srv__SetData_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_benchmark_interfaces__srv__SetData_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_benchmark_interfaces__srv__SetData_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
