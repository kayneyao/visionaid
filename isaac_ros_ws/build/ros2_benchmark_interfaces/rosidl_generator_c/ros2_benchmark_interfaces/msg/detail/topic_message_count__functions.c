// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_benchmark_interfaces:msg/TopicMessageCount.idl
// generated code does not contain a copyright notice
#include "ros2_benchmark_interfaces/msg/detail/topic_message_count__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `topic_name`
#include "rosidl_runtime_c/string_functions.h"

bool
ros2_benchmark_interfaces__msg__TopicMessageCount__init(ros2_benchmark_interfaces__msg__TopicMessageCount * msg)
{
  if (!msg) {
    return false;
  }
  // topic_name
  if (!rosidl_runtime_c__String__init(&msg->topic_name)) {
    ros2_benchmark_interfaces__msg__TopicMessageCount__fini(msg);
    return false;
  }
  // message_count
  return true;
}

void
ros2_benchmark_interfaces__msg__TopicMessageCount__fini(ros2_benchmark_interfaces__msg__TopicMessageCount * msg)
{
  if (!msg) {
    return;
  }
  // topic_name
  rosidl_runtime_c__String__fini(&msg->topic_name);
  // message_count
}

bool
ros2_benchmark_interfaces__msg__TopicMessageCount__are_equal(const ros2_benchmark_interfaces__msg__TopicMessageCount * lhs, const ros2_benchmark_interfaces__msg__TopicMessageCount * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // topic_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->topic_name), &(rhs->topic_name)))
  {
    return false;
  }
  // message_count
  if (lhs->message_count != rhs->message_count) {
    return false;
  }
  return true;
}

bool
ros2_benchmark_interfaces__msg__TopicMessageCount__copy(
  const ros2_benchmark_interfaces__msg__TopicMessageCount * input,
  ros2_benchmark_interfaces__msg__TopicMessageCount * output)
{
  if (!input || !output) {
    return false;
  }
  // topic_name
  if (!rosidl_runtime_c__String__copy(
      &(input->topic_name), &(output->topic_name)))
  {
    return false;
  }
  // message_count
  output->message_count = input->message_count;
  return true;
}

ros2_benchmark_interfaces__msg__TopicMessageCount *
ros2_benchmark_interfaces__msg__TopicMessageCount__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__msg__TopicMessageCount * msg = (ros2_benchmark_interfaces__msg__TopicMessageCount *)allocator.allocate(sizeof(ros2_benchmark_interfaces__msg__TopicMessageCount), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_benchmark_interfaces__msg__TopicMessageCount));
  bool success = ros2_benchmark_interfaces__msg__TopicMessageCount__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_benchmark_interfaces__msg__TopicMessageCount__destroy(ros2_benchmark_interfaces__msg__TopicMessageCount * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_benchmark_interfaces__msg__TopicMessageCount__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__init(ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__msg__TopicMessageCount * data = NULL;

  if (size) {
    data = (ros2_benchmark_interfaces__msg__TopicMessageCount *)allocator.zero_allocate(size, sizeof(ros2_benchmark_interfaces__msg__TopicMessageCount), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_benchmark_interfaces__msg__TopicMessageCount__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_benchmark_interfaces__msg__TopicMessageCount__fini(&data[i - 1]);
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
ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__fini(ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * array)
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
      ros2_benchmark_interfaces__msg__TopicMessageCount__fini(&array->data[i]);
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

ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence *
ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * array = (ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence *)allocator.allocate(sizeof(ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__destroy(ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__are_equal(const ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * lhs, const ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_benchmark_interfaces__msg__TopicMessageCount__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__copy(
  const ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * input,
  ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_benchmark_interfaces__msg__TopicMessageCount);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_benchmark_interfaces__msg__TopicMessageCount * data =
      (ros2_benchmark_interfaces__msg__TopicMessageCount *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_benchmark_interfaces__msg__TopicMessageCount__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_benchmark_interfaces__msg__TopicMessageCount__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_benchmark_interfaces__msg__TopicMessageCount__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
