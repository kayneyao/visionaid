// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ros2_benchmark_interfaces:msg/TopicMessageCount.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_COUNT__FUNCTIONS_H_
#define ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_COUNT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ros2_benchmark_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "ros2_benchmark_interfaces/msg/detail/topic_message_count__struct.h"

/// Initialize msg/TopicMessageCount message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ros2_benchmark_interfaces__msg__TopicMessageCount
 * )) before or use
 * ros2_benchmark_interfaces__msg__TopicMessageCount__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_benchmark_interfaces
bool
ros2_benchmark_interfaces__msg__TopicMessageCount__init(ros2_benchmark_interfaces__msg__TopicMessageCount * msg);

/// Finalize msg/TopicMessageCount message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_benchmark_interfaces
void
ros2_benchmark_interfaces__msg__TopicMessageCount__fini(ros2_benchmark_interfaces__msg__TopicMessageCount * msg);

/// Create msg/TopicMessageCount message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ros2_benchmark_interfaces__msg__TopicMessageCount__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_benchmark_interfaces
ros2_benchmark_interfaces__msg__TopicMessageCount *
ros2_benchmark_interfaces__msg__TopicMessageCount__create();

/// Destroy msg/TopicMessageCount message.
/**
 * It calls
 * ros2_benchmark_interfaces__msg__TopicMessageCount__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_benchmark_interfaces
void
ros2_benchmark_interfaces__msg__TopicMessageCount__destroy(ros2_benchmark_interfaces__msg__TopicMessageCount * msg);

/// Check for msg/TopicMessageCount message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_benchmark_interfaces
bool
ros2_benchmark_interfaces__msg__TopicMessageCount__are_equal(const ros2_benchmark_interfaces__msg__TopicMessageCount * lhs, const ros2_benchmark_interfaces__msg__TopicMessageCount * rhs);

/// Copy a msg/TopicMessageCount message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_benchmark_interfaces
bool
ros2_benchmark_interfaces__msg__TopicMessageCount__copy(
  const ros2_benchmark_interfaces__msg__TopicMessageCount * input,
  ros2_benchmark_interfaces__msg__TopicMessageCount * output);

/// Initialize array of msg/TopicMessageCount messages.
/**
 * It allocates the memory for the number of elements and calls
 * ros2_benchmark_interfaces__msg__TopicMessageCount__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_benchmark_interfaces
bool
ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__init(ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * array, size_t size);

/// Finalize array of msg/TopicMessageCount messages.
/**
 * It calls
 * ros2_benchmark_interfaces__msg__TopicMessageCount__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_benchmark_interfaces
void
ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__fini(ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * array);

/// Create array of msg/TopicMessageCount messages.
/**
 * It allocates the memory for the array and calls
 * ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_benchmark_interfaces
ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence *
ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__create(size_t size);

/// Destroy array of msg/TopicMessageCount messages.
/**
 * It calls
 * ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_benchmark_interfaces
void
ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__destroy(ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * array);

/// Check for msg/TopicMessageCount message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_benchmark_interfaces
bool
ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__are_equal(const ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * lhs, const ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * rhs);

/// Copy an array of msg/TopicMessageCount messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_benchmark_interfaces
bool
ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__copy(
  const ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * input,
  ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_COUNT__FUNCTIONS_H_
