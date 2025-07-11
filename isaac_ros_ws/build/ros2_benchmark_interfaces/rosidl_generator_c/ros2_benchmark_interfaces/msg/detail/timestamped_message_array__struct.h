// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_benchmark_interfaces:msg/TimestampedMessageArray.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TIMESTAMPED_MESSAGE_ARRAY__STRUCT_H_
#define ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TIMESTAMPED_MESSAGE_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'keys'
// Member 'timestamps_ns'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/TimestampedMessageArray in the package ros2_benchmark_interfaces.
/**
  * SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
  * Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  * http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  * SPDX-License-Identifier: Apache-2.0
 */
typedef struct ros2_benchmark_interfaces__msg__TimestampedMessageArray
{
  /// This message is used to provide message keys and timestamps.
  /// An array of message keys
  rosidl_runtime_c__int64__Sequence keys;
  /// An array of timestamps in (nanoseconds). The length should match
  /// the length of keys
  rosidl_runtime_c__int64__Sequence timestamps_ns;
} ros2_benchmark_interfaces__msg__TimestampedMessageArray;

// Struct for a sequence of ros2_benchmark_interfaces__msg__TimestampedMessageArray.
typedef struct ros2_benchmark_interfaces__msg__TimestampedMessageArray__Sequence
{
  ros2_benchmark_interfaces__msg__TimestampedMessageArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__msg__TimestampedMessageArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TIMESTAMPED_MESSAGE_ARRAY__STRUCT_H_
