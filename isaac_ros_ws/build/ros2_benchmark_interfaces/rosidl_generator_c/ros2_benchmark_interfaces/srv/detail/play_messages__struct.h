// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_benchmark_interfaces:srv/PlayMessages.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__PLAY_MESSAGES__STRUCT_H_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__PLAY_MESSAGES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'PLAYBACK_MODE_TIMELINE'.
/**
  * Definitions for playback modes
 */
enum
{
  ros2_benchmark_interfaces__srv__PlayMessages_Request__PLAYBACK_MODE_TIMELINE = 0
};

/// Constant 'PLAYBACK_MODE_LOOPING'.
enum
{
  ros2_benchmark_interfaces__srv__PlayMessages_Request__PLAYBACK_MODE_LOOPING = 1
};

/// Constant 'PLAYBACK_MODE_SWEEPING'.
enum
{
  ros2_benchmark_interfaces__srv__PlayMessages_Request__PLAYBACK_MODE_SWEEPING = 2
};

/// Struct defined in srv/PlayMessages in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__PlayMessages_Request
{
  /// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
  /// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
  ///
  /// Licensed under the Apache License, Version 2.0 (the "License");
  /// you may not use this file except in compliance with the License.
  /// You may obtain a copy of the License at
  ///
  /// http://www.apache.org/licenses/LICENSE-2.0
  ///
  /// Unless required by applicable law or agreed to in writing, software
  /// distributed under the License is distributed on an "AS IS" BASIS,
  /// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  /// See the License for the specific language governing permissions and
  /// limitations under the License.
  ///
  /// SPDX-License-Identifier: Apache-2.0
  /// Playback mode for how the buffered messages should be published
  uint8_t playback_mode;
  /// The number of messages the buffer node should publish.
  /// Note that this does NOT need to be the same as the buffer_length (the
  /// number of the buffered messages) rquested in StartRecording.srv. The
  /// node will reuse and repeat messages if this value is smaller than
  /// buffer_length; otherwise omit as necessary in order to publish exactly
  /// the requested number of messages.
  /// A value of 0 or smaller defaults to publishing all buffered messages
  /// exactly once.
  /// It is ignored when the playback_mode is set to PLAYBACK_MODE_TIMELINE.
  uint64_t message_count;
  /// The target rate, in hertz, to publish messages. A warning will be printed
  /// to the log console if the target rate cannot be maintained during the
  /// playback.
  /// It is ignored when the plackback_mode is set to PLAYBACK_MODE_TIMELINE.
  double target_publisher_rate;
  /// Whether or not to stop and return an unsuccessful response if the target
  /// publish rate cannot be maintained.
  bool enforce_publisher_rate;
  /// Whether to use header.stamp.sec in each message's header as a message
  /// ID field
  bool revise_timestamps_as_message_ids;
} ros2_benchmark_interfaces__srv__PlayMessages_Request;

// Struct for a sequence of ros2_benchmark_interfaces__srv__PlayMessages_Request.
typedef struct ros2_benchmark_interfaces__srv__PlayMessages_Request__Sequence
{
  ros2_benchmark_interfaces__srv__PlayMessages_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__PlayMessages_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'timestamps'
#include "ros2_benchmark_interfaces/msg/detail/timestamped_message_array__struct.h"

/// Struct defined in srv/PlayMessages in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__PlayMessages_Response
{
  /// Whether or not playback proceeds successfully. Possible failures include:
  /// - Message timeline information is not available when using PLAYBACK_MODE_TIMELINE
  /// - Failed to store a message's publish timestamp due to duplicate message keys
  /// - Not all expected messages are present in the buffers before starting playback
  /// - Key duplication in buffered messages provided to the node
  /// - The target publisher rate cannot be met when enforce_publisher_rate is enabled
  bool success;
  /// The keys and timestamps of the messages that have been published
  ros2_benchmark_interfaces__msg__TimestampedMessageArray timestamps;
} ros2_benchmark_interfaces__srv__PlayMessages_Response;

// Struct for a sequence of ros2_benchmark_interfaces__srv__PlayMessages_Response.
typedef struct ros2_benchmark_interfaces__srv__PlayMessages_Response__Sequence
{
  ros2_benchmark_interfaces__srv__PlayMessages_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__PlayMessages_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__PLAY_MESSAGES__STRUCT_H_
