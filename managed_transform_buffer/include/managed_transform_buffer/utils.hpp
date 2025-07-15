// Copyright 2025 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc.

#ifndef MANAGED_TRANSFORM_BUFFER__UTILS_HPP
#define MANAGED_TRANSFORM_BUFFER__UTILS_HPP

#include <tf2/buffer_core.h>

#include <cstdint>

namespace managed_transform_buffer
{

static constexpr uint32_t DEFAULT_TF_SERVER_TIMEOUT_MS = 5000;

/**
 * @brief Configuration structure for ManagedTransformBuffer.
 *
 * This structure controls how the ManagedTransformBuffer operate.
 */
struct ManagedTransformBufferConfig
{
  /**
   * @brief Construct a new ManagedTransformBufferConfig.
   *
   * @param tf_server_timeout_ms Maximum time to wait for static transform service availability
   * (milliseconds)
   * @param force_dynamic If true, forces the use of dynamic TF listener instead of static provider
   * @param non_managed If true, bypasses managed behavior and uses standard ROS TF lookup
   * @param cache_time How long to keep transform history in the buffer
   */
  explicit ManagedTransformBufferConfig(
    const uint32_t tf_server_timeout_ms = DEFAULT_TF_SERVER_TIMEOUT_MS,
    const bool force_dynamic = false, const bool non_managed = false,
    tf2::Duration cache_time = tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME))
  : tf_server_timeout_ms(tf_server_timeout_ms),
    force_dynamic(force_dynamic),
    non_managed(non_managed),
    cache_time(cache_time)
  {
  }

  uint32_t tf_server_timeout_ms{DEFAULT_TF_SERVER_TIMEOUT_MS};
  bool force_dynamic{false};
  bool non_managed{false};
  tf2::Duration cache_time{tf2::BUFFER_CORE_DEFAULT_CACHE_TIME};
};

}  // namespace managed_transform_buffer

#endif  // MANAGED_TRANSFORM_BUFFER__UTILS_HPP
