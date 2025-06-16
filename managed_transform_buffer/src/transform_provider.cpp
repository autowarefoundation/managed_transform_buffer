// Copyright 2024 The Autoware Foundation
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

#include "managed_transform_buffer/transform_provider.hpp"

#include "managed_transform_buffer/static_transform_provider.hpp"

#include <tf2/LinearMath/Transform.hpp>
#include <tf2/convert.hpp>
#include <tf2_ros/qos.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace managed_transform_buffer
{

TransformProvider::TransformProvider(
  rclcpp::Node * node, const bool force_dynamic, const bool non_managed, tf2::Duration cache_time)
: node_(node), cache_time_(cache_time)
{
  if (non_managed) {
    bypass_tf_provider_ = std::make_unique<BypassTransformProvider>(node, cache_time_);
    return;
  }
  if (force_dynamic) {
    dynamic_tf_provider_ =
      &DynamicTransformProvider::getInstance(node->get_clock()->get_clock_type(), cache_time_);
  } else {
    static_tf_provider_ = std::make_unique<StaticTransformProvider>(node);
  }
}

bool TransformProvider::isStatic() const
{
  return dynamic_tf_provider_ == nullptr;
}

std::optional<TransformStamped> TransformProvider::getTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout)
{
  if (bypass_tf_provider_) {
    return bypass_tf_provider_->getTransform(target_frame, source_frame, time, timeout);
  }
  if (isStatic()) {
    auto static_tf = static_tf_provider_->getStaticTransform(target_frame, source_frame, time);
    if (!static_tf.success) {
      return std::nullopt;
    }
    if (!static_tf.is_static) {
      dynamic_tf_provider_ =
        &DynamicTransformProvider::getInstance(node_->get_clock()->get_clock_type(), cache_time_);
      return dynamic_tf_provider_->getTransform(
        target_frame, source_frame, time, timeout, node_->get_logger());
    }
    return static_tf.transform;
  }
  return dynamic_tf_provider_->getTransform(
    target_frame, source_frame, time, timeout, node_->get_logger());
}

}  // namespace managed_transform_buffer
