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

#include "managed_transform_buffer/dynamic_transform_provider.hpp"

#include <tf2/LinearMath/Transform.hpp>
#include <tf2/convert.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/qos.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <random>
#include <string>
#include <vector>

namespace managed_transform_buffer
{

DynamicTransformProvider & DynamicTransformProvider::getInstance(
  rcl_clock_type_t clock_type, tf2::Duration cache_time)
{
  static DynamicTransformProvider instance(clock_type, cache_time);

  if (clock_type != instance.clock_->get_clock_type()) {
    RCLCPP_WARN_THROTTLE(
      instance.logger_, *instance.clock_, 3000,
      "Input clock type does not match (%d vs. %d). Input clock type will be ignored.", clock_type,
      instance.clock_->get_clock_type());
  }
  return instance;
}

std::optional<TransformStamped> DynamicTransformProvider::getTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  try {
    auto tf = tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
    return std::make_optional<TransformStamped>(tf);
  } catch (const tf2::ExtrapolationException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger, *clock_, 3000,
      "The transformation has been requested before it could be aggregated. Potentially next "
      "request will succeed. %s",
      ex.what());
    return std::nullopt;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(
      logger, *clock_, 3000, "Failure to get transform from %s to %s with error: %s",
      target_frame.c_str(), source_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

rclcpp::Clock::SharedPtr DynamicTransformProvider::getClock() const
{
  return clock_;
}

DynamicTransformProvider::DynamicTransformProvider(
  rcl_clock_type_t clock_type, tf2::Duration cache_time)
: clock_(std::make_shared<rclcpp::Clock>(clock_type)),
  logger_(rclcpp::get_logger("managed_transform_buffer"))
{
  // Node
  {
    auto random_engine = std::mt19937(std::random_device{}());
    auto dis = std::uniform_int_distribution<>(0, 0xFFFFFF);
    rclcpp::NodeOptions options;
    options.start_parameter_event_publisher(false);
    options.start_parameter_services(false);
    std::stringstream sstream;
    sstream << "managed_tf_listener_impl_" << std::hex << dis(random_engine) << dis(random_engine);
    options.arguments({"--ros-args", "-r", "__node:=" + sstream.str()});
    node_ = rclcpp::Node::make_unique("_", options);
  }

  // TF buffer
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_, cache_time);
    tf_buffer_->setUsingDedicatedThread(true);
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_->get_node_base_interface(), node_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
  }

  // TF subscriptions
  {
    auto callback_group =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto tf_options = tf2_ros::detail::get_default_transform_listener_sub_options();
    tf_options.callback_group = callback_group;
    tf_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    auto tf_static_options = tf2_ros::detail::get_default_transform_listener_static_sub_options();
    tf_static_options.callback_group = callback_group;
    cb_ = std::bind(&DynamicTransformProvider::tfCallback, this, std::placeholders::_1, false);
    cb_static_ =
      std::bind(&DynamicTransformProvider::tfCallback, this, std::placeholders::_1, true);
    tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", tf2_ros::DynamicListenerQoS(), cb_, tf_options);
    tf_static_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", tf2_ros::StaticListenerQoS(), cb_static_, tf_static_options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_thread_ = std::make_shared<std::thread>(
      std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, executor_));
    executor_->add_callback_group(callback_group, node_->get_node_base_interface());
  }
}

DynamicTransformProvider::~DynamicTransformProvider()
{
  executor_->cancel();
  if (executor_thread_->joinable()) {
    executor_thread_->join();
  }
}

void DynamicTransformProvider::tfCallback(
  const tf2_msgs::msg::TFMessage::SharedPtr msg, const bool is_static)
{
  std::string authority = "Authority undetectable";
  for (const auto & tf : msg->transforms) {
    try {
      tf_buffer_->setTransform(tf, authority, is_static);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_THROTTLE(
        logger_, *clock_, 3000, "Failure to set received transform from %s to %s with error: %s\n",
        tf.child_frame_id.c_str(), tf.header.frame_id.c_str(), ex.what());
    }
  }
}

}  // namespace managed_transform_buffer
