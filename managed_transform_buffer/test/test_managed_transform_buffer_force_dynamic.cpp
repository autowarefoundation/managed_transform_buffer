// Copyright 2026 The Autoware Foundation
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

#include "managed_transform_buffer/managed_transform_buffer.hpp"

#include <eigen3/Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>

/** @brief Fixture owning a ManagedTransformBuffer forced to dynamic mode and TF broadcasters. */
class TestManagedTransformBufferForceDynamic : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_{nullptr};
  std::unique_ptr<managed_transform_buffer::ManagedTransformBuffer> managed_tf_buffer_{nullptr};
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
  geometry_msgs::msg::TransformStamped tf_map_to_base_;
  geometry_msgs::msg::TransformStamped tf_base_to_lidar_top_;
  Eigen::Matrix4f eigen_map_to_base_;
  Eigen::Matrix4f eigen_base_to_lidar_top_;
  double precision_;
  rclcpp::Time time_;
  rclcpp::Duration timeout_ = rclcpp::Duration::from_seconds(1);
  static constexpr int broadcast_attempts_ = 10;
  rclcpp::Duration broadcast_retry_timeout_ = rclcpp::Duration::from_seconds(0.1);

  /** @brief Build a transform message from parent_frame to child_frame at the given stamp. */
  geometry_msgs::msg::TransformStamped generateTransformMsg(
    const int32_t seconds, const uint32_t nanoseconds, const std::string & parent_frame,
    const std::string & child_frame, double x, double y, double z, double qx, double qy, double qz,
    double qw)
  {
    rclcpp::Time timestamp(seconds, nanoseconds, node_->get_clock()->get_clock_type());
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = timestamp;
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;
    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;
    return tf_msg;
  }

  /** @brief Wait up to 5 s for a subscriber on topic; return whether one appeared. */
  bool waitForSubscriber(const std::string & topic) const
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node_->count_subscribers(topic) == 0) {
      if (std::chrono::steady_clock::now() >= deadline) return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
  }

  /** @brief Re-send transform on /tf until the buffer resolves it; return the resolved matrix. */
  std::optional<Eigen::Matrix4f> broadcastUntilResolved(
    const geometry_msgs::msg::TransformStamped & transform)
  {
    for (int attempt = 0; attempt < broadcast_attempts_; ++attempt) {
      tf_broadcaster_->sendTransform(transform);
      auto resolved = managed_tf_buffer_->getTransform<Eigen::Matrix4f>(
        transform.header.frame_id, transform.child_frame_id, rclcpp::Time(transform.header.stamp),
        broadcast_retry_timeout_);
      if (resolved.has_value()) return resolved;
    }
    return std::nullopt;
  }

  /** @brief Create the buffer forced dynamic before any transform is broadcast. */
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_managed_transform_buffer_force_dynamic");
    managed_tf_buffer_ = std::make_unique<managed_transform_buffer::ManagedTransformBuffer>(
      node_->get_clock()->get_clock_type(), true);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    tf_map_to_base_ = generateTransformMsg(
      10, 100'000'000, "map", "base_link", 120.0, 240.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    tf_base_to_lidar_top_ = generateTransformMsg(
      10, 100'000'000, "base_link", "lidar_top", 0.690, 0.000, 2.100, -0.007, -0.007, 0.692, 0.722);
    eigen_map_to_base_ = tf2::transformToEigen(tf_map_to_base_).matrix().cast<float>();
    eigen_base_to_lidar_top_ = tf2::transformToEigen(tf_base_to_lidar_top_).matrix().cast<float>();
    precision_ = 0.01;
    time_ = rclcpp::Time(10, 100'000'000);

    ASSERT_TRUE(rclcpp::ok());
  }
};

TEST_F(TestManagedTransformBufferForceDynamic, TestListenerActiveBeforeFirstQuery)
{
  EXPECT_FALSE(managed_tf_buffer_->isStatic());
  ASSERT_TRUE(waitForSubscriber("/tf"));
  ASSERT_TRUE(waitForSubscriber("/tf_static"));

  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_top_);
  auto eigen_map_to_base = broadcastUntilResolved(tf_map_to_base_);
  ASSERT_TRUE(eigen_map_to_base.has_value());
  EXPECT_TRUE(eigen_map_to_base.value().isApprox(eigen_map_to_base_, precision_));

  auto eigen_base_to_lidar_top =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  ASSERT_TRUE(eigen_base_to_lidar_top.has_value());
  EXPECT_TRUE(eigen_base_to_lidar_top.value().isApprox(eigen_base_to_lidar_top_, precision_));

  auto eigen_map_to_nonexistent = managed_tf_buffer_->getTransform<Eigen::Matrix4f>(
    "map", "nonexistent_frame", time_, rclcpp::Duration(0, 0));
  EXPECT_FALSE(eigen_map_to_nonexistent.has_value());

  const auto tf_map_to_base_later =
    generateTransformMsg(11, 0, "map", "base_link", 130.0, 250.0, 1.0, 0.0, 0.0, 0.0, 1.0);
  auto eigen_map_to_base_later = broadcastUntilResolved(tf_map_to_base_later);
  ASSERT_TRUE(eigen_map_to_base_later.has_value());
  EXPECT_TRUE(eigen_map_to_base_later.value().isApprox(
    tf2::transformToEigen(tf_map_to_base_later).matrix().cast<float>(), precision_));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
