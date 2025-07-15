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

#include "managed_transform_buffer/managed_transform_buffer.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

class TestManagedTransformBuffer : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::unique_ptr<managed_transform_buffer::ManagedTransformBuffer> managed_tf_buffer_{nullptr};
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
  geometry_msgs::msg::TransformStamped tf_map_to_base_;
  geometry_msgs::msg::TransformStamped tf_base_to_lidar_top_;
  geometry_msgs::msg::TransformStamped tf_base_to_lidar_right_;
  Eigen::Matrix4f eigen_map_to_base_;
  Eigen::Matrix4f eigen_base_to_lidar_top_;
  Eigen::Matrix4f eigen_base_to_lidar_right_;
  std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud_in_{nullptr};
  double precision_;
  rclcpp::Time time_;
  rclcpp::Duration timeout_ = rclcpp::Duration::from_seconds(1);

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

  void broadcastDynamicTf(geometry_msgs::msg::TransformStamped transform, uint32_t seconds = 1)
  {
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this, transform]() -> void {
      tf_broadcaster_->sendTransform(transform);
    });

    rclcpp::Rate r(10);
    rclcpp::spin_some(node_);
    for (uint32_t i = 0; i < 10u * seconds; ++i) {
      r.sleep();
      rclcpp::spin_some(node_);
    }

    timer_->cancel();
    timer_->reset();
  }

  void sendStaticTransform(const geometry_msgs::msg::TransformStamped & transform, bool wait)
  {
    static_tf_broadcaster_->sendTransform(transform);
    if (wait) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    rclcpp::spin_some(node_);
  }

  void sendDynamicTransform(const geometry_msgs::msg::TransformStamped & transform, bool wait)
  {
    tf_broadcaster_->sendTransform(transform);
    if (wait) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    rclcpp::spin_some(node_);
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_managed_transform_buffer");
    managed_tf_buffer_ =
      std::make_unique<managed_transform_buffer::ManagedTransformBuffer>(node_.get());
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    tf_map_to_base_ = generateTransformMsg(
      10, 100'000'000, "map", "base_link", 120.0, 240.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    tf_base_to_lidar_top_ = generateTransformMsg(
      10, 100'000'000, "base_link", "lidar_top", 0.690, 0.000, 2.100, -0.007, -0.007, 0.692, 0.722);
    tf_base_to_lidar_right_ = generateTransformMsg(
      10, 100'000'000, "base_link", "lidar_right", 0.0, -0.56362, -0.30555, 0.244, 0.248, 0.665,
      0.661);
    eigen_map_to_base_ = tf2::transformToEigen(tf_map_to_base_).matrix().cast<float>();
    eigen_base_to_lidar_top_ = tf2::transformToEigen(tf_base_to_lidar_top_).matrix().cast<float>();
    eigen_base_to_lidar_right_ =
      tf2::transformToEigen(tf_base_to_lidar_right_).matrix().cast<float>();
    cloud_in_ = std::make_unique<sensor_msgs::msg::PointCloud2>();
    precision_ = 0.01;
    time_ = rclcpp::Time(10, 100'000'000);

    // Set up the fields for x, y, and z coordinates
    cloud_in_->fields.resize(3);
    sensor_msgs::PointCloud2Modifier modifier(*cloud_in_);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    // Resize the cloud to hold points_per_pointcloud_ points
    modifier.resize(10);

    // Create an iterator for the x, y, z fields
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_in_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_in_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_in_, "z");

    // Populate the point cloud
    for (size_t i = 0; i < modifier.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
      *iter_x = static_cast<float>(i);
      *iter_y = static_cast<float>(i);
      *iter_z = static_cast<float>(i);
    }

    // Set up cloud header
    cloud_in_->header.frame_id = "lidar_top";
    cloud_in_->header.stamp = time_;

    // Set up the static transform server
    sendStaticTransform(tf_base_to_lidar_top_, false);
    sendStaticTransform(tf_base_to_lidar_right_, false);
    sendDynamicTransform(tf_map_to_base_, true);

    ASSERT_TRUE(rclcpp::ok());
  }

  void TearDown() override {}
};

TEST_F(TestManagedTransformBuffer, TestReturn)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);

  auto eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  EXPECT_TRUE(eigen_transform.has_value());

  auto tf2_transform =
    managed_tf_buffer_->getTransform<tf2::Transform>("base_link", "lidar_top", time_, timeout_);
  EXPECT_TRUE(tf2_transform.has_value());

  auto tf_msg_transform = managed_tf_buffer_->getTransform<geometry_msgs::msg::TransformStamped>(
    "base_link", "lidar_top", time_, timeout_);
  EXPECT_TRUE(tf_msg_transform.has_value());
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformNoExist)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);

  auto eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "fake_link", time_, timeout_);
  EXPECT_FALSE(eigen_transform.has_value());
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformBase)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);
  auto eigen_base_to_lidar_top =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  ASSERT_TRUE(eigen_base_to_lidar_top.has_value());
  EXPECT_TRUE(eigen_base_to_lidar_top.value().isApprox(eigen_base_to_lidar_top_, precision_));
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformSameFrame)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);

  auto eigen_base_to_base =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "base_link", time_, timeout_);
  ASSERT_TRUE(eigen_base_to_base.has_value());
  EXPECT_TRUE(eigen_base_to_base.value().isApprox(Eigen::Matrix4f::Identity(), precision_));
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformInverse)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);

  auto eigen_lidar_top_tobase =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("lidar_top", "base_link", time_, timeout_);
  ASSERT_TRUE(eigen_lidar_top_tobase.has_value());
  EXPECT_TRUE(
    eigen_lidar_top_tobase.value().isApprox(eigen_base_to_lidar_top_.inverse(), precision_));
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformNonDirect)
{
  sendStaticTransform(tf_base_to_lidar_top_, false);
  sendStaticTransform(tf_base_to_lidar_right_, true);

  auto eigen_lidar_top_to_lidar_right =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("lidar_top", "lidar_right", time_, timeout_);
  ASSERT_TRUE(eigen_lidar_top_to_lidar_right.has_value());
  EXPECT_TRUE(eigen_lidar_top_to_lidar_right.value().isApprox(
    eigen_base_to_lidar_top_.inverse() * eigen_base_to_lidar_right_, precision_));
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformDynamic)
{
  sendStaticTransform(tf_base_to_lidar_top_, false);
  sendStaticTransform(tf_base_to_lidar_right_, true);

  std::future<void> future =
    std::async(std::launch::async, [this]() { broadcastDynamicTf(tf_map_to_base_); });
  auto eigen_map_to_base =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("map", "base_link", time_, timeout_);
  future.wait();

  ASSERT_TRUE(eigen_map_to_base.has_value());
  EXPECT_TRUE(eigen_map_to_base.value().isApprox(eigen_map_to_base_, precision_));
  EXPECT_FALSE(managed_tf_buffer_->isStatic());

  auto eigen_lidar_top_to_lidar_right =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("lidar_top", "lidar_right", time_, timeout_);
  ASSERT_TRUE(eigen_lidar_top_to_lidar_right.has_value());
  EXPECT_TRUE(eigen_lidar_top_to_lidar_right.value().isApprox(
    eigen_base_to_lidar_top_.inverse() * eigen_base_to_lidar_right_, precision_));
  EXPECT_FALSE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformMultipleCall)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);

  std::optional<Eigen::Matrix4f> eigen_transform;
  eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "fake_link", time_, timeout_);
  EXPECT_FALSE(eigen_transform.has_value());

  eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("lidar_top", "base_link", time_, timeout_);
  ASSERT_TRUE(eigen_transform.has_value());
  EXPECT_TRUE(eigen_transform.value().isApprox(eigen_base_to_lidar_top_.inverse(), precision_));

  eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("fake_link", "fake_link", time_, timeout_);
  ASSERT_TRUE(eigen_transform.has_value());
  EXPECT_TRUE(eigen_transform.value().isApprox(Eigen::Matrix4f::Identity(), precision_));

  eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  ASSERT_TRUE(eigen_transform.has_value());
  EXPECT_TRUE(eigen_transform.value().isApprox(eigen_base_to_lidar_top_, precision_));

  eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("fake_link", "lidar_top", time_, timeout_);
  EXPECT_FALSE(eigen_transform.has_value());

  eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  ASSERT_TRUE(eigen_transform.has_value());
  EXPECT_TRUE(eigen_transform.value().isApprox(eigen_base_to_lidar_top_, precision_));

  std::future<void> future =
    std::async(std::launch::async, [this]() { broadcastDynamicTf(tf_map_to_base_); });
  auto eigen_map_to_base =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("map", "base_link", time_, timeout_);
  future.wait();
  ASSERT_TRUE(eigen_map_to_base.has_value());
  EXPECT_TRUE(eigen_map_to_base.value().isApprox(eigen_map_to_base_, precision_));

  eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("fake_link", "fake_link", time_, timeout_);
  ASSERT_TRUE(eigen_transform.has_value());
  EXPECT_TRUE(eigen_transform.value().isApprox(Eigen::Matrix4f::Identity(), precision_));
  EXPECT_FALSE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformEmptyPointCloud)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);

  auto cloud_in = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud_in->header.frame_id = "lidar_top";
  cloud_in->header.stamp = rclcpp::Time(10, 100'000'000);
  auto cloud_out = std::make_unique<sensor_msgs::msg::PointCloud2>();

  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("lidar_top", *cloud_in, *cloud_out, time_, timeout_));
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("base_link", *cloud_in, *cloud_out, time_, timeout_));
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("fake_link", *cloud_in, *cloud_out, time_, timeout_));
}

TEST_F(TestManagedTransformBuffer, TestTransformEmptyPointCloudNoHeader)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);
  auto cloud_in = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto cloud_out = std::make_unique<sensor_msgs::msg::PointCloud2>();

  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("lidar_top", *cloud_in, *cloud_out, time_, timeout_));
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("base_link", *cloud_in, *cloud_out, time_, timeout_));
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("fake_link", *cloud_in, *cloud_out, time_, timeout_));
}

TEST_F(TestManagedTransformBuffer, TestTransformPointCloud)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);
  auto cloud_out = std::make_unique<sensor_msgs::msg::PointCloud2>();

  // Transform cloud with header
  EXPECT_TRUE(
    managed_tf_buffer_->transformPointcloud("lidar_top", *cloud_in_, *cloud_out, time_, timeout_));
  EXPECT_TRUE(
    managed_tf_buffer_->transformPointcloud("base_link", *cloud_in_, *cloud_out, time_, timeout_));
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("fake_link", *cloud_in_, *cloud_out, time_, timeout_));
}

TEST_F(TestManagedTransformBuffer, TestTransformPointCloudNoHeader)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);
  auto cloud_out = std::make_unique<sensor_msgs::msg::PointCloud2>();

  // Transform cloud without header
  auto cloud_in = std::make_unique<sensor_msgs::msg::PointCloud2>(*cloud_in_);
  cloud_in->header.frame_id = "";
  cloud_in->header.stamp = rclcpp::Time(0, 0);
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("lidar_top", *cloud_in, *cloud_out, time_, timeout_));
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("base_link", *cloud_in, *cloud_out, time_, timeout_));
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("fake_link", *cloud_in, *cloud_out, time_, timeout_));
}

TEST_F(TestManagedTransformBuffer, TestAllTemplateSpecializations)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);

  // Test all Eigen types
  auto eigen_affine3f =
    managed_tf_buffer_->getTransform<Eigen::Affine3f>("base_link", "lidar_top", time_, timeout_);
  EXPECT_TRUE(eigen_affine3f.has_value());

  auto eigen_affine3d =
    managed_tf_buffer_->getTransform<Eigen::Affine3d>("base_link", "lidar_top", time_, timeout_);
  EXPECT_TRUE(eigen_affine3d.has_value());

  auto eigen_matrix4f =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  EXPECT_TRUE(eigen_matrix4f.has_value());

  auto eigen_matrix4d =
    managed_tf_buffer_->getTransform<Eigen::Matrix4d>("base_link", "lidar_top", time_, timeout_);
  EXPECT_TRUE(eigen_matrix4d.has_value());

  // Test rclcpp::Time versions
  auto rclcpp_time_tf = managed_tf_buffer_->getTransform<geometry_msgs::msg::TransformStamped>(
    "base_link", "lidar_top", time_, rclcpp::Duration::from_seconds(1));
  EXPECT_TRUE(rclcpp_time_tf.has_value());

  // Test latest transform versions
  auto latest_eigen =
    managed_tf_buffer_->getLatestTransform<Eigen::Matrix4f>("base_link", "lidar_top");
  EXPECT_TRUE(latest_eigen.has_value());

  auto latest_tf2 =
    managed_tf_buffer_->getLatestTransform<tf2::Transform>("base_link", "lidar_top");
  EXPECT_TRUE(latest_tf2.has_value());
}

TEST_F(TestManagedTransformBuffer, TestForceDynamicMode)
{
  auto config = managed_transform_buffer::ManagedTransformBufferConfig(
    managed_transform_buffer::DEFAULT_TF_SERVER_TIMEOUT_MS, true, false);  // force_dynamic = true
  auto force_dynamic_buffer =
    std::make_unique<managed_transform_buffer::ManagedTransformBuffer>(node_.get(), config);

  sendStaticTransform(tf_base_to_lidar_top_, true);

  // Even with static transforms, should be in dynamic mode
  auto transform =
    force_dynamic_buffer->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  EXPECT_TRUE(transform.has_value());
  EXPECT_FALSE(force_dynamic_buffer->isStatic());  // Should be false due to force_dynamic
}

TEST_F(TestManagedTransformBuffer, TestNonManagedMode)
{
  auto config = managed_transform_buffer::ManagedTransformBufferConfig(
    managed_transform_buffer::DEFAULT_TF_SERVER_TIMEOUT_MS, false, true);  // non_managed = true
  auto non_managed_buffer =
    std::make_unique<managed_transform_buffer::ManagedTransformBuffer>(node_.get(), config);

  sendStaticTransform(tf_base_to_lidar_top_, true);

  auto transform =
    non_managed_buffer->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  EXPECT_TRUE(transform.has_value());
  EXPECT_TRUE(non_managed_buffer->isStatic());  // Bypass mode should report static
}

TEST_F(TestManagedTransformBuffer, TestTimeoutBehavior)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);

  // Very short timeout for non-existent transform
  auto short_timeout = rclcpp::Duration::from_seconds(0.001);
  auto transform = managed_tf_buffer_->getTransform<Eigen::Matrix4f>(
    "base_link", "non_existent_frame", time_, short_timeout);
  EXPECT_FALSE(transform.has_value());

  // Zero timeout
  auto zero_timeout = rclcpp::Duration::from_seconds(0);
  transform = managed_tf_buffer_->getTransform<Eigen::Matrix4f>(
    "base_link", "non_existent_frame", time_, zero_timeout);
  EXPECT_FALSE(transform.has_value());
}

TEST_F(TestManagedTransformBuffer, TestEdgeCaseFrameNames)
{
  // Empty frame names
  auto transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("", "lidar_top", time_, timeout_);
  EXPECT_FALSE(transform.has_value());

  transform = managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "", time_, timeout_);
  EXPECT_FALSE(transform.has_value());

  // Both empty
  transform = managed_tf_buffer_->getTransform<Eigen::Matrix4f>("", "", time_, timeout_);
  EXPECT_TRUE(transform.has_value());  // Should return identity
  EXPECT_TRUE(transform.value().isApprox(Eigen::Matrix4f::Identity(), precision_));

  // Special characters in frame names
  auto special_tf = generateTransformMsg(
    10, 100'000'000, "frame/with/slashes", "frame-with-dashes", 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  sendStaticTransform(special_tf, true);

  transform = managed_tf_buffer_->getTransform<Eigen::Matrix4f>(
    "frame/with/slashes", "frame-with-dashes", time_, timeout_);
  EXPECT_TRUE(transform.has_value());
}

TEST_F(TestManagedTransformBuffer, TestTimeEdgeCases)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);

  // Future time
  auto future_time = rclcpp::Time(100, 0);
  auto transform = managed_tf_buffer_->getTransform<Eigen::Matrix4f>(
    "base_link", "lidar_top", future_time, timeout_);
  EXPECT_TRUE(transform.has_value());  // Static transforms should work with any time

  // Very old time
  auto old_time = rclcpp::Time(1, 0);
  transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", old_time, timeout_);
  EXPECT_TRUE(transform.has_value());

  // Zero time (should get latest)
  auto zero_time = rclcpp::Time(0, 0);
  transform = managed_tf_buffer_->getTransform<Eigen::Matrix4f>(
    "base_link", "lidar_top", zero_time, timeout_);
  EXPECT_TRUE(transform.has_value());
}

TEST_F(TestManagedTransformBuffer, TestCacheBehavior)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);

  // First call should populate cache
  auto transform1 =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  EXPECT_TRUE(transform1.has_value());
  EXPECT_TRUE(managed_tf_buffer_->isStatic());

  // Second call should use cache (should be faster)
  auto start = std::chrono::high_resolution_clock::now();
  auto transform2 =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  auto end = std::chrono::high_resolution_clock::now();
  EXPECT_TRUE(transform2.has_value());
  EXPECT_TRUE(managed_tf_buffer_->isStatic());

  // Results should be identical
  EXPECT_TRUE(transform1.value().isApprox(transform2.value(), precision_));

  // Cached call should be very fast (< 1ms)
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  EXPECT_LT(duration.count(), 10);  // Should be very fast from cache
}

TEST_F(TestManagedTransformBuffer, TestPointCloudTransformationEdgeCases)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);

  // Empty point cloud data but with proper fields
  auto empty_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*empty_cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(0);  // No points
  empty_cloud->header.frame_id = "lidar_top";
  empty_cloud->header.stamp = time_;

  auto cloud_out = std::make_unique<sensor_msgs::msg::PointCloud2>();
  EXPECT_TRUE(managed_tf_buffer_->transformPointcloud(
    "base_link", *empty_cloud, *cloud_out, time_, timeout_));
  EXPECT_EQ(cloud_out->header.frame_id, "base_link");

  // Large point cloud (stress test)
  auto large_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  sensor_msgs::PointCloud2Modifier large_modifier(*large_cloud);
  large_modifier.setPointCloud2FieldsByString(1, "xyz");
  large_modifier.resize(10000);  // Large cloud
  large_cloud->header.frame_id = "lidar_top";
  large_cloud->header.stamp = time_;

  auto start = std::chrono::high_resolution_clock::now();
  EXPECT_TRUE(managed_tf_buffer_->transformPointcloud(
    "base_link", *large_cloud, *cloud_out, time_, timeout_));
  auto end = std::chrono::high_resolution_clock::now();
  EXPECT_EQ(cloud_out->header.frame_id, "base_link");

  // Performance check - should complete within reasonable time
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  EXPECT_LT(duration.count(), 1000);  // Should complete within 1 second
}

TEST_F(TestManagedTransformBuffer, TestConcurrentAccess)
{
  sendStaticTransform(tf_base_to_lidar_top_, false);
  sendStaticTransform(tf_base_to_lidar_right_, true);

  const int num_threads = 4;
  const int calls_per_thread = 50;
  std::vector<std::thread> threads;
  std::vector<bool> results(num_threads, false);

  // Launch multiple threads accessing the buffer concurrently
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([this, i, calls_per_thread, &results]() {
      bool all_success = true;
      for (int j = 0; j < calls_per_thread; ++j) {
        auto transform = managed_tf_buffer_->getTransform<Eigen::Matrix4f>(
          "base_link", "lidar_top", time_, timeout_);
        if (!transform.has_value()) {
          all_success = false;
          break;
        }
      }
      results[i] = all_success;
    });
  }

  // Wait for all threads to complete
  for (auto & thread : threads) {
    thread.join();
  }

  // All threads should succeed
  for (bool result : results) {
    EXPECT_TRUE(result);
  }
}

TEST_F(TestManagedTransformBuffer, TestTransformConsistency)
{
  sendStaticTransform(tf_base_to_lidar_top_, true);

  // Get transform in different formats
  auto tf_stamped = managed_tf_buffer_->getTransform<geometry_msgs::msg::TransformStamped>(
    "base_link", "lidar_top", time_, timeout_);
  auto tf2_transform =
    managed_tf_buffer_->getTransform<tf2::Transform>("base_link", "lidar_top", time_, timeout_);
  auto eigen_matrix =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  auto eigen_affine =
    managed_tf_buffer_->getTransform<Eigen::Affine3f>("base_link", "lidar_top", time_, timeout_);

  ASSERT_TRUE(tf_stamped.has_value());
  ASSERT_TRUE(tf2_transform.has_value());
  ASSERT_TRUE(eigen_matrix.has_value());
  ASSERT_TRUE(eigen_affine.has_value());

  // Convert tf_stamped to Eigen for comparison
  Eigen::Matrix4f expected_matrix =
    tf2::transformToEigen(tf_stamped.value()).matrix().cast<float>();

  // All should represent the same transformation
  EXPECT_TRUE(eigen_matrix.value().isApprox(expected_matrix, precision_));
  EXPECT_TRUE(eigen_affine.value().matrix().isApprox(expected_matrix, precision_));

  // Check tf2::Transform consistency
  tf2::Transform expected_tf2;
  tf2::fromMsg(tf_stamped.value().transform, expected_tf2);
  EXPECT_NEAR(tf2_transform.value().getOrigin().x(), expected_tf2.getOrigin().x(), precision_);
  EXPECT_NEAR(tf2_transform.value().getOrigin().y(), expected_tf2.getOrigin().y(), precision_);
  EXPECT_NEAR(tf2_transform.value().getOrigin().z(), expected_tf2.getOrigin().z(), precision_);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
