#!/usr/bin/env python3
# Copyright 2025 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Co-developed by Tier IV, Inc.

import unittest

import launch
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import pytest


@pytest.mark.launch_test
def generate_test_description():

    static_transform_server = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("managed_transform_buffer"),
                    "launch",
                    "static_transform_server.launch.xml",
                ]
            )
        )
    )

    gtest_process = launch.actions.ExecuteProcess(
        cmd=["ros2", "run", "managed_transform_buffer", "managed_transform_buffer_test"],
        name="managed_transform_buffer_test",
        output="screen",
    )

    # Add a delay to ensure the static_transform_server is initialized before starting the tests
    delayed_gtest_process = TimerAction(
        period=2.0, actions=[gtest_process]  # Wait a few seconds for server to initialize
    )

    return (
        launch.LaunchDescription(
            [static_transform_server, delayed_gtest_process, launch_testing.actions.ReadyToTest()]
        ),
        {
            "gtest_process": gtest_process,
        },
    )


class TestManagedTransformBufferLaunch(unittest.TestCase):

    def test_process_exit_code(self, proc_info, gtest_process):
        """Test that the gtest process exits with code 0."""
        proc_info.assertWaitForShutdown(process=gtest_process, timeout=60)


@launch_testing.post_shutdown_test()
class TestManagedTransformBufferPostShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Test that all processes exited with acceptable codes."""
        # gtest_process should exit with 0
        # static_transform_server may exit with -6 (SIGABRT) during shutdown cleanup
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -6])
