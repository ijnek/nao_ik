# Copyright 2023 Kenji Brameld
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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    nao_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('nao_state_publisher'), 'launch', 'nao_state_publisher_launch.py'])))

    nao_ik_ims = Node(package='nao_ik_ims', executable='nao_ik_ims')
    nao_ik = Node(package='nao_ik', executable='ik_node')
    nao_loopback = Node(package='nao_loopback', executable='nao_loopback')

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('nao_ik_ims'), 'rviz', 'default.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        nao_state_publisher_launch,
        nao_ik_ims,
        nao_ik,
        nao_loopback,
        rviz_node,
    ])
