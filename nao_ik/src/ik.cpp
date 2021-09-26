// Copyright 2021 Kenji Brameld
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

#include "nao_ik/ik.hpp"
#include "nao_command_msgs/msg/joint_indexes.hpp"
#include "nao_ik/bhuman/ik_bhuman.hpp"

IK::IK()
: Node("IK")
{
  sub_ankle_poses =
    create_subscription<biped_interfaces::msg::AnklePoses>(
    "motion/ankle_poses", 1,
    [this](biped_interfaces::msg::AnklePoses::SharedPtr ankle_poses) {
      nao_command_msgs::msg::JointPositions joints;
      RobotDimensions rd;
      rd.yHipOffset = 0.050;
      rd.upperLegLength = 0.100;
      rd.lowerLegLength = 0.1029;
      ik_bhuman::calcLegJoints(ankle_poses->l_ankle, ankle_poses->r_ankle, joints, rd);
      pub_joints->publish(joints);
    });

  pub_joints =
    this->create_publisher<nao_command_msgs::msg::JointPositions>("effectors/joint_positions", 10);
}
