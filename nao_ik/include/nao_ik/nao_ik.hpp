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

#ifndef NAO_IK__NAO_IK_HPP_
#define NAO_IK__NAO_IK_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "biped_interfaces/msg/sole_poses.hpp"

namespace nao_ik
{

class NaoIK : public rclcpp::Node
{
public:
  explicit NaoIK(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  nao_lola_command_msgs::msg::JointPositions calculate_joints(
    biped_interfaces::msg::SolePoses & sole_poses);

  rclcpp::Subscription<biped_interfaces::msg::SolePoses>::SharedPtr sub_sole_poses;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointPositions>::SharedPtr pub_joints;
};

}  // namespace nao_ik

#endif  // NAO_IK__NAO_IK_HPP_
