// Copyright 2023 Kenji Brameld
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

#include "nao_ik_ims/nao_ik_ims.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"

#define FREQUENCY 1  // Hz
#define PERIOD std::chrono::microseconds(1000000 / FREQUENCY)

#define STAND_HEIGHT (0.04519 + 0.10290 + 0.100 + 0.085)
#define HIP_OFFSET_Y 0.050

namespace nao_ik_ims
{

NaoIkIms::NaoIkIms(const rclcpp::NodeOptions & options)
: rclcpp::Node{"nao_ik_ims", options}
{
  server = std::make_shared<interactive_markers::InteractiveMarkerServer>("nao_ik_ims", this);
  solePosesPub = create_publisher<biped_interfaces::msg::SolePoses>("motion/sole_poses", 1);

  solePoses.l_sole.position.y = HIP_OFFSET_Y;
  solePoses.r_sole.position.y = -HIP_OFFSET_Y;
  solePoses.l_sole.position.z = -STAND_HEIGHT;
  solePoses.r_sole.position.z = -STAND_HEIGHT;

  auto lFootInteractiveMarker = createMarker("l_foot", solePoses.l_sole);
  auto rFootInteractiveMarker = createMarker("r_foot", solePoses.r_sole);

  server->insert(
    lFootInteractiveMarker,
    std::bind(&NaoIkIms::lFootCallback, this, std::placeholders::_1));
  server->insert(
    rFootInteractiveMarker,
    std::bind(&NaoIkIms::rFootCallback, this, std::placeholders::_1));

  // 'commit' changes and send to all clients
  server->applyChanges();

  solePosesPub->publish(solePoses);
}

visualization_msgs::msg::InteractiveMarker NaoIkIms::createMarker(
  const std::string & name,
  const geometry_msgs::msg::Pose & pose)
{
  visualization_msgs::msg::InteractiveMarker interactive_marker;
  interactive_marker.header.frame_id = "base_link";
  interactive_marker.header.stamp = get_clock()->now();
  interactive_marker.name = name;
  interactive_marker.scale = 0.15;
  interactive_marker.pose = pose;

  // create a non-interactive control which contains the box
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(control);

  return interactive_marker;
}

void NaoIkIms::lFootCallback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  solePoses.l_sole = feedback->pose;
  publishSolePoses();
}

void NaoIkIms::rFootCallback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  solePoses.r_sole = feedback->pose;
  publishSolePoses();
}

void NaoIkIms::publishSolePoses()
{
  solePosesPub->publish(solePoses);
}

}  // namespace nao_ik_ims

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nao_ik_ims::NaoIkIms)
