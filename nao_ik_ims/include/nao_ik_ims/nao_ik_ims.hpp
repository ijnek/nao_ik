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

#ifndef NAO_IK_IMS__NAO_IK_IMS_HPP_
#define NAO_IK_IMS__NAO_IK_IMS_HPP_

#include <memory>

#include "biped_interfaces/msg/sole_poses.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/node.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"

namespace nao_ik_ims
{

class NaoIkIms : public rclcpp::Node
{
public:
  explicit NaoIkIms(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  rclcpp::Publisher<biped_interfaces::msg::SolePoses>::SharedPtr solePosesPub;

  visualization_msgs::msg::InteractiveMarker createMarker(const std::string& name);

  void lFootCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);
  void rFootCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  void publishSolePoses();

  biped_interfaces::msg::SolePoses solePoses;
};

}  // namespace nao_ik_ims

#endif  // NAO_IK_IMS__NAO_IK_IMS_HPP_
