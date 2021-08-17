#ifndef nao_ik__IK_HPP_
#define nao_ik__IK_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nao_command_msgs/msg/joint_positions.hpp"
#include "nao_ik_interfaces/msg/ik_command.hpp"

struct Pose3f;
struct RobotDimensions;

class IK : public rclcpp::Node
{
public:
  IK();

private:
  nao_command_msgs::msg::JointPositions calculate_joints(
    nao_ik_interfaces::msg::IKCommand & ik_command);

  rclcpp::Subscription<nao_ik_interfaces::msg::IKCommand>::SharedPtr sub_ik_command;
  rclcpp::Publisher<nao_command_msgs::msg::JointPositions>::SharedPtr pub_joints;

  /**
   * This method calculates the joint angles for the legs of the robot from a Pose3f for each leg.
   * @param positionLeft The desired position (translation + rotation) of the left foots ankle point.
   * @param positionRight The desired position (translation + rotation) of the right foots ankle point.
   * @param jointAngles The instance of JointAngles where the resulting joint angles are written into.
   * @param robotDimensions The Robot Dimensions needed for calculation.
   * @param ratio The ratio between the left and right yaw angle.
   * @return Whether the target position was reachable or not (if the given target position is not reachable the computation proceeds using the closest reachable position near the target).
   */
  bool calcLegJoints(
    const Pose3f & positionLeft, const Pose3f & positionRight, nao_command_msgs::msg::JointPositions & jointAngles,
    const RobotDimensions & robotDimensions, float ratio = 0.5f);
};

#endif  // nao_ik__IK_HPP_