#include "geometry_msgs/msg/pose.hpp"
#include "nao_command_msgs/msg/joint_positions.hpp"
#include "nao_ik/bhuman/RobotDimensions.hpp"

namespace ik_bhuman
{
/**
 * This method calculates the joint angles for the legs of the robot from a Pose3f for each leg.
 * @param positionLeft The desired position (translation + rotation) of the left foots sole point.
 * @param positionRight The desired position (translation + rotation) of the right foots sole point.
 * @param jointAngles The instance of JointAngles where the resulting joint angles are written into.
 * @param robotDimensions The Robot Dimensions needed for calculation.
 * @param ratio The ratio between the left and right yaw angle.
 * @return Whether the target position was reachable or not (if the given target position is not reachable the computation proceeds using the closest reachable position near the target).
 */
bool calcLegJoints(
  const geometry_msgs::msg::Pose & positionLeft, const geometry_msgs::msg::Pose & positionRight,
  nao_command_msgs::msg::JointPositions & jointAngles,
  const RobotDimensions & robotDimensions, float ratio = 0.5f);
}  // namespace ik_bhuman
