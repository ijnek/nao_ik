#include "nao_ik/bhuman/ik_bhuman.hpp"
#include "nao_ik/bhuman/Pose3f.hpp"
#include "nao_ik/bhuman/Range.hpp"
#include "nao_ik/bhuman/RobotDimensions.hpp"
#include "nao_ik/bhuman/BHMath.hpp"
#include "nao_ik/bhuman/Rotation.hpp"
#include "nao_command_msgs/msg/joint_indexes.hpp"

namespace ik_bhuman
{

static Pose3f poseToPose3f(geometry_msgs::msg::Pose pose);
static void insert(
  nao_command_msgs::msg::JointPositions & msg,
  const uint8_t & jointIndex,
  const float & jointPosition);

bool calcLegJoints(
  const geometry_msgs::msg::Pose & positionLeft, const geometry_msgs::msg::Pose & positionRight,
  nao_command_msgs::msg::JointPositions & jointAngles,
  const RobotDimensions & robotDimensions, float ratio)
{
  Pose3f pose3fLeft = poseToPose3f(positionLeft);
  Pose3f pose3fRight = poseToPose3f(positionRight);

  static const Pose3f rotPi_4 = RotationMatrix::aroundX(pi_4);
  static const Pose3f rotMinusPi_4 = RotationMatrix::aroundX(-pi_4);
  const Rangef cosClipping = Rangef::OneRange();

  Rangef::ZeroOneRange().clamp(ratio);

  const Pose3f lTarget0 =
    ((rotMinusPi_4 + Vector3f(0.f, -robotDimensions.yHipOffset, 0.f)) *= pose3fLeft) +=
    Vector3f(0.f, 0.f, robotDimensions.footHeight);
  const Pose3f rTarget0 =
    ((rotPi_4 + Vector3f(0.f, robotDimensions.yHipOffset, 0.f)) *= pose3fRight) +=
    Vector3f(0.f, 0.f, robotDimensions.footHeight);
  const Vector3f lFootToHip = lTarget0.rotation.inverse() * -lTarget0.translation;
  const Vector3f rFootToHip = rTarget0.rotation.inverse() * -rTarget0.translation;
  const float lMinusJoint5 = std::atan2(lFootToHip.y(), lFootToHip.z());
  const float rJoint5 = std::atan2(rFootToHip.y(), rFootToHip.z());
  const float lMinusBetaAndJoint4 =
    -std::atan2(lFootToHip.x(), std::sqrt(sqr(lFootToHip.y()) + sqr(lFootToHip.z())));
  const float rMinusBetaAndJoint4 =
    -std::atan2(rFootToHip.x(), std::sqrt(sqr(rFootToHip.y()) + sqr(rFootToHip.z())));
  const Vector3f lHipRotationC1 = lTarget0.rotation *
    (RotationMatrix::aroundX(-lMinusJoint5) * RotationMatrix::aroundY(-lMinusBetaAndJoint4)).col(1);
  const Vector3f rHipRotationC1 = rTarget0.rotation *
    (RotationMatrix::aroundX(-rJoint5) * RotationMatrix::aroundY(-rMinusBetaAndJoint4)).col(1);
  const float lMinusJoint0 = std::atan2(-lHipRotationC1.x(), lHipRotationC1.y());
  const float rJoint0 = std::atan2(-rHipRotationC1.x(), rHipRotationC1.y());
  const float lJoint0Combined = -lMinusJoint0 * ratio + rJoint0 * (1.f - ratio);

  const Pose3f lTarget1 = RotationMatrix::aroundZ(lJoint0Combined) * lTarget0;
  const Pose3f rTarget1 = RotationMatrix::aroundZ(-lJoint0Combined) * rTarget0;
  const Vector3f & lHipToFoot = lTarget1.translation;
  const Vector3f & rHipToFoot = rTarget1.translation;
  const float lMinusPi_4MinusJoint1 = -std::atan2(-lHipToFoot.y(), -lHipToFoot.z());
  const float rPi_4AndJoint1 = -std::atan2(-rHipToFoot.y(), -rHipToFoot.z());
  const float lJoint2MinusAlpha =
    std::atan2(
    -lHipToFoot.x(),
    std::sqrt(sqr(lHipToFoot.y()) + sqr(lHipToFoot.z())) * -sgn(lHipToFoot.z()));
  const float rJoint2MinusAlpha =
    std::atan2(
    -rHipToFoot.x(),
    std::sqrt(sqr(rHipToFoot.y()) + sqr(rHipToFoot.z())) * -sgn(rHipToFoot.z()));
  const Vector3f lFootRotationC2 = Rotation::aroundY(-lJoint2MinusAlpha) * Rotation::aroundX(
    -lMinusPi_4MinusJoint1) * lTarget1.rotation.col(2);
  const Vector3f rFootRotationC2 = Rotation::aroundY(-rJoint2MinusAlpha) * Rotation::aroundX(
    -rPi_4AndJoint1) * rTarget1.rotation.col(2);
  const float h1 = robotDimensions.upperLegLength;
  const float h2 = robotDimensions.lowerLegLength;
  const float hl = lTarget1.translation.norm();
  const float hr = rTarget1.translation.norm();
  const float h1Sqr = h1 * h1;
  const float h2Sqr = h2 * h2;
  const float hlSqr = hl * hl;
  const float hrSqr = hr * hr;
  const float lCosMinusAlpha = (h1Sqr + hlSqr - h2Sqr) / (2.f * h1 * hl);
  const float rCosMinusAlpha = (h1Sqr + hrSqr - h2Sqr) / (2.f * h1 * hr);
  const float lCosMinusBeta = (h2Sqr + hlSqr - h1Sqr) / (2.f * h2 * hl);
  const float rCosMinusBeta = (h2Sqr + hrSqr - h1Sqr) / (2.f * h2 * hr);
  const float lAlpha = -std::acos(cosClipping.limit(lCosMinusAlpha));
  const float rAlpha = -std::acos(cosClipping.limit(rCosMinusAlpha));
  const float lBeta = -std::acos(cosClipping.limit(lCosMinusBeta));
  const float rBeta = -std::acos(cosClipping.limit(rCosMinusBeta));

  insert(jointAngles, nao_command_msgs::msg::JointIndexes::LHIPYAWPITCH, lJoint0Combined);

  insert(
    jointAngles, nao_command_msgs::msg::JointIndexes::LHIPROLL,
    (lMinusPi_4MinusJoint1 + pi_4));
  insert(jointAngles, nao_command_msgs::msg::JointIndexes::LHIPPITCH, lJoint2MinusAlpha + lAlpha);
  insert(jointAngles, nao_command_msgs::msg::JointIndexes::LKNEEPITCH, -lAlpha - lBeta);
  insert(
    jointAngles, nao_command_msgs::msg::JointIndexes::LANKLEPITCH,
    std::atan2(lFootRotationC2.x(), lFootRotationC2.z()) + lBeta);
  insert(
    jointAngles, nao_command_msgs::msg::JointIndexes::LANKLEROLL,
    std::asin(-lFootRotationC2.y()));

  // insert(jointAngles, nao_command_msgs::msg::JointIndexes::RHIPYAWPITCH, lJoint0Combined);
  insert(jointAngles, nao_command_msgs::msg::JointIndexes::RHIPROLL, rPi_4AndJoint1 - pi_4);
  insert(jointAngles, nao_command_msgs::msg::JointIndexes::RHIPPITCH, rJoint2MinusAlpha + rAlpha);
  insert(jointAngles, nao_command_msgs::msg::JointIndexes::RKNEEPITCH, -rAlpha - rBeta);
  insert(
    jointAngles, nao_command_msgs::msg::JointIndexes::RANKLEPITCH,
    std::atan2(rFootRotationC2.x(), rFootRotationC2.z()) + rBeta);
  insert(
    jointAngles, nao_command_msgs::msg::JointIndexes::RANKLEROLL,
    std::asin(-rFootRotationC2.y()));
  const float maxLen = h1 + h2;
  return hl <= maxLen && hr <= maxLen;
}

static Pose3f poseToPose3f(geometry_msgs::msg::Pose pose)
{
  Quaternionf quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  RotationMatrix rot(quat);
  Vector3f trans(pose.position.x, pose.position.y, pose.position.z);
  return Pose3f(rot, trans);
}

static void insert(
  nao_command_msgs::msg::JointPositions & msg,
  const uint8_t & jointIndex,
  const float & jointPosition)
{
  msg.indexes.push_back(jointIndex);
  msg.positions.push_back(jointPosition);
}

}  // ik_bhuman
