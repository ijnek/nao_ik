# nao_ik

[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=iron)](../../actions/workflows/build_and_test_humble.yaml?query=branch:iron)
[![Build and Test (iron)](../../actions/workflows/build_and_test_iron.yaml/badge.svg?branch=iron)](../../actions/workflows/build_and_test_iron.yaml?query=branch:iron)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

This ROS2 package can be used to solve inverse kinematics of a NAO's legs joints.

## Nodes

One node is provided, which performs the invverse kinematics via publishers and subscriptions:

```sh
# If using Rolling, or J-turtle onwards
ros2 run nao_ik nao_ik

# If using I-turtle or earlier
ros2 run nao_ik ik_node
```

## Subscription
`motion/sole_poses` (*biped_interfaces/msg/SolePoses.msg*)

## Publish
`effectors/joint_positions` - (*nao_lola_command_msgs/msg/JointPositions*)

# Code Usage

Please refer to LICENSE for software licensing for different parts of the codebase.

## **Note for Robocup Teams**

**You do not have to announce B-Human code usage when using this package.**

This codebase includes software from the B-Human Code Release. Usually, when using B-Human's code in a competition,
teams are required to announce code usage to the Technical Committee one month before competition, as specified in B-Human's license.

However, B-Human has suggested to release their code in this repository with a reduced B-Human license, due to
the small amount of borrowed code. The reduced license does not include restrictions about announcing
code usage.

We thank the authors of the borrowed code and B-Human for their permissiveness in providing code under a reduced license.
