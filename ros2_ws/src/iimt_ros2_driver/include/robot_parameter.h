#ifndef ROS2_METATYPE_H_
#define ROS2_METATYPE_H_
#include "math.h"

const int ARM_DOF = 6;
const double MAX_JOINT_ACC = 10.0/180.0*M_PI;
const double MAX_JOINT_VEL = 10.0/180.0*M_PI;
const int RECONNECT_RATE = 500;
const int UPDATE_RATE_ = 100;
const int MINIMUM_BUFFER_SIZE = 300;
const std::string joint_name_[ARM_DOF] = {"shoulder_joint","upper_arm_joint","forearm_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};

namespace iimt_ros2_driver
{
  struct PlanningState
  {
    double joint_vel_[ARM_DOF];
    double joint_acc_[ARM_DOF];
    double joint_pos_[ARM_DOF];
  };

  struct ArmStopped
  {
    bool user_stopped;
    bool collision_stopped;
    bool emergency_stopped;
    bool protective_stopped;
    bool singularity_stopped;
  };
}

#endif
