
#ifndef IIMT_JOINT_TRAJECTORY_ACTION_H
#define IIMT_JOINT_TRAJECTORY_ACTION_H

#include <memory>
#include <queue>
#include <thread>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "std_msgs/msg/string.hpp"

namespace iimt_ros2_trajectory_action
{

class JointTrajectoryAction : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFjt = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  JointTrajectoryAction(std::string controller_name);

private:
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr moveit_controller_pub_;
  rclcpp::Subscription<control_msgs::action::FollowJointTrajectory_Feedback>::SharedPtr fjt_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr moveit_execution_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr moveit_to_driver_pub_;

  rclcpp::TimerBase::SharedPtr watch_dog_timer_;

  bool has_active_goal_;
  bool trajectory_state_recvd_;
  trajectory_msgs::msg::JointTrajectory current_trajectory_;
  std::shared_ptr<GoalHandleFjt> active_goal_;

  void watchDogTimer();

  void fjtFeedbackCallback(const control_msgs::action::FollowJointTrajectory_Feedback::ConstSharedPtr msg);
  void moveitExecutionCallback(const std_msgs::msg::String::ConstSharedPtr msg);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleFjt> goal_handle);
  void handleAccept(const std::shared_ptr<GoalHandleFjt> goal_handle);

  void abortActiveGoal();
  void moveitControllerPublisher();
  void calculateMotionTrajectory();
  double toSec(const builtin_interfaces::msg::Duration &duration);

  bool isSimilar(std::vector<std::string> lhs, std::vector<std::string> rhs);

  trajectory_msgs::msg::JointTrajectory remapTrajectoryByJointName(trajectory_msgs::msg::JointTrajectory &trajectory);

  bool checkReachTarget(const control_msgs::action::FollowJointTrajectory_Feedback::ConstSharedPtr feedback, const trajectory_msgs::msg::JointTrajectory &traj);

  std::vector<std::string> joint_names;
};

}

#endif
