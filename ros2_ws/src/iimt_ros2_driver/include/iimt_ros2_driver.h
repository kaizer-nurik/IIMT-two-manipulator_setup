#ifndef IIMT_ROS2_DRIVER_H_
#define IIMT_ROS2_DRIVER_H_

#include "trajectory.hpp"
#include "joint_reader.hpp"
#pragma pack(push)
#include "robotapi.h"
#pragma pack(pop)
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <cstdint>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>

namespace iimt_ros2_driver {
// Executor sends commands; one reader owns feedback I/O and shares only a cached snapshot.
class iimtRos2Driver : public rclcpp::Node {
public:
  iimtRos2Driver();
  ~iimtRos2Driver() override;
  bool start();

private:
  using Action = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using Point = trajectory::Point;
  using Joints = trajectory::Joints;
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  template <class T> T parameter(const std::string &name, const T &value);
  Joints joint_parameter(const std::string &name, const Joints &value, bool positive = false);
  void configure();
  bool connect();
  void disconnect();
  bool stop_motion();
  bool read_state();
  void read_loop() noexcept;
  void stop_reader();
  bool consume_state();
  void accept_state(const Joints &measured, Time received, const rclcpp::Time &stamp);
  bool send_command(const Point &point, Time now);
  void warn_timing(const char *stage, double duration, uint64_t &counter);
  void tick();
  void execute(Time now, bool new_state);
  bool evaluate_state(bool new_state);
  void publish_feedback(const Point &desired, double elapsed);
  void publish_diagnostics(Time now);
  void publish_trajectory_timing();
  std::string identity() const;
  std::string state_context() const;
  std::string joint_context(size_t axis, double desired, double tolerance = NAN) const;
  std::string execution_context() const;
  void fail(const std::string &message);
  void finish(int code, const std::string &message, bool canceled = false);
  std::vector<Point> parse_trajectory(const Action::Goal &goal) const;
  Joints parse_tolerances(const std::vector<control_msgs::msg::JointTolerance> &input,
                          double fallback) const;
  void check_position(const Joints &q) const;
  rclcpp_action::GoalResponse goal_requested(const rclcpp_action::GoalUUID &,
                                             std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse cancel_requested(const std::shared_ptr<GoalHandle> goal);
  void goal_accepted(const std::shared_ptr<GoalHandle> goal);
  static double seconds(Time now, Time before);

  std::vector<std::string> names_{"shoulder_joint", "upper_arm_joint", "forearm_joint",
                                  "wrist_1_joint",  "wrist_2_joint",   "wrist_3_joint"};
  bool mock_{false}, enabled_{false}, connected_{false}, pending_goal_{false}, motion_sent_{false},
      have_state_{false}, have_send_{false};
  std::string ip_, fault_, feedback_transport_, joint_prefix_;
  JointReader joint_reader_;
  int port_{29999};
  double read_timeout_{.1}, read_rate_{30};
  struct FeedbackSnapshot {
    Joints position{};
    Time received{};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    uint64_t generation{0}, calls{0}, timeouts{0};
    double last_duration{0}, max_duration{0}, sample_duration{0};
    std::string error;
  };
  FeedbackSnapshot feedback_;
  std::mutex feedback_mutex_;
  std::condition_variable reader_wake_;
  std::thread reader_;
  bool reader_stop_{false};
  uint64_t consumed_generation_{0};
  bool start_on_dispatch_{false};
  double advance_s_{0};
  rclcpp::Time receipt_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time first_command_stamp_{0, 0, RCL_ROS_TIME};
  double rate_{30}, period_{1.0 / 30}, state_timeout_{.2}, max_lateness_{.1},
      clock_jump_tolerance_{.1}, diagnostics_period_{1};
  double start_tolerance_{.02}, path_tolerance_{.05}, goal_tolerance_{.003},
      goal_time_tolerance_{.25}, goal_margin_{.25};
  Joints minimum_{}, maximum_{}, max_velocity_{}, max_acceleration_{}, max_step_{}, actual_{},
      velocity_{}, last_command_{}, path_bounds_{}, goal_bounds_{};
  ServojParam servo_{};
  JTRHD state_handle_{}, control_handle_{}, servo_handle_{};
  bool state_login_{false}, control_login_{false}, servo_login_{false};
  Time state_time_{}, send_time_{}, start_time_{}, last_tick_{}, timer_epoch_{}, diagnostic_time_{},
      timing_warning_time_{};
  rclcpp::Time state_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time goal_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<Point> points_;
  std::shared_ptr<GoalHandle> active_;
  rclcpp_action::Server<Action>::SharedPtr action_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr states_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_,
      trajectory_timing_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t read_timeouts_{0};
  uint64_t sends_{0}, reads_{0}, missed_slots_{0}, last_timer_slot_{0};
  uint64_t dispatch_overruns_{0}, start_overruns_{0}, send_overruns_{0}, cycle_overruns_{0};
  double max_timing_overrun_{0};
  double state_read_duration_{0}, last_read_{0}, max_read_{0}, last_send_{0}, max_send_{0},
      last_interval_{0};
};
} // namespace iimt_ros2_driver
#endif
