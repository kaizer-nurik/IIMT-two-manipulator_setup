#include "iimt_ros2_driver.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <functional>
#include <iomanip>
#include <sstream>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <stdexcept>

namespace iimt_ros2_driver {
static_assert(sizeof(JointRad) == 48 && sizeof(ServojParam) == 68, "Vendor SDK ABI mismatch");

template <class T> T iimtRos2Driver::parameter(const std::string &name, const T &value) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  return declare_parameter<T>(name, value, descriptor);
}

iimtRos2Driver::Joints iimtRos2Driver::joint_parameter(const std::string &name, const Joints &value,
                                                       bool positive) {
  const auto values = parameter<std::vector<double>>(name, {value.begin(), value.end()});
  if (values.size() != 6)
    throw std::invalid_argument(
        name + " must contain six values; received=" + std::to_string(values.size()));
  Joints result{};
  for (size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(values[i]) || (positive && values[i] <= 0))
      throw std::invalid_argument(
          name + "[" + std::to_string(i) + "] joint=" + names_[i] +
          " value=" + std::to_string(values[i]) +
          (positive ? "; expected finite positive value" : "; expected finite value"));
    result[i] = values[i];
  }
  return result;
}

iimtRos2Driver::iimtRos2Driver() : Node("iimt_ros2_driver") {
  try {
    configure();
  } catch (const std::exception &error) {
    throw std::invalid_argument(identity() + " configuration: " + error.what());
  }
}

iimtRos2Driver::~iimtRos2Driver() {
  stop_motion();
  disconnect();
}

void iimtRos2Driver::configure() {
  joint_prefix_ = parameter<std::string>("joint_prefix", "");
  const auto &prefix = joint_prefix_;
  if (!std::all_of(prefix.begin(), prefix.end(),
                   [](unsigned char ch) { return std::isalnum(ch) || ch == '_'; }))
    throw std::invalid_argument("joint_prefix='" + prefix +
                                "' must contain only letters, digits or underscores");
  for (auto &name : names_)
    name = prefix + name;
  ip_ = parameter<std::string>("robot_ip", "192.168.50.148");
  port_ = parameter<int>("port", 29999);
  enabled_ = parameter<bool>("enable_motion", false);
  mock_ = parameter<bool>("use_mock_hardware", false);
  rate_ = parameter<double>("command_rate", 30.0);
  advance_s_ = trajectory::advance_seconds(parameter<double>("trajectory_advance_ms", 0.0));
  read_rate_ = parameter<double>("read_rate", 30.0);
  state_timeout_ = parameter<double>("state_timeout", .2);
  read_timeout_ = parameter<double>("read_timeout", .1);
  feedback_transport_ = parameter<std::string>("feedback_transport", "sdk");
  if (feedback_transport_ != "sdk" && feedback_transport_ != "tcp")
    throw std::invalid_argument("feedback_transport='" + feedback_transport_ +
                                "'; expected sdk or tcp");
  if (!std::isfinite(read_timeout_) || read_timeout_ <= 0 || read_timeout_ > state_timeout_)
    throw std::invalid_argument("read_timeout=" + std::to_string(read_timeout_) +
                                " s must be finite, positive and <= state_timeout=" +
                                std::to_string(state_timeout_) + " s");
  max_lateness_ = parameter<double>("max_lateness", .1);
  clock_jump_tolerance_ = parameter<double>("clock_jump_tolerance", .1);
  diagnostics_period_ = parameter<double>("diagnostics_period", 1.0);
  start_tolerance_ = parameter<double>("start_tolerance", .02);
  path_tolerance_ = parameter<double>("path_tolerance", .05);
  goal_tolerance_ = parameter<double>("goal_tolerance", .003);
  goal_time_tolerance_ = parameter<double>("goal_time_tolerance", .25);
  for (const auto &name : {"command_rate", "read_rate", "state_timeout", "max_lateness",
                           "clock_jump_tolerance", "diagnostics_period", "start_tolerance",
                           "path_tolerance", "goal_tolerance", "goal_time_tolerance"}) {
    const double value = get_parameter(name).as_double();
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument(std::string(name) + "=" + std::to_string(value) +
                                  "; expected finite positive value");
  }
  if (port_ <= 0 || port_ > 65535 || rate_ < 1 || rate_ > 125 || read_rate_ < 1 || read_rate_ > 125)
    throw std::invalid_argument("port=" + std::to_string(port_) +
                                " (expected 1..65535), command_rate=" + std::to_string(rate_) +
                                " Hz, read_rate=" + std::to_string(read_rate_) +
                                " Hz (supported rate range 1..125 Hz)");
  if (!mock_ && get_parameter("use_sim_time").as_bool())
    throw std::invalid_argument("Real hardware cannot use simulated time");
  period_ = 1.0 / rate_;
  const double configured_period = parameter<double>("servo.period", period_);
  if (!std::isfinite(configured_period) || std::abs(configured_period - period_) > 1e-9)
    throw std::invalid_argument("servo.period=" + std::to_string(configured_period) +
                                " s must equal 1 / command_rate=" + std::to_string(period_) + " s");
  servo_.t = period_;
  servo_.a = parameter<double>("servo.a", 7.0);
  servo_.v = parameter<double>("servo.v", 3.0);
  servo_.lookahead_time = parameter<double>("servo.lookahead_time", .012);
  servo_.gain = parameter<double>("servo.gain", 0.0);
  if (!std::isfinite(servo_.a) || servo_.a <= 0 || !std::isfinite(servo_.v) || servo_.v <= 0 ||
      !std::isfinite(servo_.lookahead_time) || servo_.lookahead_time < 0 ||
      !std::isfinite(servo_.gain) || servo_.gain < 0)
    throw std::invalid_argument("Invalid ServoJ parameters: a=" + std::to_string(servo_.a) +
                                ", v=" + std::to_string(servo_.v) +
                                ", lookahead_time=" + std::to_string(servo_.lookahead_time) +
                                ", gain=" + std::to_string(servo_.gain) +
                                "; all must be finite, a/v > 0, lookahead_time/gain >= 0");
  minimum_ = joint_parameter("limits.min_position",
                             {-6.2832, -2.6167, -2.8783, -3.0528, -3.0528, -6.2832});
  maximum_ =
      joint_parameter("limits.max_position", {6.2832, 2.6167, 2.8783, 3.0528, 3.0528, 6.2832});
  max_velocity_ = joint_parameter("limits.max_velocity", {1, 1, 1, 1, 1, 1}, true);
  max_acceleration_ = joint_parameter("limits.max_acceleration", {1.875, 2, 2, 2, 2, 2}, true);
  max_step_ = joint_parameter("limits.max_step", {.02, .02, .02, .02, .02, .02}, true);
  for (size_t i = 0; i < 6; ++i)
    if (minimum_[i] >= maximum_[i])
      throw std::invalid_argument("Inverted joint bounds: joint=" + names_[i] +
                                  " min_rad=" + std::to_string(minimum_[i]) +
                                  " max_rad=" + std::to_string(maximum_[i]));
}

bool iimtRos2Driver::connect() {
  if (mock_)
    return connected_ = true;
  if (feedback_transport_ == "sdk")
    state_login_ = API_Login(ip_.c_str(), port_, state_handle_);
  if (state_login_ || feedback_transport_ == "tcp")
    control_login_ = API_Login(ip_.c_str(), port_, control_handle_);
  if (control_login_)
    servo_login_ = API_Login(ip_.c_str(), port_, servo_handle_);
  connected_ = (state_login_ || feedback_transport_ == "tcp") && control_login_ && servo_login_;
  if (!connected_) {
    RCLCPP_ERROR(get_logger(),
                 "%s API_Login failed (state=%d control=%d servo=%d feedback_transport=%s); "
                 "SDK returns only success/failure, not a joint-specific cause",
                 identity().c_str(), state_login_, control_login_, servo_login_,
                 feedback_transport_.c_str());
    disconnect();
  }
  return connected_;
}

void iimtRos2Driver::disconnect() {
  stop_reader();
  joint_reader_.close();
  auto logout = [this](JTRHD &handle, bool &logged_in, const char *channel) {
    if (logged_in && !API_Logout(handle))
      RCLCPP_ERROR(get_logger(), "%s API_Logout failed: channel=%s", identity().c_str(), channel);
    logged_in = false;
  };
  logout(servo_handle_, servo_login_, "servo");
  logout(control_handle_, control_login_, "control");
  logout(state_handle_, state_login_, "state");
  connected_ = false;
}

bool iimtRos2Driver::start() {
  states_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
  diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 10);
  trajectory_timing_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "~/trajectory_timing", rclcpp::QoS(1).reliable().transient_local());
  if (!connect())
    return false;
  // Hardware startup does not wait for feedback; goals require a fresh cached sample.
  if (mock_)
    read_state();
  else
    reader_ = std::thread(&iimtRos2Driver::read_loop, this);
  action_ = rclcpp_action::create_server<Action>(
      this, "iimt_group_controller/follow_joint_trajectory",
      std::bind(&iimtRos2Driver::goal_requested, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&iimtRos2Driver::cancel_requested, this, std::placeholders::_1),
      std::bind(&iimtRos2Driver::goal_accepted, this, std::placeholders::_1));
  last_tick_ = timer_epoch_ = diagnostic_time_ = Clock::now();
  timer_ = create_wall_timer(std::chrono::duration<double>(period_),
                             std::bind(&iimtRos2Driver::tick, this));
  RCLCPP_INFO(get_logger(),
              "Direct ServoJ: %.3f Hz, motion=%s, mock=%s. No power/brake commands. SDK calls are "
              "blocking; no hard realtime guarantee.",
              rate_, enabled_ ? "enabled" : "disabled", mock_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "Feedback transport=%s, read_timeout=%.6f s (%s)",
              feedback_transport_.c_str(), read_timeout_,
              feedback_transport_ == "tcp" ? "whole socket transaction deadline"
                                           : "inactive: SDK read is blocking");
  if (!mock_ && feedback_transport_ == "sdk")
    RCLCPP_WARN(get_logger(),
                "SDK reader has no interruptible timeout; shutdown may wait for read");
  return true;
}

double iimtRos2Driver::seconds(Time now, Time before) {
  return std::chrono::duration<double>(now - before).count();
}

void iimtRos2Driver::check_position(const Joints &q) const {
  for (size_t i = 0; i < 6; ++i)
    if (!std::isfinite(q[i]) || q[i] < minimum_[i] || q[i] > maximum_[i])
      throw std::invalid_argument(
          "Position outside limits: joint=" + names_[i] + " sdk_axis=" + std::to_string(i + 1) +
          " position_rad=" + std::to_string(q[i]) + " min_rad=" + std::to_string(minimum_[i]) +
          " max_rad=" + std::to_string(maximum_[i]));
}

std::vector<iimtRos2Driver::Point>
iimtRos2Driver::parse_trajectory(const Action::Goal &goal) const {
  const auto &trajectory = goal.trajectory;
  const auto order = trajectory::joint_order(names_, trajectory.joint_names);
  if (!goal.multi_dof_trajectory.points.empty() || !goal.multi_dof_trajectory.joint_names.empty())
    throw std::invalid_argument("Multi-DOF goals are unsupported");
  std::vector<Point> result;
  for (size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto &input = trajectory.points[index];
    if (input.positions.size() != 6 ||
        (!input.velocities.empty() && input.velocities.size() != 6) ||
        (!input.accelerations.empty() && input.accelerations.size() != 6) ||
        (!input.accelerations.empty() && input.velocities.empty()) || !input.effort.empty())
      throw std::invalid_argument("point[" + std::to_string(index) +
                                  "]: positions=" + std::to_string(input.positions.size()) +
                                  ", velocities=" + std::to_string(input.velocities.size()) +
                                  ", accelerations=" + std::to_string(input.accelerations.size()) +
                                  ", effort=" + std::to_string(input.effort.size()) +
                                  "; six positions required; derivatives must be empty or six "
                                  "values, acceleration requires velocity, effort unsupported");
    Point point;
    point.time = rclcpp::Duration(input.time_from_start).seconds();
    point.derivatives = input.accelerations.empty() ? (input.velocities.empty() ? 0 : 1) : 2;
    std::copy(input.positions.begin(), input.positions.end(), point.position.begin());
    if (point.derivatives >= 1)
      std::copy(input.velocities.begin(), input.velocities.end(), point.velocity.begin());
    if (point.derivatives == 2)
      std::copy(input.accelerations.begin(), input.accelerations.end(), point.acceleration.begin());
    point = trajectory::reorder(point, order);
    try {
      check_position(point.position);
    } catch (const std::exception &error) {
      throw std::invalid_argument("point[" + std::to_string(index) +
                                  "] t=" + std::to_string(point.time) + " s: " + error.what());
    }
    result.push_back(point);
  }
  try {
    trajectory::validate(result);
    trajectory::validate_endpoint(result.back(), max_acceleration_);
  } catch (const std::exception &error) {
    std::string axes = "; SDK axes=[";
    for (size_t i = 0; i < names_.size(); ++i)
      axes += (i ? ", " : "") + std::to_string(i + 1) + "=" + names_[i];
    throw std::invalid_argument(std::string(error.what()) + axes + "]");
  }
  if (result.front().time == 0) {
    for (size_t i = 0; i < 6; ++i)
      if (std::abs(result.front().position[i] - actual_[i]) > start_tolerance_)
        throw std::invalid_argument("Start position mismatch: " +
                                    joint_context(i, result.front().position[i], start_tolerance_) +
                                    "; " + state_context());
  } else {
    Point first;
    first.position = actual_;
    first.derivatives = result.front().derivatives;
    result.insert(result.begin(), first);
  }
  return result;
}

iimtRos2Driver::Joints
iimtRos2Driver::parse_tolerances(const std::vector<control_msgs::msg::JointTolerance> &input,
                                 double fallback) const {
  Joints values{};
  values.fill(fallback);
  std::vector<std::string> seen;
  for (const auto &tolerance : input) {
    const auto joint = std::find(names_.begin(), names_.end(), tolerance.name);
    if (joint == names_.end() || std::find(seen.begin(), seen.end(), tolerance.name) != seen.end())
      throw std::invalid_argument("Unknown/duplicate tolerance joint='" + tolerance.name + "'");
    if (!std::isfinite(tolerance.position) || tolerance.position < 0 || tolerance.velocity != 0 ||
        tolerance.acceleration != 0)
      throw std::invalid_argument(
          "joint=" + tolerance.name +
          " position_tolerance_rad=" + std::to_string(tolerance.position) +
          " velocity_tolerance=" + std::to_string(tolerance.velocity) +
          " acceleration_tolerance=" + std::to_string(tolerance.acceleration) +
          "; only finite nonnegative position tolerances supported; zero uses default");
    if (tolerance.position > 0)
      values[std::distance(names_.begin(), joint)] = tolerance.position;
    seen.push_back(tolerance.name);
  }
  return values;
}

rclcpp_action::GoalResponse
iimtRos2Driver::goal_requested(const rclcpp_action::GoalUUID &,
                               std::shared_ptr<const Action::Goal> goal) {
  const auto receipt_stamp = now();
  try {
    // Rejected concurrent goals must not consume the active goal's feedback generation.
    if (!enabled_)
      throw std::invalid_argument("Motion disabled: enable_motion=false");
    if (!fault_.empty())
      throw std::invalid_argument("Driver fault latched: " + fault_);
    if (active_ || pending_goal_)
      throw std::invalid_argument(
          "Another goal owns commands: active=" + std::to_string(bool(active_)) +
          " pending=" + std::to_string(pending_goal_));
    if (!mock_)
      consume_state();
    if (!fault_.empty())
      throw std::invalid_argument("Feedback fault: " + fault_);
    if (!have_state_ || seconds(Clock::now(), state_time_) > state_timeout_ ||
        state_read_duration_ > state_timeout_)
      throw std::invalid_argument("No fresh joint state or last read exceeded state_timeout; " +
                                  state_context());
    const rclcpp::Time stamp(goal->trajectory.header.stamp);
    if (stamp.nanoseconds() && (stamp - now()).seconds() < advance_s_)
      throw std::invalid_argument(
          "Trajectory start timestamp leaves insufficient advance pre-roll: stamp_ns=" +
          std::to_string(stamp.nanoseconds()) + " remaining_s=" +
          std::to_string((stamp - now()).seconds()) + " required_s=" + std::to_string(advance_s_));
    if (rclcpp::Duration(goal->goal_time_tolerance).seconds() < 0)
      throw std::invalid_argument(
          "Negative goal_time_tolerance_s=" +
          std::to_string(rclcpp::Duration(goal->goal_time_tolerance).seconds()));
    parse_trajectory(*goal);
    parse_tolerances(goal->path_tolerance, path_tolerance_);
    parse_tolerances(goal->goal_tolerance, goal_tolerance_);
    receipt_stamp_ = receipt_stamp;
    pending_goal_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "%s Reject trajectory: %s", identity().c_str(), error.what());
    return rclcpp_action::GoalResponse::REJECT;
  }
}

void iimtRos2Driver::goal_accepted(const std::shared_ptr<GoalHandle> goal) {
  active_ = goal;
  pending_goal_ = false;
  start_time_ = Time{}; // No valid nominal clock until this goal has been parsed.
  goal_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  try {
    points_ = parse_trajectory(*goal->get_goal());
    path_bounds_ = parse_tolerances(goal->get_goal()->path_tolerance, path_tolerance_);
    goal_bounds_ = parse_tolerances(goal->get_goal()->goal_tolerance, goal_tolerance_);
    goal_margin_ = rclcpp::Duration(goal->get_goal()->goal_time_tolerance).seconds();
    if (goal_margin_ == 0)
      goal_margin_ = goal_time_tolerance_;
    goal_stamp_ = rclcpp::Time(goal->get_goal()->trajectory.header.stamp);
    const auto ros_now = now();
    start_on_dispatch_ = !goal_stamp_.nanoseconds();
    if (start_on_dispatch_)
      goal_stamp_ = ros_now + rclcpp::Duration::from_seconds(advance_s_);
    else if ((goal_stamp_ - ros_now).seconds() < advance_s_)
      throw std::invalid_argument(
          "Trajectory start timestamp no longer allows full advance pre-roll: stamp_ns=" +
          std::to_string(goal_stamp_.nanoseconds()) +
          " remaining_s=" + std::to_string((goal_stamp_ - ros_now).seconds()) +
          " required_s=" + std::to_string(advance_s_));
    start_time_ =
        Clock::now() + std::chrono::duration_cast<Clock::duration>(
                           std::chrono::duration<double>((goal_stamp_ - ros_now).seconds()));
    first_command_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    publish_trajectory_timing();
    have_send_ = false;
    if (advance_s_ > 0)
      RCLCPP_WARN(get_logger(),
                  "Motion starts %.3f ms BEFORE nominal trajectory t=0; "
                  "feedback reference and deadline remain unshifted",
                  advance_s_ * 1000.);
    RCLCPP_INFO(get_logger(), "Accepted %zu points, nominal duration %.6f s (no retiming)",
                points_.size(), points_.back().time);
  } catch (const std::exception &error) {
    finish(Action::Result::INVALID_GOAL, error.what());
  }
}

rclcpp_action::CancelResponse
iimtRos2Driver::cancel_requested(const std::shared_ptr<GoalHandle> goal) {
  return active_ == goal ? rclcpp_action::CancelResponse::ACCEPT
                         : rclcpp_action::CancelResponse::REJECT;
}

// Diagnostic formatting runs only on rejection/failure/completion, never per ServoJ sample.
std::string iimtRos2Driver::identity() const {
  std::string arm = joint_prefix_;
  while (!arm.empty() && arm.back() == '_')
    arm.pop_back();
  return "arm=" + (arm.empty() ? std::string("single") : arm) +
         " node=" + get_fully_qualified_name() + " robot=" + ip_ + ":" + std::to_string(port_);
}

std::string iimtRos2Driver::state_context() const {
  std::ostringstream out;
  out << std::fixed << std::setprecision(6);
  out << "feedback_available=" << (have_state_ ? "true" : "false");
  if (have_state_) {
    const double age = seconds(Clock::now(), state_time_);
    out << " feedback_stamp_ns=" << state_stamp_.nanoseconds() << " feedback_age_s=" << age
        << " feedback_fresh="
        << (age <= state_timeout_ && state_read_duration_ <= state_timeout_ ? "true" : "false");
  } else {
    out << " feedback_stamp_ns=unavailable feedback_age_s=unavailable feedback_fresh=false";
  }
  out << " read_duration_s=" << state_read_duration_ << " state_timeout_s=" << state_timeout_
      << " transport=" << feedback_transport_;
  return out.str();
}

std::string iimtRos2Driver::joint_context(size_t axis, double desired, double tolerance) const {
  std::ostringstream out;
  out << std::fixed << std::setprecision(6);
  out << "joint=" << names_[axis] << " sdk_axis=" << axis + 1 << " desired_rad=";
  if (std::isfinite(desired))
    out << desired;
  else
    out << "unavailable";
  out << " actual_rad=";
  if (have_state_) {
    out << actual_[axis];
    if (std::isfinite(desired))
      out << " error_rad=" << desired - actual_[axis]
          << " abs_error_rad=" << std::abs(desired - actual_[axis]);
  } else {
    out << "unavailable error_rad=unavailable";
  }
  if (std::isfinite(tolerance))
    out << " tolerance_rad=" << tolerance;
  return out.str();
}

std::string iimtRos2Driver::execution_context() const {
  std::ostringstream out;
  out << std::fixed << std::setprecision(6) << state_context();
  if (active_) {
    out << " goal_uuid=";
    for (const auto byte : active_->get_goal_id())
      out << "0123456789abcdef"[byte >> 4] << "0123456789abcdef"[byte & 15];
  }
  const bool reference_available = have_state_ && !points_.empty() && start_time_ != Time{};
  Joints desired{};
  desired.fill(NAN);
  if (reference_available) {
    const double measured_elapsed = seconds(state_time_, start_time_);
    desired = trajectory::sample(points_, measured_elapsed).position;
    out << " reference=nominal_at_feedback_receipt_time"
        << " nominal_start_stamp_ns=" << goal_stamp_.nanoseconds()
        << " measured_elapsed_s=" << measured_elapsed
        << " execution_elapsed_s=" << seconds(Clock::now(), start_time_)
        << " duration_s=" << points_.back().time << " goal_margin_s=" << goal_margin_
        << " deadline_elapsed_s=" << points_.back().time + goal_margin_
        << " trajectory_advance_ms=" << advance_s_ * 1000.;
  } else {
    out << " reference=unavailable";
  }
  out << "; snapshot (error=desired-actual, angles=rad; receipt time is NOT robot acquisition "
         "time)=[";
  for (size_t i = 0; i < names_.size(); ++i) {
    out << (i ? "; " : "") << joint_context(i, desired[i]);
    if (reference_available)
      out << " path_tolerance_rad=" << path_bounds_[i]
          << " goal_desired_rad=" << points_.back().position[i]
          << " goal_error_rad=" << points_.back().position[i] - actual_[i]
          << " goal_tolerance_rad=" << goal_bounds_[i];
    if (have_send_)
      out << " last_sent_rad=" << last_command_[i];
  }
  out << "]";
  return out.str();
}

bool iimtRos2Driver::stop_motion() {
  if (!motion_sent_)
    return true;
  bool stopped = mock_ || API_Stop(control_handle_);
  bool cleared = mock_ || API_ClearUnfinishedTrajectory(control_handle_);
  if (!stopped || !cleared) {
    fault_ = std::string("SDK Stop/Clear failed; Stop=") + (stopped ? "true" : "false") +
             " Clear=" + (cleared ? "true" : "false") +
             "; physical stop is NOT confirmed; joint_cause=not_reported_by_sdk";
    RCLCPP_ERROR(get_logger(), "%s %s", identity().c_str(), fault_.c_str());
    return false;
  }
  motion_sent_ = false;
  return true;
}

void iimtRos2Driver::finish(int code, const std::string &message, bool canceled) {
  if (!active_)
    return;
  // Freeze evidence BEFORE blocking Stop/Clear and before either can replace fault_.
  const std::string original =
      "original_code=" + std::to_string(code) + " original_reason=" + message;
  const std::string context = execution_context();
  const bool stopped = stop_motion();
  auto result = std::make_shared<Action::Result>();
  result->error_code = stopped ? code : Action::Result::PATH_TOLERANCE_VIOLATED;
  result->error_string =
      code == Action::Result::SUCCESSFUL && stopped ? message : identity() + "; " + original;
  if (!stopped)
    result->error_string += "; stop_failure=" + fault_;
  if (!stopped || code != Action::Result::SUCCESSFUL)
    result->error_string += "; " + context;
  if (!stopped)
    fault_ = result->error_string;
  if (canceled && stopped)
    active_->canceled(result);
  else if (code == Action::Result::SUCCESSFUL && stopped)
    active_->succeed(result);
  else
    active_->abort(result);
  if (result->error_code != Action::Result::SUCCESSFUL)
    RCLCPP_ERROR(get_logger(), "Trajectory ended: %s", result->error_string.c_str());
  active_.reset();
  have_send_ = false;
  points_.clear();
}

void iimtRos2Driver::fail(const std::string &message) {
  // A motion fault disables commands, not telemetry. Never resume the goal automatically.
  if (fault_.empty())
    fault_ = identity() + "; " + message + "; " + execution_context();
  RCLCPP_ERROR(get_logger(), "%s %s", identity().c_str(), message.c_str());
  finish(Action::Result::PATH_TOLERANCE_VIOLATED, message);
}

// I/O happens outside the cache mutex. Only this worker accesses the feedback connection.
void iimtRos2Driver::read_loop() noexcept {
  const auto interval =
      std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / read_rate_));
  auto next = Clock::now();
  try {
    for (;;) {
      {
        std::unique_lock<std::mutex> lock(feedback_mutex_);
        if (reader_wake_.wait_until(lock, next, [this] { return reader_stop_; }))
          return;
      }
      Joints measured{};
      const auto begin = Clock::now();
      JointReader::Result result;
      if (feedback_transport_ == "tcp") {
        result = joint_reader_.read(ip_, port_, read_timeout_, measured);
      } else {
        JointRad state{};
        result = API_GetRobotJointRad(state_handle_, &state) ? JointReader::Result::success
                                                             : JointReader::Result::error;
        std::copy(std::begin(state.robotRad), std::end(state.robotRad), measured.begin());
      }
      const auto received = Clock::now();
      const auto stamp = now();
      const double duration = seconds(received, begin);
      std::string error;
      if (result == JointReader::Result::error)
        error =
            feedback_transport_ == "tcp"
                ? "TCP joint read: " + joint_reader_.error()
                : "API_GetRobotJointRad returned false; duration_s=" + std::to_string(duration) +
                      "; SDK does not report a joint-specific cause";
      if (result == JointReader::Result::success)
        check_position(measured);
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        ++feedback_.calls;
        feedback_.last_duration = duration;
        feedback_.max_duration = std::max(feedback_.max_duration, duration);
        if (result == JointReader::Result::timeout)
          ++feedback_.timeouts;
        else if (result == JointReader::Result::success) {
          feedback_.position = measured;
          feedback_.received = received;
          feedback_.stamp = stamp;
          feedback_.sample_duration = duration;
          ++feedback_.generation;
        }
        feedback_.error = error;
      }
      if (!error.empty())
        return;
      // Skip missed slots; never burst queued reads after a slow response.
      next += interval;
      if (next <= received)
        next += interval * ((received - next) / interval + 1);
    }
  } catch (const std::exception &error) {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_.error = std::string("Feedback worker: ") + error.what();
  } catch (...) {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_.error = "Unknown feedback worker failure";
  }
}

void iimtRos2Driver::stop_reader() {
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    reader_stop_ = true;
  }
  reader_wake_.notify_all();
  if (reader_.joinable())
    reader_.join();
}

// Executor alone owns actions, current state, diagnostics and ROS publication.
bool iimtRos2Driver::consume_state() {
  FeedbackSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    snapshot = feedback_;
  }
  reads_ = snapshot.calls;
  last_read_ = snapshot.last_duration;
  max_read_ = snapshot.max_duration;
  if (snapshot.timeouts > read_timeouts_) {
    read_timeouts_ = snapshot.timeouts;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s Joint read timed out (budget %.6f s, last_duration %.6f s, total %lu); "
        "retaining last sample without refreshing its timestamp; %s",
        identity().c_str(), read_timeout_, snapshot.last_duration,
        static_cast<unsigned long>(read_timeouts_), state_context().c_str());
  }
  if (!snapshot.error.empty()) {
    if (fault_.empty())
      fail(snapshot.error);
    return false;
  }
  if (snapshot.generation == consumed_generation_)
    return false;
  consumed_generation_ = snapshot.generation;
  state_read_duration_ = snapshot.sample_duration;
  if (active_ && snapshot.sample_duration > state_timeout_) {
    fail("Joint read exceeded state_timeout (detected after return): read_duration_s=" +
         std::to_string(snapshot.sample_duration) + " state_timeout_s=" +
         std::to_string(state_timeout_) + "; returned sample was not accepted");
    return false;
  }
  accept_state(snapshot.position, snapshot.received, snapshot.stamp);
  return true;
}

bool iimtRos2Driver::read_state() {
  // Mock feedback stays on the executor: no concurrent access to simulated commands.
  ++reads_;
  accept_state(have_send_ ? last_command_ : actual_, Clock::now(), now());
  return true;
}

void iimtRos2Driver::accept_state(const Joints &measured, Time received,
                                  const rclcpp::Time &stamp) {
  if (have_state_) {
    const double dt = seconds(received, state_time_);
    for (size_t i = 0; i < 6; ++i)
      velocity_[i] = dt > 0 ? (measured[i] - actual_[i]) / dt : 0;
  }
  actual_ = measured;
  state_time_ = received;
  state_stamp_ = stamp;
  have_state_ = true;
  sensor_msgs::msg::JointState message;
  message.header.stamp = state_stamp_;
  message.name = names_;
  message.position.assign(actual_.begin(), actual_.end());
  message.velocity.assign(velocity_.begin(), velocity_.end());
  states_->publish(message);
}

bool iimtRos2Driver::send_command(const Point &point, Time current) {
  try {
    check_position(point.position);
    const double dt = have_send_ ? seconds(current, send_time_) : period_;
    const Joints &previous = have_send_ ? last_command_ : actual_;
    for (size_t i = 0; i < 6; ++i) {
      const double step = std::abs(point.position[i] - previous[i]);
      if (dt <= 0 || step > max_step_[i] || step / dt > max_velocity_[i] ||
          std::abs(point.velocity[i]) > max_velocity_[i] ||
          std::abs(point.acceleration[i]) > max_acceleration_[i])
        throw std::invalid_argument(
            "Command limit exceeded (requested command, not nominal feedback reference): " +
            joint_context(i, point.position[i]) +
            " previous_command_rad=" + std::to_string(previous[i]) + " dt_s=" + std::to_string(dt) +
            " step_rad=" + std::to_string(step) + " max_step_rad=" + std::to_string(max_step_[i]) +
            " step_velocity_rad_s=" + std::to_string(dt > 0 ? step / dt : NAN) +
            " requested_velocity_rad_s=" + std::to_string(point.velocity[i]) +
            " max_velocity_rad_s=" + std::to_string(max_velocity_[i]) +
            " requested_acceleration_rad_s2=" + std::to_string(point.acceleration[i]) +
            " max_acceleration_rad_s2=" + std::to_string(max_acceleration_[i]));
      servo_.q[i] = point.position[i];
    }
  } catch (const std::exception &error) {
    fail(error.what());
    return false;
  }
  const auto begin = Clock::now();
  // Даже неуспешный SDK-вызов мог частично передать команду: нужен Stop.
  motion_sent_ = true;
  const auto command_stamp = now();
  const bool ok = mock_ || API_Servoj(servo_handle_, &servo_);
  last_send_ = seconds(Clock::now(), begin);
  max_send_ = std::max(max_send_, last_send_);
  if (!ok) {
    std::string message = "API_Servoj failed (returned false); joint_cause=not_reported_by_sdk; "
                          "partial transmission possible; duration_s=" +
                          std::to_string(last_send_) + "; attempted_command=[";
    for (size_t i = 0; i < names_.size(); ++i)
      message += (i ? "; " : "") + joint_context(i, point.position[i]);
    fail(message + "]");
    return false;
  }
  if (!have_send_) {
    first_command_stamp_ = command_stamp;
    publish_trajectory_timing();
  }
  ++sends_;
  if (have_send_)
    last_interval_ = seconds(begin, send_time_);
  send_time_ = begin;
  have_send_ = true;
  last_command_ = point.position;
  warn_timing("ServoJ call", last_send_, send_overruns_);
  return true;
}

void iimtRos2Driver::execute(Time current, bool new_state) {
  if (active_->is_canceling()) {
    finish(Action::Result::SUCCESSFUL, "Canceled", true);
    return;
  }
  // An immediate goal starts at its first dispatch tick, not action acceptance.
  // This keeps the first q[0] and the following step on the same absolute clock.
  if (start_on_dispatch_) {
    goal_stamp_ = now() + rclcpp::Duration::from_seconds(advance_s_);
    start_time_ = current + std::chrono::duration_cast<Clock::duration>(
                                std::chrono::duration<double>(advance_s_));
    start_on_dispatch_ = false;
  }
  const double elapsed = seconds(current, start_time_);
  if (std::abs((now() - goal_stamp_).seconds() - elapsed) > clock_jump_tolerance_) {
    fail("ROS clock diverged from steady trajectory clock: ros_elapsed_s=" +
         std::to_string((now() - goal_stamp_).seconds()) + " steady_elapsed_s=" +
         std::to_string(elapsed) + " tolerance_s=" + std::to_string(clock_jump_tolerance_));
    return;
  }
  // Mock has no reader thread; keep its stationary feedback alive before a future start.
  if (mock_ && elapsed < 0)
    read_state();
  if (!have_state_ || seconds(current, state_time_) > state_timeout_) {
    fail("Stale joint state before command; " + state_context());
    return;
  }
  const double dispatch_elapsed = trajectory::command_time(elapsed, advance_s_);
  if (dispatch_elapsed < 0)
    return;
  if (elapsed > points_.back().time + goal_margin_) {
    // A fresh measurement taken before the deadline may have arrived in this late tick.
    // Evaluate it before rejecting, but never send another command after the deadline.
    if (evaluate_state(new_state))
      return;
    finish(Action::Result::GOAL_TOLERANCE_VIOLATED, "Missed endpoint deadline");
    return;
  }
  if (!have_send_)
    warn_timing("Trajectory start", dispatch_elapsed, start_overruns_);
  // Абсолютное время, а не счётчик отправленных точек: пропуск не растягивает путь.
  // Explicit future starts retain their requested clock; late samples use normal limits.
  const auto desired = trajectory::sample(points_, dispatch_elapsed);
  if (!send_command(desired, current))
    return;
  if (mock_)
    new_state = read_state();
  warn_timing("Command dispatch cycle", seconds(Clock::now(), current), cycle_overruns_);
  evaluate_state(new_state);
}

bool iimtRos2Driver::evaluate_state(bool new_state) {
  if (!new_state)
    return false;
  const double measured_elapsed = seconds(state_time_, start_time_);
  const auto reference = trajectory::sample(points_, measured_elapsed);
  // Pre-roll is checked against q[0], but omitted from the nonnegative CSV time axis.
  if (measured_elapsed >= 0)
    publish_feedback(reference, measured_elapsed);
  bool arrived = true;
  for (size_t i = 0; i < 6; ++i) {
    if (std::abs(reference.position[i] - actual_[i]) > path_bounds_[i]) {
      finish(Action::Result::PATH_TOLERANCE_VIOLATED,
             "Path position tolerance exceeded: " +
                 joint_context(i, reference.position[i], path_bounds_[i]));
      return true;
    }
    arrived = arrived && std::abs(points_.back().position[i] - actual_[i]) <= goal_bounds_[i];
  }
  // Геометрическое совпадение до конца назначенного времени не означает успех.
  if (measured_elapsed > points_.back().time + goal_margin_)
    finish(Action::Result::GOAL_TOLERANCE_VIOLATED,
           "Endpoint measurement arrived after goal deadline");
  else if (measured_elapsed >= points_.back().time && arrived)
    finish(Action::Result::SUCCESSFUL, "Reached measured endpoint after nominal duration");
  return !active_;
}

// Плановое время — мягкий бюджет. Свежесть состояния, пределы и endpoint остаются строгими.
void iimtRos2Driver::warn_timing(const char *stage, double duration, uint64_t &counter) {
  if (duration <= period_ + max_lateness_)
    return;
  ++counter;
  max_timing_overrun_ = std::max(max_timing_overrun_, duration - period_);
  const auto current = Clock::now();
  if (timing_warning_time_ != Time{} && seconds(current, timing_warning_time_) < 1.0)
    return;
  timing_warning_time_ = current;
  RCLCPP_WARN(get_logger(),
              "%s took %.6f s (nominal %.6f s, soft allowance %.6f s); "
              "continuing on absolute trajectory time, without replaying missed samples",
              stage, duration, period_, max_lateness_);
}

void iimtRos2Driver::tick() {
  const auto current = Clock::now();
  const double interval = seconds(current, last_tick_);
  last_tick_ = current;
  // Абсолютные слоты сохраняют дробный остаток: 60 мс при 30 Гц не теряются при округлении.
  const auto slot = static_cast<uint64_t>(seconds(current, timer_epoch_) / period_);
  if (slot > last_timer_slot_ + 1)
    missed_slots_ += slot - last_timer_slot_ - 1;
  last_timer_slot_ = slot;
  // Keep receiving valid measurements even while motion is latched off.
  const bool new_state = mock_ ? false : consume_state();
  if (fault_.empty()) {
    warn_timing("Timer dispatch", interval, dispatch_overruns_);
    if (fault_.empty() && active_)
      execute(Clock::now(), new_state);
    else if (fault_.empty() && mock_)
      read_state();
  }
  publish_diagnostics(Clock::now());
}

void iimtRos2Driver::publish_feedback(const Point &desired, double elapsed) {
  if (!active_)
    return;
  auto feedback = std::make_shared<Action::Feedback>();
  feedback->header.stamp = state_stamp_;
  feedback->joint_names = names_;
  feedback->desired.positions.assign(desired.position.begin(), desired.position.end());
  feedback->desired.velocities.assign(desired.velocity.begin(), desired.velocity.end());
  feedback->desired.accelerations.assign(desired.acceleration.begin(), desired.acceleration.end());
  feedback->actual.positions.assign(actual_.begin(), actual_.end());
  feedback->actual.velocities.assign(velocity_.begin(), velocity_.end());
  feedback->error.positions.resize(6);
  for (size_t i = 0; i < 6; ++i)
    feedback->error.positions[i] = desired.position[i] - actual_[i];
  feedback->desired.time_from_start = rclcpp::Duration::from_seconds(std::max(0.0, elapsed));
  feedback->actual.time_from_start = feedback->desired.time_from_start;
  active_->publish_feedback(feedback);
}

// Per-goal clock evidence, latched for subscribers that join after acceptance.
void iimtRos2Driver::publish_trajectory_timing() {
  if (!active_)
    return;
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "trajectory_timing";
  status.hardware_id = mock_ ? "mock" : ip_;
  status.message =
      "Host receipt, nominal start and first ServoJ invocation (not robot acquisition)";
  const auto add = [&status](const std::string &key, const std::string &value) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = value;
    status.values.push_back(entry);
  };
  std::string uuid;
  for (const auto byte : active_->get_goal_id()) {
    uuid += "0123456789abcdef"[byte >> 4];
    uuid += "0123456789abcdef"[byte & 15];
  }
  add("goal_uuid", uuid);
  add("receipt_stamp_ns", std::to_string(receipt_stamp_.nanoseconds()));
  add("nominal_start_stamp_ns", std::to_string(goal_stamp_.nanoseconds()));
  add("first_command_stamp_ns", std::to_string(first_command_stamp_.nanoseconds()));
  add("nominal_start_final", start_on_dispatch_ ? "false" : "true");
  add("trajectory_advance_ms", std::to_string(advance_s_ * 1000.));
  message.status.push_back(status);
  trajectory_timing_->publish(message);
}

void iimtRos2Driver::publish_diagnostics(Time current) {
  if (seconds(current, diagnostic_time_) < diagnostics_period_)
    return;
  diagnostic_time_ = current;
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_name();
  status.hardware_id = mock_ ? "mock" : ip_;
  const bool timing_warning = read_timeouts_ > 0 || max_read_ > period_ || max_timing_overrun_ > 0;
  status.level = fault_.empty() ? (timing_warning ? 1 : 0) : 2;
  status.message = !fault_.empty() ? fault_
                   : timing_warning
                       ? "Observed timing overruns; best-effort execution, no retiming"
                       : "Best-effort command timer and feedback reader; no realtime guarantee";
  auto add = [&status](const std::string &key, const std::string &value) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = value;
    status.values.push_back(entry);
  };
  add("enable_motion", enabled_ ? "true" : "false");
  add("use_mock_hardware", mock_ ? "true" : "false");
  add("active_goal", active_ ? "true" : "false");
  add("stop_pending", motion_sent_ ? "true" : "false");
  add("feedback_transport", feedback_transport_);
  add("read_timeout_s", std::to_string(read_timeout_));
  add("read_timeouts", std::to_string(read_timeouts_));
  add("have_state", have_state_ ? "true" : "false");
  add("servo_a", std::to_string(servo_.a));
  add("servo_v", std::to_string(servo_.v));
  add("servo_period_s", std::to_string(servo_.t));
  add("servo_lookahead_time_s", std::to_string(servo_.lookahead_time));
  add("servo_gain", std::to_string(servo_.gain));
  add("trajectory_advance_ms", std::to_string(advance_s_ * 1000.));
  add("nominal_start_stamp_ns", std::to_string(goal_stamp_.nanoseconds()));
  add("command_rate", std::to_string(rate_));
  add("read_rate", std::to_string(read_rate_));
  add("feedback_generation", std::to_string(consumed_generation_));
  add("soft_lateness_allowance_s", std::to_string(max_lateness_));
  add("hard_state_timeout_s", std::to_string(state_timeout_));
  add("hard_clock_jump_tolerance_s", std::to_string(clock_jump_tolerance_));
  add("timer_dispatch_overruns", std::to_string(dispatch_overruns_));
  add("trajectory_start_overruns", std::to_string(start_overruns_));
  add("servoj_call_overruns", std::to_string(send_overruns_));
  add("sdk_cycle_overruns", std::to_string(cycle_overruns_));
  add("max_timing_overrun_s", std::to_string(max_timing_overrun_));
  add("sent", std::to_string(sends_));
  add("read_calls", std::to_string(reads_));
  add("missed_timer_slots", std::to_string(missed_slots_));
  add("last_read_duration_s", std::to_string(last_read_));
  add("max_read_duration_s", std::to_string(max_read_));
  add("last_servoj_duration_s", std::to_string(last_send_));
  add("max_servoj_duration_s", std::to_string(max_send_));
  add("last_send_interval_s", std::to_string(last_interval_));
  add("feedback_age_s", have_state_ ? std::to_string(seconds(current, state_time_)) : "unknown");
  message.status.push_back(status);
  diagnostics_->publish(message);
}
} // namespace iimt_ros2_driver
