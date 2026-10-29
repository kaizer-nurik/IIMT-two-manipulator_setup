#include "iimt_ros2_control/system.hpp"
#include "fake_sdk.hpp"
#include <rcutils/logging.h>
#include <cstdarg>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;
using iimt_ros2_control::Clock;
using Callback = hardware_interface::CallbackReturn;
using Return = hardware_interface::return_type;
namespace {
void require(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
bool wait_until(const std::function<bool()> &predicate,
                std::chrono::milliseconds timeout = 2000ms) {
  const auto deadline = Clock::now() + timeout;
  while (Clock::now() < deadline) {
    if (predicate())
      return true;
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}
// Observe synchronous lifecycle logs without capturing unrelated SDK-worker logs.
// Install once before creating worker threads; restore the process handler on exit.
struct ErrorLogCapture {
  const char *callback;
  bool observed{false};
};
thread_local ErrorLogCapture *active_capture = nullptr;
rcutils_logging_output_handler_t previous_log_handler = nullptr;
void observe_log(const rcutils_log_location_t *location, int severity, const char *name,
                 rcutils_time_point_value_t timestamp, const char *format, va_list *args) {
  if (active_capture && severity == RCUTILS_LOG_SEVERITY_ERROR) {
    char message[2048];
    va_list copy;
    va_copy(copy, *args);
    std::vsnprintf(message, sizeof(message), format, copy);
    va_end(copy);
    if (std::strstr(message, active_capture->callback))
      active_capture->observed = true;
  }
  if (previous_log_handler)
    previous_log_handler(location, severity, name, timestamp, format, args);
}
struct LogObserver {
  LogObserver() {
    previous_log_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(observe_log);
  }
  ~LogObserver() {
    rcutils_logging_set_output_handler(previous_log_handler);
  }
};
void require_callback_error_log(const char *callback, const std::function<Callback()> &invoke) {
  ErrorLogCapture capture{callback};
  struct ResetCapture {
    ~ResetCapture() {
      active_capture = nullptr;
    }
  } reset;
  active_capture = &capture;
  require(invoke() == Callback::ERROR, "invalid lifecycle transition did not return ERROR");
  require(capture.observed, "Callback::ERROR omitted an ERROR log naming its lifecycle callback");
}
hardware_interface::HardwareInfo hardware_info() {
  hardware_interface::HardwareInfo info;
  info.name = "iimt_test";
  info.type = "system";
  for (const auto *name : {"shoulder_joint", "upper_arm_joint", "forearm_joint", "wrist_1_joint",
                           "wrist_2_joint", "wrist_3_joint"}) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    hardware_interface::InterfaceInfo position;
    position.name = "position";
    hardware_interface::InterfaceInfo velocity;
    velocity.name = "velocity";
    joint.command_interfaces.push_back(position);
    joint.state_interfaces = {position, velocity};
    info.joints.push_back(joint);
  }
  return info;
}
struct Fixture {
  std::string yaml;
  iimt_ros2_control::IimtSystem system;
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<std::string> keys;
  rclcpp_lifecycle::State state{0, "test"};
  bool cleaned{false};
  int period_ms;
  Fixture(const std::string &sdk, bool motion, int cycle_ms = 20, const std::string &prefix = "",
          const std::string &ip = "192.0.2.50", const std::string &group = "",
          double sdk_budget = 0.1, int read_delay_ms = 0)
      : period_ms(cycle_ms) {
    if (prefix.empty()) {
      fake_sdk_reset();
      fake_sdk_set_feedback(0.01);
      fake_sdk_set_read_delay(read_delay_ms);
    }
    char path[] = "/tmp/iimt-system-test-XXXXXX";
    int fd = mkstemp(path);
    require(fd >= 0, "cannot create test parameter file");
    close(fd);
    yaml = path;
    std::ofstream out(yaml);
    out << "/**:\n  ros__parameters:\n"
        << "    servo.period: " << period_ms / 1000.0
        << "\n    state_period: " << period_ms / 1000.0 << "\n"
        << "    sdk_budget: " << sdk_budget << "\n    max_lateness: 0.2\n"
        << "    command_jitter_tolerance: " << period_ms / 2000.0 << "\n"
        << "    command_timeout: 0.3\n    state_timeout: 0.3\n"
        << "    startup_timeout: 2.0\n    diagnostics_period: 0.02\n"
        << "    worker_priority: 0\n    controller_cancel_service: ''\n";
    out.close();
    auto info = hardware_info();
    info.hardware_parameters = {{"hardware_parameters_file", yaml},
                                {"robot_ip", ip},
                                {"sdk_library_path", sdk},
                                {"enable_motion", motion ? "true" : "false"},
                                {"controller_update_rate", std::to_string(1000 / period_ms)}};
    if (!prefix.empty()) {
      info.name = prefix + "hardware";
      info.hardware_parameters["joint_prefix"] = prefix;
      info.hardware_parameters["hardware_node_name"] = prefix + "iimt_hardware";
      info.hardware_parameters["coordination_group"] = group;
      for (auto &joint : info.joints)
        joint.name = prefix + joint.name;
    }
    for (const auto &joint : info.joints)
      keys.push_back(joint.name + "/position");
    require(system.on_init(info) == Callback::SUCCESS, "on_init failed");
    commands = system.export_command_interfaces();
    require(system.on_configure(state) == Callback::SUCCESS, "on_configure failed");
    require(system.on_activate(state) == Callback::SUCCESS, "on_activate failed");
  }
  ~Fixture() {
    if (!cleaned)
      system.on_cleanup(state);
    if (!yaml.empty())
      std::remove(yaml.c_str());
  }
  void cleanup() {
    system.on_cleanup(state);
    cleaned = true;
  }
  void claim() {
    require(system.prepare_command_mode_switch(keys, {}) == Return::OK, "complete claim rejected");
    require(system.perform_command_mode_switch(keys, {}) == Return::OK, "complete claim failed");
  }
  Return write(Clock::time_point source_stamp = Clock::now()) {
    const auto ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(source_stamp.time_since_epoch())
            .count();
    return system.write(rclcpp::Time(ns, RCL_STEADY_TIME),
                        rclcpp::Duration(0, period_ms * 1000000));
  }
  Return read() {
    return system.read(rclcpp::Time(0, 0, RCL_STEADY_TIME),
                       rclcpp::Duration(0, period_ms * 1000000));
  }
  void live_hold(int count = 8) {
    const auto start = Clock::now();
    for (int tick = 1; tick <= count; ++tick) {
      std::this_thread::sleep_until(start + std::chrono::milliseconds(period_ms * tick));
      const int sent = fake_sdk_snapshot().sends;
      require(write() == Return::OK, "live hold write faulted");
      require(wait_until([&] { return fake_sdk_snapshot().sends > sent; }, 100ms),
              "worker did not emit fresh hold command");
    }
  }
};
void smooth_motion(Fixture &f, int phase_amplitude_ms) {
  // A 0.2 rad / 2 s quintic has peak velocity 0.1875 rad/s and
  // peak acceleration < 0.289 rad/s^2, well below unchanged default limits.
  // Modulate writer phase without changing trajectory time; limits stay unchanged.
  const auto start = Clock::now();
  double maximum_sent = 0.01;
  for (int tick = 1; tick <= 2200 / f.period_ms; ++tick) {
    const int phase_ms = (tick / 10) % 2 ? phase_amplitude_ms : -phase_amplitude_ms;
    std::this_thread::sleep_until(start + std::chrono::milliseconds(f.period_ms * tick) +
                                  std::chrono::milliseconds(phase_ms));
    const auto source_stamp = Clock::now();
    const double elapsed = std::chrono::duration<double>(source_stamp - start).count();
    const double u = std::clamp(elapsed / 2.0, 0.0, 1.0);
    const double q = 0.01 + 0.2 * u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
    f.commands[0].set_value(q);
    if (f.write(source_stamp) != Return::OK || f.read() != Return::OK) {
      throw std::runtime_error("smooth bounded trajectory faulted at t=" + std::to_string(elapsed) +
                               " s, requested J1=" + std::to_string(q) +
                               ", last sent J1=" + std::to_string(fake_sdk_snapshot().sent.q[0]));
    }
    maximum_sent = std::max(maximum_sent, fake_sdk_snapshot().sent.q[0]);
  }
  const auto sent = fake_sdk_snapshot();
  require(maximum_sent >= 0.209, "worker did not execute meaningful 0.2 rad excursion");
  require(std::abs(sent.feedback.robotRad[0] - 0.21) < 1e-6, "timed motion did not reach endpoint");
  require(sent.stops == 0, "smooth motion triggered a stop");
}
} // namespace
int main(int argc, char **argv) {
  if (argc < 2 || argc > 3) {
    std::cerr << "fake SDK path and optional case-name filter required\n";
    return 1;
  }
  const std::string sdk = argv[1];
  const std::string filter = argc == 3 ? argv[2] : "";
  rclcpp::init(argc, argv);
  LogObserver log_observer;
  using Test = std::pair<const char *, std::function<void()>>;
  const std::vector<Test> tests{
      {"on_init invalid joint count logs ERROR before diagnostic node exists",
       [&] {
         auto info = hardware_info();
         info.joints.pop_back();
         iimt_ros2_control::IimtSystem system;
         require_callback_error_log("on_init", [&] { return system.on_init(info); });
       }},
      {"on_init invalid SDK joint order logs ERROR",
       [&] {
         auto info = hardware_info();
         std::swap(info.joints[0], info.joints[1]);
         iimt_ros2_control::IimtSystem system;
         require_callback_error_log("on_init", [&] { return system.on_init(info); });
       }},
      {"on_init invalid command interface logs ERROR",
       [&] {
         auto info = hardware_info();
         info.joints[0].command_interfaces[0].name = "velocity";
         iimt_ros2_control::IimtSystem system;
         require_callback_error_log("on_init", [&] { return system.on_init(info); });
       }},
      {"on_init missing velocity state logs ERROR",
       [&] {
         auto info = hardware_info();
         info.joints[0].state_interfaces[1].name = "effort";
         iimt_ros2_control::IimtSystem system;
         require_callback_error_log("on_init", [&] { return system.on_init(info); });
       }},
      {"claimed startup waits for first write without independent feedback polls",
       [&] {
         Fixture f(sdk, true, 50, "", "192.0.2.50", "", 0.025, 15);
         f.claim();
         // Configure has just published its initial read; the next independent
         // poll would be due at 50 ms, before this intentionally delayed write.
         const int reads = fake_sdk_snapshot().reads;
         std::this_thread::sleep_for(75ms);
         require(fake_sdk_snapshot().reads == reads,
                 "claimed startup launched a feedback read before first command");
         require(f.read() == Return::OK, "waiting for first write faulted prematurely");
         f.live_hold(3);
         require(fake_sdk_snapshot().reads > reads, "post-send feedback did not resume");
         require(f.read() == Return::OK, "normal delayed-read command stream faulted");
       }},
      {"SDK budget unclaimed slow reads remain observable without motion calls",
       [&] {
         for (bool motion_enabled : {false, true}) {
           Fixture f(sdk, motion_enabled, 50, "", "192.0.2.50", "", 0.01, 15);
           auto observer = std::make_shared<rclcpp::Node>("sdk_budget_observer");
           diagnostic_msgs::msg::DiagnosticStatus diagnostic;
           bool received = false;
           auto subscription = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
               "/iimt_hardware/diagnostics", 10,
               [&](const diagnostic_msgs::msg::DiagnosticArray &message) {
                 for (const auto &status : message.status)
                   if (status.name == "iimt_hardware") {
                     diagnostic = status;
                     received = true;
                   }
               });
           const int initial_reads = fake_sdk_snapshot().reads;
           require(wait_until([&] {
                     rclcpp::spin_some(observer);
                     return received && fake_sdk_snapshot().reads >= initial_reads + 3;
                   }),
                   "slow unclaimed polling/diagnostic publication stopped");
           require(diagnostic.level == diagnostic_msgs::msg::DiagnosticStatus::WARN,
                   "unclaimed SDK latency overrun was not reported as WARN");
           auto metric = [&](const std::string &key) {
             auto it = std::find_if(diagnostic.values.begin(), diagnostic.values.end(),
                                    [&](const auto &item) { return item.key == key; });
             require(it != diagnostic.values.end(), ("missing diagnostic metric " + key).c_str());
             return std::stod(it->value);
           };
           require(metric("last_read_duration_s") > .01 && metric("max_read_duration_s") > .01,
                   "slow read wall time absent from per-operation diagnostics");
           require(metric("read_calls") > 0 && metric("read_budget_overruns") > 0 &&
                       metric("unclaimed_read_budget_overruns") > 0 && metric("servoj_calls") == 0,
                   "unclaimed read/overrun counters or ServoJ isolation are wrong");
           require(f.read() == Return::OK,
                   "successful unclaimed read over motion budget latched a fault");
           const auto measured = f.system.export_state_interfaces();
           auto q = std::find_if(measured.begin(), measured.end(), [](const auto &v) {
             return v.get_name() == "shoulder_joint/position";
           });
           require(q != measured.end() && std::abs(q->get_value() - .01) < 1e-12,
                   "successful slow measurement was not exposed");
           f.cleanup();
           const auto result = fake_sdk_snapshot();
           require(result.sends == 0 && result.stops == 0 && result.clears == 0,
                   "unclaimed latency measurement issued a motion-affecting SDK call");
         }
       }},
      {"SDK budget armed slow read still latches fault and requests Stop",
       [&] {
         Fixture f(sdk, true, 50, "", "192.0.2.50", "", 0.01);
         f.claim();
         fake_sdk_set_read_delay(15);
         auto next_write = Clock::now();
         require(wait_until([&] {
                   if (f.read() == Return::ERROR)
                     return true;
                   if (Clock::now() >= next_write) {
                     if (f.write() != Return::OK)
                       return true;
                     next_write += 50ms;
                   }
                   return false;
                 }),
                 "armed SDK read over budget did not fault despite live controller heartbeat");
         require(wait_until([&] { return fake_sdk_snapshot().stops >= 1; }),
                 "armed over-budget read did not request Stop");
         const int sent = fake_sdk_snapshot().sends;
         require(f.write() == Return::ERROR, "armed budget fault allowed resumed commands");
         require(fake_sdk_snapshot().sends == sent, "commands emitted after budget fault");
       }},
      {"SDK budget claim during an idle blocked read cannot bypass armed enforcement",
       [&] {
         Fixture f(sdk, true, 50, "", "192.0.2.50", "", 0.01);
         struct ReleaseRead {
           ~ReleaseRead() {
             fake_sdk_block_read(false);
           }
         } release;
         const int reads = fake_sdk_snapshot().reads;
         fake_sdk_block_read(true);
         require(wait_until([&] { return fake_sdk_snapshot().reads > reads; }),
                 "worker did not enter unclaimed blocked SDK read");
         // A controlled in-call stall, not a retry: exceed the 10 ms budget while
         // the query is known to be in flight, then claim before it returns.
         std::this_thread::sleep_for(15ms);
         f.claim();
         fake_sdk_block_read(false);
         require(wait_until([&] { return f.read() == Return::ERROR; }, 100ms),
                 "claim acquired during slow read escaped armed budget enforcement");
         require(wait_until([&] { return fake_sdk_snapshot().stops >= 1; }),
                 "mid-read claim fault did not request Stop");
         require(fake_sdk_snapshot().sends == 0,
                 "mid-read claim emitted ServoJ after timing fault");
       }},
      {"SDK budget idle read failure is fatal rather than a latency warning",
       [&] {
         Fixture f(sdk, false, 50, "", "192.0.2.50", "", 0.01);
         fake_sdk_set_read_delay(15);
         fake_sdk_set_read_ok(false);
         require(wait_until([&] { return f.read() == Return::ERROR; }),
                 "failed idle SDK read was treated as a successful timing overrun");
         f.cleanup();
         const auto result = fake_sdk_snapshot();
         require(result.sends == 0 && result.stops == 0 && result.clears == 0,
                 "unclaimed failed read interfered with robot motion");
       }},
      {"dual hardware preserves independent six-axis feedback, commands and diagnostic nodes",
       [&] {
         fake_sdk_reset();
         const std::array<double, 6> left_q{.01, .02, .03, .04, .05, .06};
         const std::array<double, 6> right_q{-.01, -.02, -.03, -.04, -.05, -.06};
         fake_sdk_robot_feedback("192.0.2.51", left_q);
         fake_sdk_robot_feedback("192.0.2.52", right_q);
         Fixture left(sdk, true, 20, "left_", "192.0.2.51", "dual_routing");
         Fixture right(sdk, true, 20, "right_", "192.0.2.52", "dual_routing");
         auto observer = std::make_shared<rclcpp::Node>("dual_test_observer");
         require(
             wait_until([&] {
               auto names = observer->get_node_names();
               return std::find(names.begin(), names.end(), "/left_iimt_hardware") != names.end() &&
                      std::find(names.begin(), names.end(), "/right_iimt_hardware") != names.end();
             }),
             "distinct hardware parameter nodes missing");
         auto ls = left.system.export_state_interfaces(),
              rs = right.system.export_state_interfaces();
         require(left.read() == Return::OK && right.read() == Return::OK,
                 "dual feedback read failed");
         for (int i = 0; i < 6; ++i) {
           require(left.commands[i].get_name() == left.keys[i],
                   "left command joint prefix missing");
           require(right.commands[i].get_name() == right.keys[i],
                   "right command joint prefix missing");
           require(std::abs(left.commands[i].get_value() - left_q[i]) < 1e-12,
                   "left activation axis routing wrong");
           require(std::abs(right.commands[i].get_value() - right_q[i]) < 1e-12,
                   "right activation axis routing wrong");
         }
         // Named state interfaces avoid imposing an internal export ordering.
         for (int i = 0; i < 6; ++i) {
           auto check = [&](const auto &interfaces, const std::string &key, double value) {
             auto it = std::find_if(interfaces.begin(), interfaces.end(),
                                    [&](const auto &v) { return v.get_name() == key; });
             require(it != interfaces.end() && std::abs(it->get_value() - value) < 1e-12,
                     "measured feedback crossed robot/axis");
           };
           check(ls, left.keys[i], left_q[i]);
           check(rs, right.keys[i], right_q[i]);
         }
         left.claim();
         right.claim();
         require(left.write() == Return::OK && right.write() == Return::OK, "dual write failed");
         require(wait_until([&] {
                   return fake_sdk_robot_snapshot("192.0.2.51").sends == 1 &&
                          fake_sdk_robot_snapshot("192.0.2.52").sends == 1;
                 }),
                 "dual command not delivered");
         const auto l = fake_sdk_robot_snapshot("192.0.2.51"),
                    r = fake_sdk_robot_snapshot("192.0.2.52");
         for (int i = 0; i < 6; ++i) {
           require(std::abs(l.sent.q[i] - left_q[i]) < 1e-12, "left ServoJ routed wrong");
           require(std::abs(r.sent.q[i] - right_q[i]) < 1e-12, "right ServoJ routed wrong");
         }
         require(l.send_handle != r.send_handle, "both robots share SDK handle");
         require(fake_sdk_peak_concurrent_calls() == 1,
                 "vendor API calls overlapped between robots");
       }},
      {"dual 125Hz moving paths share source time without crossing robot commands",
       [&] {
         fake_sdk_reset();
         const std::string left_ip = "192.0.2.51", right_ip = "192.0.2.52";
         const std::array<double, 6> left_base{.01, .02, .03, .04, .05, .06};
         const std::array<double, 6> right_base{-.01, -.02, -.03, -.04, -.05, -.06};
         const std::array<double, 6> left_delta{.12, .10, -.11, .10, -.12, .11};
         const std::array<double, 6> right_delta{.10, -.12, .11, -.10, .12, -.11};
         fake_sdk_robot_feedback(left_ip, left_base);
         fake_sdk_robot_feedback(right_ip, right_base);
         fake_sdk_robot_follow_commands(left_ip, true);
         fake_sdk_robot_follow_commands(right_ip, true);
         Fixture left(sdk, true, 8, "left_", left_ip, "dual_moving_125hz");
         Fixture right(sdk, true, 8, "right_", right_ip, "dual_moving_125hz");
         left.claim();
         right.claim();
         // Both six-axis paths use the same two-second quintic phase. The largest
         // displacement is 0.12 rad: peak v=0.1125 rad/s, peak a<0.174 rad/s^2.
         // Do not enlarge the production position/velocity/acceleration guards.
         const auto start = Clock::now();
         constexpr int ticks = 275; // 2 s movement plus 0.2 s endpoint hold.
         for (int tick = 1; tick <= ticks; ++tick) {
           std::this_thread::sleep_until(start + 8ms * tick);
           const auto source = Clock::now();
           const double elapsed = std::chrono::duration<double>(source - start).count();
           const double u = std::clamp(elapsed / 2.0, 0.0, 1.0);
           const double phase = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
           for (int axis = 0; axis < 6; ++axis) {
             left.commands[axis].set_value(left_base[axis] + left_delta[axis] * phase);
             right.commands[axis].set_value(right_base[axis] + right_delta[axis] * phase);
           }
           if (left.write(source) != Return::OK || right.write(source) != Return::OK ||
               left.read() != Return::OK || right.read() != Return::OK)
             throw std::runtime_error(
                 "dual bounded moving path faulted at t=" + std::to_string(elapsed) +
                 " s; left sends=" + std::to_string(fake_sdk_robot_snapshot(left_ip).sends) +
                 ", right sends=" + std::to_string(fake_sdk_robot_snapshot(right_ip).sends));
         }
         require(wait_until(
                     [&] {
                       return fake_sdk_robot_snapshot(left_ip).sends == ticks &&
                              fake_sdk_robot_snapshot(right_ip).sends == ticks;
                     },
                     100ms),
                 "dual motion skipped or repeated a controller sample");
         const auto l = fake_sdk_robot_snapshot(left_ip), r = fake_sdk_robot_snapshot(right_ip);
         require(left.read() == Return::OK && right.read() == Return::OK, "dual endpoint faulted");
         require(l.stops == 0 && r.stops == 0, "dual smooth paths triggered Stop");
         for (int axis = 0; axis < 6; ++axis) {
           require(std::abs(l.sent.q[axis] - left_base[axis] - left_delta[axis]) < 1e-6 &&
                       std::abs(l.feedback.robotRad[axis] - l.sent.q[axis]) < 1e-12,
                   "left moving SDK command/feedback endpoint or axis routing wrong");
           require(std::abs(r.sent.q[axis] - right_base[axis] - right_delta[axis]) < 1e-6 &&
                       std::abs(r.feedback.robotRad[axis] - r.sent.q[axis]) < 1e-12,
                   "right moving SDK command/feedback endpoint or axis routing wrong");
         }
         require(fake_sdk_peak_concurrent_calls() == 1,
                 "dual moving paths overlapped vendor API calls");
       }},
      {"dual peer feedback fault latches both arms and blocks subsequent motion",
       [&] {
         fake_sdk_reset();
         Fixture left(sdk, true, 20, "left_", "192.0.2.51", "dual_fault");
         Fixture right(sdk, true, 20, "right_", "192.0.2.52", "dual_fault");
         fake_sdk_robot_read_ok("192.0.2.51", false);
         require(wait_until(
                     [&] { return left.read() == Return::ERROR && right.read() == Return::ERROR; }),
                 "peer failure did not latch both hardware components");
         const auto l = fake_sdk_robot_snapshot("192.0.2.51").sends;
         const auto r = fake_sdk_robot_snapshot("192.0.2.52").sends;
         fake_sdk_robot_read_ok("192.0.2.51", true);
         require(left.write() == Return::ERROR && right.write() == Return::ERROR,
                 "peer fault recovered automatically");
         require(left.system.perform_command_mode_switch(left.keys, {}) == Return::ERROR &&
                     right.system.perform_command_mode_switch(right.keys, {}) == Return::ERROR,
                 "faulted pair accepted claim");
         require(fake_sdk_robot_snapshot("192.0.2.51").sends == l &&
                     fake_sdk_robot_snapshot("192.0.2.52").sends == r,
                 "pair sent after fault latch");
       }},
      {"dual blocked shared SDK gate faults armed pair without delivering queued commands",
       [&] {
         fake_sdk_reset();
         Fixture left(sdk, true, 20, "left_", "192.0.2.51", "dual_gate_timeout");
         Fixture right(sdk, true, 20, "right_", "192.0.2.52", "dual_gate_timeout");
         const int reads = fake_sdk_robot_snapshot("192.0.2.51").reads +
                           fake_sdk_robot_snapshot("192.0.2.52").reads;
         struct ReleaseRead {
           ~ReleaseRead() {
             fake_sdk_block_read(false);
           }
         } release;
         fake_sdk_block_read(true);
         require(wait_until([&] {
                   return fake_sdk_robot_snapshot("192.0.2.51").reads +
                              fake_sdk_robot_snapshot("192.0.2.52").reads >
                          reads;
                 }),
                 "fake SDK failed to enter blocked read");
         // Claimed hardware polls only after a command. Enter the idle read
         // first, then claim while the SDK gate is held, without sending a
         // warm-up command that would weaken the zero-delivery assertion below.
         left.claim();
         right.claim();
         // One SDK owner is now blocked while the other worker cannot enter.
         // A command accepted before the watchdog fires must never be replayed later.
         left.write();
         right.write();
         require(wait_until(
                     [&] { return left.read() == Return::ERROR && right.read() == Return::ERROR; }),
                 "shared gate contention failed to latch both arms");
         fake_sdk_block_read(false);
         require(wait_until([&] {
                   return fake_sdk_robot_snapshot("192.0.2.51").stops > 0 &&
                          fake_sdk_robot_snapshot("192.0.2.52").stops > 0;
                 }),
                 "both armed robots did not attempt stop after gate released");
         require(fake_sdk_robot_snapshot("192.0.2.51").sends == 0 &&
                     fake_sdk_robot_snapshot("192.0.2.52").sends == 0,
                 "expired pending command sent after gate reopened");
         require(fake_sdk_peak_concurrent_calls() == 1,
                 "blocked SDK call was entered concurrently");
       }},
      {"dual slow second robot login does not falsely fault an idle first robot",
       [&] {
         fake_sdk_reset();
         Fixture left(sdk, false, 20, "left_", "192.0.2.51", "dual_startup");
         fake_sdk_robot_login_delay("192.0.2.52", 120);
         Fixture right(sdk, false, 20, "right_", "192.0.2.52", "dual_startup");
         require(wait_until([&] {
                   return fake_sdk_robot_snapshot("192.0.2.51").reads >= 2 &&
                          fake_sdk_robot_snapshot("192.0.2.52").reads >= 2;
                 }),
                 "dual startup did not resume feedback");
         require(left.read() == Return::OK && right.read() == Return::OK,
                 "startup gate contention latched idle robot");
         require(fake_sdk_peak_concurrent_calls() == 1, "Login overlapped another vendor call");
       }},
      {"swapped SDK joint axis order is rejected during initialization",
       [&] {
         auto info = hardware_info();
         std::swap(info.joints[0], info.joints[1]);
         iimt_ros2_control::IimtSystem system;
         require(system.on_init(info) == Callback::ERROR, "swapped physical joint axes accepted");
       }},
      {"activation seeds measured positions without sending before claim",
       [&] {
         Fixture f(sdk, true);
         for (const auto &command : f.commands)
           require(std::abs(command.get_value() - 0.01) < 1e-12,
                   "activation not seeded from measured position");
         const int reads = fake_sdk_snapshot().reads;
         require(wait_until([&] { return fake_sdk_snapshot().reads >= reads + 3; }),
                 "feedback worker stopped");
         require(fake_sdk_snapshot().sends == 0, "SDK motion before controller claim");
         f.cleanup();
         require(fake_sdk_snapshot().stops == 0 && fake_sdk_snapshot().clears == 0,
                 "unclaimed cleanup interfered with robot motion");
       }},
      {"read-only hardware rejects claim and never sends Stop or Clear",
       [&] {
         Fixture f(sdk, false);
         require(f.system.prepare_command_mode_switch(f.keys, {}) == Return::ERROR,
                 "read-only claim accepted");
         require(f.system.perform_command_mode_switch(f.keys, {}) == Return::ERROR,
                 "read-only perform claim accepted");
         require(f.write() == Return::OK, "read-only write failed");
         f.system.on_deactivate(f.state);
         f.cleanup();
         const auto s = fake_sdk_snapshot();
         require(s.sends == 0 && s.stops == 0 && s.clears == 0,
                 "read-only path issued motion-affecting SDK call");
         require(s.logouts == 3, "read-only cleanup leaked SDK handles");
       }},
      {"all-six claim required and stable hold heartbeat remains healthy",
       [&] {
         Fixture f(sdk, true);
         auto partial = f.keys;
         partial.pop_back();
         require(f.system.prepare_command_mode_switch(partial, {}) == Return::ERROR,
                 "partial claim accepted");
         auto duplicate = f.keys;
         duplicate[5] = duplicate[0];
         require(f.system.prepare_command_mode_switch(duplicate, {}) == Return::ERROR,
                 "duplicate claim accepted");
         f.claim();
         f.live_hold(
             20); // Longer than command_timeout: unchanged positions still refresh heartbeat.
         require(f.read() == Return::OK, "stable hold lost feedback");
         const auto s = fake_sdk_snapshot();
         for (const auto q : s.sent.q)
           require(std::abs(q - 0.01) < 1e-12, "hold did not use seeded measured joints");
         f.system.on_deactivate(f.state);
         f.cleanup();
         require(fake_sdk_snapshot().stops >= 1 && fake_sdk_snapshot().clears >= 1,
                 "deactivation did not stop and clear");
       }},
      {"each controller write is sent once without periodic repeats",
       [&] {
         Fixture f(sdk, true);
         f.claim();
         f.live_hold(3);
         const int sent = fake_sdk_snapshot().sends;
         require(sent == 3, "claim or writer generated extra ServoJ sends");
         // This is an observation window spanning three servo periods, not a retry.
         std::this_thread::sleep_for(60ms);
         require(fake_sdk_snapshot().sends == sent,
                 "worker repeated an old command without a write");
         require(f.read() == Return::ERROR, "missed controller deadline did not fault");
       }},
      {"bunched controller commands fault before another ServoJ send",
       [&] {
         Fixture f(sdk, true);
         f.claim();
         f.live_hold(3);
         const int sent = fake_sdk_snapshot().sends;
         // live_hold returns as soon as the last write has been consumed (2ms poll).
         f.write();
         require(wait_until([&] { return f.read() == Return::ERROR; }),
                 "bunched command interval accepted");
         require(fake_sdk_snapshot().sends == sent, "bunched command reached ServoJ");
       }},
      {"controller cannot reclaim while asynchronous Stop is pending",
       [&] {
         Fixture f(sdk, true);
         f.claim();
         f.live_hold(3);
         struct ReleaseStop {
           ~ReleaseStop() {
             fake_sdk_block_stop(false);
             fake_sdk_block_read(false);
           }
         } release;
         fake_sdk_block_stop(true);
         require(f.system.perform_command_mode_switch({}, f.keys) == Return::OK,
                 "controller release failed");
         require(wait_until([&] { return fake_sdk_snapshot().stops > 0; }),
                 "worker did not enter Stop");
         require(f.system.perform_command_mode_switch({}, f.keys) == Return::OK,
                 "repeated controller release failed");
         require(f.system.on_deactivate(f.state) == Callback::SUCCESS,
                 "deactivate during Stop failed");
         require(f.system.on_deactivate(f.state) == Callback::SUCCESS,
                 "repeated deactivate failed");
         require(f.system.on_activate(f.state) == Callback::FAILURE,
                 "activation accepted during blocked Stop");
         require(f.system.prepare_command_mode_switch(f.keys, {}) == Return::ERROR,
                 "claim accepted during blocked Stop");
         require(f.system.perform_command_mode_switch(f.keys, {}) == Return::ERROR,
                 "claim executed during blocked Stop");
         const int reads = fake_sdk_snapshot().reads;
         fake_sdk_set_feedback(0.02);
         fake_sdk_block_read(true);
         fake_sdk_block_stop(false);
         require(wait_until([&] { return fake_sdk_snapshot().reads > reads; }),
                 "post-stop measurement not requested");
         require(f.system.on_activate(f.state) == Callback::FAILURE,
                 "activation accepted before post-stop measurement completed");
         require(f.system.perform_command_mode_switch(f.keys, {}) == Return::ERROR,
                 "claim executed before post-stop feedback");
         fake_sdk_block_read(false);
         require(wait_until([&] {
                   return f.system.prepare_command_mode_switch(f.keys, {}) == Return::OK;
                 }),
                 "completed Stop and feedback did not release pending handover");
         require(f.system.on_activate(f.state) == Callback::SUCCESS,
                 "activation rejected after post-stop feedback");
         f.claim();
         require(std::abs(f.commands[0].get_value() - 0.02) < 1e-12,
                 "reclaim did not seed post-stop measurement");
         f.live_hold(2);
         require(fake_sdk_snapshot().stops == 1,
                 "duplicate stop requests survived completed handover");
         require(f.read() == Return::OK, "post-stop reclaim failed to resume fresh commands");
       }},
      {"smooth timed motion survives independent writer and worker phase",
       [&] {
         Fixture f(sdk, true);
         fake_sdk_set_follow_commands(true);
         f.claim();
         f.live_hold(3);
         smooth_motion(f, 4);
       }},
      {"125Hz smooth timed motion stays healthy with bounded writer jitter",
       [&] {
         Fixture f(sdk, true, 8);
         fake_sdk_set_follow_commands(true);
         f.claim();
         f.live_hold(3);
         smooth_motion(f, 1);
       }},
      {"stale controller heartbeat latches fault and blocks later sends",
       [&] {
         Fixture f(sdk, true);
         f.claim();
         f.live_hold();
         require(wait_until([&] { return f.read() == Return::ERROR; }),
                 "stale controller did not fault");
         const int sent = fake_sdk_snapshot().sends;
         require(f.write() == Return::ERROR, "write resumed after stale-command fault");
         require_callback_error_log("on_activate", [&] { return f.system.on_activate(f.state); });
         require_callback_error_log("on_configure", [&] { return f.system.on_configure(f.state); });
         require(wait_until([&] { return fake_sdk_snapshot().stops >= 1; }),
                 "fault did not request stop");
         require(fake_sdk_snapshot().sends == sent, "commands continued after fault");
       }},
      {"SDK read failure latches fault without refreshing feedback",
       [&] {
         Fixture f(sdk, true);
         f.claim();
         f.live_hold();
         fake_sdk_set_read_ok(false);
         require(wait_until([&] { return f.read() == Return::ERROR; }),
                 "SDK feedback failure did not fault");
         const int sent = fake_sdk_snapshot().sends;
         fake_sdk_set_read_ok(true);
         require(f.write() == Return::ERROR, "restored SDK cleared latch");
         require(f.system.on_activate(f.state) == Callback::ERROR,
                 "feedback fault reactivation accepted");
         require(wait_until([&] { return fake_sdk_snapshot().stops >= 1; }),
                 "feedback fault did not stop");
         require(fake_sdk_snapshot().sends == sent, "send resumed after feedback fault");
       }},
      {"on_error returns promptly and retains diagnostic node until cleanup",
       [&] {
         Fixture f(sdk, true);
         f.claim();
         f.live_hold();
         auto observer = std::make_shared<rclcpp::Node>("iimt_test_observer");
         auto visible = [&] {
           const auto names = observer->get_node_names();
           return std::find(names.begin(), names.end(), "/iimt_hardware") != names.end();
         };
         require(wait_until(visible), "diagnostic node absent before error");
         const auto before = Clock::now();
         require_callback_error_log("on_error", [&] { return f.system.on_error(f.state); });
         require(Clock::now() - before < 100ms, "on_error blocked controller thread");
         require(f.read() == Return::ERROR && f.write() == Return::ERROR,
                 "on_error failed to latch IO");
         require(visible(), "on_error removed diagnostic node before cleanup");
         f.cleanup();
       }},
  };
  unsigned failed = 0, selected = 0;
  for (const auto &test : tests) {
    if (std::string(test.first).find(filter) == std::string::npos)
      continue;
    ++selected;
    try {
      test.second();
      std::cout << "PASS " << test.first << '\n';
    } catch (const std::exception &e) {
      ++failed;
      std::cerr << "FAIL " << test.first << ": " << e.what() << '\n';
    }
  }
  rclcpp::shutdown();
  std::cout << selected - failed << " passed, " << failed << " failed\n";
  return failed || selected == 0 ? 1 : 0;
}
