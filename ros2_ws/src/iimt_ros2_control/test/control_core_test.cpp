#include "iimt_ros2_control/core.hpp"

#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace iimt_ros2_control;
using namespace std::chrono_literals;
namespace {
void require(bool ok, const char * reason) {
  if (!ok) throw std::runtime_error(reason);
}
Config config() {
  Config c;
  c.robot_ip = "192.0.2.1"; // Documentation-only address: tests never create a transport.
  c.sdk_library_path = "/not-loaded/libiimt.so";
  return c;
}
void rejects(const Config & c) {
  bool rejected = false;
  try { c.validate(); } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "unsafe configuration accepted");
}
const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double inf = std::numeric_limits<double>::infinity();
}
int main() {
  using Test = std::pair<const char *, std::function<void()>>;
  const std::vector<Test> tests{
    {"valid configuration preserves vendor zero gain", [] {
      auto c = config(); c.validate();
      require(c.gain == 0, "zero gain must remain an explicit supported setting");
    }},
    {"connection configuration rejects missing and invalid values", [] {
      auto c = config(); c.robot_ip.clear(); rejects(c);
      c = config(); c.sdk_library_path.clear(); rejects(c);
      c = config(); c.lock_directory.clear(); rejects(c);
      for (int port : {0, -1, 65536}) { c = config(); c.port = port; rejects(c); }
    }},
    {"all positive scalar settings reject nonfinite zero and negative values", [] {
      const std::vector<double Config::*> fields{
        &Config::servo_a, &Config::servo_v, &Config::period, &Config::state_period,
        &Config::sdk_budget, &Config::sdk_lock_timeout, &Config::command_timeout, &Config::state_timeout,
        &Config::startup_timeout, &Config::max_lateness, &Config::ros_clock_jump,
        &Config::diagnostics_period};
      for (auto field : fields) for (double value : {0.0, -1.0, kNaN, inf}) {
        auto c = config(); c.*field = value; rejects(c);
      }
    }},
    {"servo period below steady clock resolution rejected", [] {
      auto c = config();
      c.period = std::chrono::duration<double>(Clock::duration{1}).count()/2;
      rejects(c);
    }},
    {"clock and SDK representation overflow and underflow rejected", [] {
      for (auto field : {&Config::period, &Config::state_period, &Config::sdk_budget, &Config::sdk_lock_timeout,
                         &Config::command_timeout, &Config::state_timeout, &Config::startup_timeout,
                         &Config::max_lateness, &Config::ros_clock_jump, &Config::diagnostics_period,
                         &Config::controller_cancel_timeout}) {
        auto c = config(); c.*field = 1e100; rejects(c);
        c = config(); c.*field = 1e-100; rejects(c);
      }
      for (auto field : {&Config::servo_a, &Config::servo_v, &Config::lookahead_time, &Config::gain}) {
        auto c = config(); c.*field = 1e100; rejects(c);
        c = config(); c.*field = 1e-100; rejects(c);
      }
    }},
    {"emitted duplicate hold cannot mask excessive restart acceleration", [] {
      auto c = config(); c.max_acceleration.fill(13.0);
      Joints q0{}, q1{}, q2{}, q3{}, v{}, measured{};
      q1.fill(0.0008); q2.fill(0.0016); q3.fill(0.0032);
      require(valid_step(q1,q0,v,measured,0.008,c), "initial safe acceleration rejected");
      v.fill(0.1);
      require(valid_step(q2,q1,v,measured,0.008,c), "constant velocity rejected");
      require(valid_step(q2,q2,v,measured,0.008,c), "safe hold deceleration rejected");
      v.fill(0);
      require(!valid_step(q3,q2,v,measured,0.008,c), "duplicate hold hid excessive emitted acceleration");
      require(valid_step(q3,q2,v,measured,0.016,c), "source timing counterexample no longer distinguishes emitted timing");
    }},
    {"ServoJ tuning rejects negative or nonfinite settings", [] {
      for (auto field : {&Config::gain, &Config::lookahead_time}) {
        for (double value : {-1.0, kNaN, inf}) { auto c = config(); c.*field = value; rejects(c); }
      }
    }},
    {"all six joint limit configurations are validated", [] {
      for (std::size_t joint = 0; joint < 6; ++joint) {
        for (auto field : {&Config::max_velocity, &Config::max_acceleration,
                           &Config::max_step, &Config::tracking_tolerance}) {
          for (double value : {0.0, -1.0, kNaN, inf}) {
            auto c = config(); (c.*field)[joint] = value; rejects(c);
          }
        }
        auto c = config(); c.min_position[joint] = c.max_position[joint]; rejects(c);
        c = config(); c.min_position[joint] = kNaN; rejects(c);
        c = config(); c.max_position[joint] = inf; rejects(c);
      }
    }},
    {"position limits inclusive and nonfinite positions rejected on each joint", [] {
      const auto c = config();
      require(valid_position(c.min_position, c), "lower boundary rejected");
      require(valid_position(c.max_position, c), "upper boundary rejected");
      for (std::size_t joint = 0; joint < 6; ++joint) {
        for (double value : {kNaN, inf, -inf, c.max_position[joint] + 0.001,
                             c.min_position[joint] - 0.001}) {
          Joints q{}; q[joint] = value;
          require(!valid_position(q,c), "invalid position accepted");
        }
      }
    }},
    {"stationary hold and acceleration-limited step accepted", [] {
      const auto c = config(); Joints zero{}, q{};
      require(valid_step(zero,zero,zero,zero,0.008,c), "hold rejected");
      q.fill(0.00005);
      require(valid_step(q,zero,zero,zero,0.008,c), "safe small step rejected");
    }},
    {"step velocity acceleration and tracking limits independently enforced", [] {
      Joints zero{}, q{};
      auto c = config(); q[0] = 0.03; c.max_velocity.fill(100); c.max_acceleration.fill(1e6);
      require(!valid_step(q,zero,zero,zero,0.008,c), "step limit bypassed");
      c = config(); q[0] = 0.009; c.max_acceleration.fill(1e6);
      require(!valid_step(q,zero,zero,zero,0.008,c), "velocity limit bypassed");
      c = config(); q[0] = 0.001;
      require(!valid_step(q,zero,zero,zero,0.008,c), "acceleration limit bypassed");
      c = config(); q[0] = 0.001; c.max_acceleration.fill(1e6);
      Joints measured{}; measured[0] = -0.3;
      require(!valid_step(q,zero,zero,measured,0.008,c), "tracking limit bypassed");
    }},
    {"invalid dt rejected", [] {
      const auto c = config(); Joints q{};
      for (double dt : {0.0, -0.008, kNaN, inf})
        require(!valid_step(q,q,q,q,dt,c), "invalid dt accepted");
    }},
    {"nonfinite previous command velocity and feedback fail closed", [] {
      const auto c = config(); Joints zero{};
      for (std::size_t joint = 0; joint < 6; ++joint) {
        for (double value : {kNaN, inf, -inf}) {
          Joints bad{}; bad[joint] = value;
          require(!valid_step(zero,bad,zero,zero,0.008,c), "nonfinite previous accepted");
          require(!valid_step(zero,zero,bad,zero,0.008,c), "nonfinite velocity accepted");
          require(!valid_step(zero,zero,zero,bad,0.008,c), "nonfinite feedback accepted");
        }
      }
    }},
    {"freshness watchdog rejects missing and expired samples", [] {
      const auto now = Clock::time_point{} + 1s;
      require(stale({},now,0.1), "uninitialized sample accepted");
      require(!stale(now-99ms,now,0.1), "fresh sample rejected");
      require(!stale(now-100ms,now,0.1), "exact timeout boundary rejected");
      require(stale(now-101ms,now,0.1), "expired sample accepted");
    }},
    {"mailbox contains only latest command without FIFO replay", [] {
      Mailbox box;
      require(box.get().sequence == 0, "mailbox not initially empty");
      const auto start = Clock::time_point{} + 1s;
      for (int k = 1; k <= 1000; ++k) {
        Joints q{}; q.fill(k * 0.001); box.put(q,start + k*1ms);
      }
      const auto latest = box.get();
      require(latest.sequence == 1000 && latest.position[5] == 1.0,
              "old command replayed instead of newest command");
      require(latest.stamp == start+1s, "command timestamp not preserved");
      require(box.get().sequence == latest.sequence, "reading invents command updates");
    }},
    {"global servo grid does not accumulate per-segment rounding", [] {
      const auto origin = Clock::time_point{} + 1s;
      auto slot = origin; std::uint64_t skipped = 0;
      for (int k = 1; k <= 500; ++k) {
        slot = next_slot(slot,slot+1ms,8ms,skipped);
        require(slot == origin+k*8ms, "servo schedule drifted from global grid");
      }
      require(slot == origin+4s && skipped == 0, "four-second schedule stretched");
    }},
    {"feedback polling yields to claimed command clock before first send", [] {
      require(feedback_poll_allowed(false,false), "idle/read-only polling disabled");
      require(!feedback_poll_allowed(true,false), "poll can delay first claimed command");
      require(feedback_poll_allowed(true,true), "post-send feedback disabled");
      require(feedback_poll_allowed(false,true), "release/post-stop feedback disabled");
    }},
    {"missed slots skipped without catch-up command bursts", [] {
      const auto origin = Clock::time_point{} + 1s; std::uint64_t skipped = 0;
      auto next = next_slot(origin,origin+25ms,8ms,skipped);
      require(next == origin+32ms && skipped == 3, "missed deadlines replayed");
      next = next_slot(next,origin+33ms,8ms,skipped);
      require(next == origin+40ms && skipped == 3, "normal slot drifted after skip");
      next = next_slot(next,origin+48ms,8ms,skipped);
      require(next == origin+56ms && skipped == 4, "exact due slot not skipped");
    }},
  };
  unsigned failed = 0;
  for (const auto & test : tests) {
    try { test.second(); std::cout << "PASS " << test.first << '\n'; }
    catch (const std::exception & error) {
      ++failed; std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
    }
  }
  std::cout << tests.size()-failed << " passed, " << failed << " failed\n";
  return failed ? 1 : 0;
}
