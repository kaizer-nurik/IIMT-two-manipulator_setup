#pragma once
#pragma pack(push)
#include "robotapi.h"
#pragma pack(pop)
struct FakeSdkState {
  int logins{0}, logouts{0}, stops{0}, clears{0}, sends{0}, reads{0};
  int fail_login_at{0}; bool read_ok{true}, send_ok{true}, stop_ok{true}, clear_ok{true};
  bool follow_commands{false}, logout_ok{true};
  int read_delay_ms{0};
  int port{0}; JTRHD read_handle{0}, send_handle{0}, stop_handle{0}, clear_handle{0};
  JointRad feedback{}; ServojParam sent{};
};
// Raw access is restricted to the single-threaded transport test.
// Worker integration uses locked snapshots and setters below.
extern "C" FakeSdkState * fake_sdk_state();
extern "C" void fake_sdk_reset();
extern "C" FakeSdkState fake_sdk_snapshot();
extern "C" void fake_sdk_set_read_ok(bool);
extern "C" void fake_sdk_set_read_delay(int milliseconds);
extern "C" void fake_sdk_set_follow_commands(bool);
extern "C" void fake_sdk_block_stop(bool);
extern "C" void fake_sdk_set_feedback(double);
extern "C" void fake_sdk_block_read(bool);


#include <array>
#include <string>
FakeSdkState fake_sdk_robot_snapshot(const std::string & ip);
void fake_sdk_robot_feedback(const std::string & ip, const std::array<double,6> & q);
void fake_sdk_robot_follow_commands(const std::string & ip, bool enabled);
void fake_sdk_robot_read_ok(const std::string & ip, bool ok);
void fake_sdk_robot_login_delay(const std::string & ip, int ms);
int fake_sdk_peak_concurrent_calls();
