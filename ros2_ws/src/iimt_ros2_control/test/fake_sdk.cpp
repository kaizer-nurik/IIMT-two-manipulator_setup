#include "fake_sdk.hpp"
#include <mutex>
#include <condition_variable>
#include <map>
#include <string>
#include <atomic>
#include <thread>
#include <chrono>
namespace {
FakeSdkState state;
std::map<std::string, FakeSdkState> robots;
std::map<JTRHD, FakeSdkState *> handles;
std::string first_ip;
JTRHD next_handle = 100;
std::map<std::string, int> login_delay;
std::atomic<int> active_calls{0}, peak_calls{0};
struct Call {
  Call() { const int n = ++active_calls; int old = peak_calls.load();
    while (n > old && !peak_calls.compare_exchange_weak(old,n)) {} }
  ~Call() { --active_calls; }
};
FakeSdkState & for_ip(const std::string & ip) {
  if (first_ip.empty()) first_ip = ip;
  return ip == first_ip ? state : robots[ip];
}
FakeSdkState & for_handle(JTRHD handle) { return *handles.at(handle); }
 std::mutex state_mutex; std::condition_variable stop_gate; bool stop_blocked = false; std::condition_variable read_gate; bool read_blocked = false; }
extern "C" FakeSdkState * fake_sdk_state() { return &state; }
extern "C" void fake_sdk_reset() { std::lock_guard<std::mutex> lock(state_mutex); state = FakeSdkState{}; robots.clear(); handles.clear(); first_ip.clear(); next_handle = 100; login_delay.clear(); peak_calls = 0; }
BOOL API_Login(const char * ip, int port, JTRHD & handle) {
  Call call;
  std::unique_lock<std::mutex> lock(state_mutex);
  auto & state = for_ip(ip);
  ++state.logins; state.port = port;
  const int delay = login_delay[ip];
  lock.unlock(); std::this_thread::sleep_for(std::chrono::milliseconds(delay)); lock.lock();
  if (state.logins == state.fail_login_at) return FALSE;
  handle = ++next_handle; handles[handle] = &state; return TRUE;
}
BOOL API_Logout(JTRHD & handle) {
  Call call; std::lock_guard<std::mutex> lock(state_mutex);
  auto & state = for_handle(handle); ++state.logouts;
  if (!state.logout_ok) return FALSE;
  handles.erase(handle); handle = 0; return TRUE;
}
BOOL API_GetRobotJointRad(JTRHD handle, JointRad * q) {
  Call call; std::unique_lock<std::mutex> lock(state_mutex); auto & state = for_handle(handle); ++state.reads;
  read_gate.wait(lock, [] { return !read_blocked; });
  state.read_handle = handle;
  const int delay = state.read_delay_ms;
  lock.unlock();
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  lock.lock();
  if (!state.read_ok) return FALSE;
  *q = state.feedback; return TRUE;
}
BOOL API_Servoj(JTRHD handle, ServojParam * p) {
  Call call; std::lock_guard<std::mutex> lock(state_mutex); auto & state = for_handle(handle); ++state.sends;
  state.send_handle = handle; state.sent = *p;
  if (state.send_ok && state.follow_commands)
    for (int i = 0; i < 6; ++i) state.feedback.robotRad[i] = p->q[i];
  return state.send_ok ? TRUE : FALSE;
}
BOOL API_Stop(JTRHD handle) {
  Call call; std::unique_lock<std::mutex> lock(state_mutex); auto & state = for_handle(handle); ++state.stops; state.stop_handle = handle;
  stop_gate.wait(lock, [] { return !stop_blocked; });
  return state.stop_ok ? TRUE : FALSE;
}
BOOL API_ClearUnfinishedTrajectory(JTRHD handle) {
  Call call; std::lock_guard<std::mutex> lock(state_mutex); auto & state = for_handle(handle); ++state.clears; state.clear_handle = handle; return state.clear_ok ? TRUE : FALSE;
}

extern "C" FakeSdkState fake_sdk_snapshot() {
  std::lock_guard<std::mutex> lock(state_mutex); return state;
}
extern "C" void fake_sdk_set_read_ok(bool ok) {
  std::lock_guard<std::mutex> lock(state_mutex); state.read_ok = ok;
}
extern "C" void fake_sdk_set_read_delay(int milliseconds) {
  std::lock_guard<std::mutex> lock(state_mutex); state.read_delay_ms = milliseconds;
}
extern "C" void fake_sdk_set_feedback(double value) {
  std::lock_guard<std::mutex> lock(state_mutex);
  for (auto & q : state.feedback.robotRad) q = value;
}

extern "C" void fake_sdk_set_follow_commands(bool enabled) {
  std::lock_guard<std::mutex> lock(state_mutex); state.follow_commands = enabled;
}

extern "C" void fake_sdk_block_stop(bool blocked) {
  { std::lock_guard<std::mutex> lock(state_mutex); stop_blocked = blocked; }
  stop_gate.notify_all();
}

extern "C" void fake_sdk_block_read(bool blocked) {
  { std::lock_guard<std::mutex> lock(state_mutex); read_blocked = blocked; }
  read_gate.notify_all();
}


// IP-scoped helpers leave the legacy first-robot API unchanged.
FakeSdkState fake_sdk_robot_snapshot(const std::string & ip) {
  std::lock_guard<std::mutex> lock(state_mutex); return for_ip(ip);
}
void fake_sdk_robot_feedback(const std::string & ip, const std::array<double,6> & q) {
  std::lock_guard<std::mutex> lock(state_mutex);
  for (int i=0;i<6;++i) for_ip(ip).feedback.robotRad[i]=q[i];
}
void fake_sdk_robot_follow_commands(const std::string & ip, bool enabled) {
  std::lock_guard<std::mutex> lock(state_mutex); for_ip(ip).follow_commands=enabled;
}
void fake_sdk_robot_read_ok(const std::string & ip, bool ok) {
  std::lock_guard<std::mutex> lock(state_mutex); for_ip(ip).read_ok=ok;
}
void fake_sdk_robot_login_delay(const std::string & ip, int ms) {
  std::lock_guard<std::mutex> lock(state_mutex); login_delay[ip]=ms;
}
int fake_sdk_peak_concurrent_calls() { return peak_calls.load(); }
