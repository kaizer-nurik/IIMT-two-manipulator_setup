// Test-only SDK symbols: no vendor library, sockets, power or brake operations.
#include "robotapi.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <mutex>

namespace {
std::mutex mutex;
std::array<double, 6> position{};
bool enabled(const char *name) {
  const char *value = std::getenv(name);
  return value && std::strcmp(value, "1") == 0;
}
} // namespace

BOOL API_Login(const char *, int, JTRHD &handle) {
  static JTRHD next = 0;
  handle = ++next;
  return TRUE;
}
BOOL API_Logout(JTRHD &handle) {
  handle = 0;
  return TRUE;
}
BOOL API_GetRobotJointRad(JTRHD, JointRad *state) {
  std::lock_guard<std::mutex> lock(mutex);
  std::copy(position.begin(), position.end(), state->robotRad);
  return TRUE;
}
BOOL API_Servoj(JTRHD, ServojParam *command) {
  if (enabled("IIMT_TEST_SERVO_FAIL"))
    return FALSE;
  if (enabled("IIMT_TEST_FOLLOW_COMMAND")) {
    std::lock_guard<std::mutex> lock(mutex);
    std::copy(command->q, command->q + 6, position.begin());
  }
  return TRUE;
}
BOOL API_Stop(JTRHD) {
  return enabled("IIMT_TEST_STOP_FAIL") ? FALSE : TRUE;
}
BOOL API_ClearUnfinishedTrajectory(JTRHD) {
  return enabled("IIMT_TEST_CLEAR_FAIL") ? FALSE : TRUE;
}
