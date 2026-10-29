#pragma once

#include <array>
#include <chrono>
#include <string>

namespace iimt_ros2_driver {

// Bounded request/reply transport for the manufacturer's "get joint rad" command.
// A failed transaction closes the connection, so late replies cannot become fresh state.
class JointReader {
public:
  enum class Result { success, timeout, error };

  JointReader() = default;
  ~JointReader();
  JointReader(const JointReader &) = delete;
  JointReader &operator=(const JointReader &) = delete;

  Result read(const std::string &ip, int port, double timeout, std::array<double, 6> &output);
  void close();
  std::string error() const {
    return error_;
  }

private:
  using Clock = std::chrono::steady_clock;
  using Deadline = Clock::time_point;

  Result fail(Result result, const std::string &message);
  Result check_deadline(Deadline deadline);
  Result wait(short events, Deadline deadline);
  Result connect(const std::string &ip, int port, Deadline deadline);
  Result send_request(Deadline deadline);
  Result read_line(std::string &line, Deadline deadline);
  Result check_idle(Deadline deadline, bool allow_closed = false);
  static bool parse(const std::string &line, std::array<double, 6> &output);

  int socket_{-1};
  std::string ip_;
  int port_{0};
  std::string error_;
};

} // namespace iimt_ros2_driver
