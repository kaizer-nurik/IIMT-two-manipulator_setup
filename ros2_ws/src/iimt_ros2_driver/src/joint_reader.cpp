#include "joint_reader.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstring>
#include <locale>
#include <sstream>

namespace iimt_ros2_driver {

JointReader::~JointReader() {
  close();
}

void JointReader::close() {
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
  ip_.clear();
  port_ = 0;
}

JointReader::Result JointReader::fail(Result result, const std::string &message) {
  error_ = message;
  close();
  return result;
}

JointReader::Result JointReader::check_deadline(Deadline deadline) {
  if (Clock::now() >= deadline)
    return fail(Result::timeout, "Joint read deadline exceeded");
  return Result::success;
}

JointReader::Result JointReader::wait(short events, Deadline deadline) {
  while (true) {
    if (check_deadline(deadline) != Result::success)
      return Result::timeout;
    const double remaining_ms =
        std::chrono::duration<double, std::milli>(deadline - Clock::now()).count();
    const int timeout_ms =
        static_cast<int>(std::clamp(std::ceil(remaining_ms), 1.0, static_cast<double>(INT_MAX)));
    pollfd descriptor{socket_, events, 0};
    const int ready = ::poll(&descriptor, 1, timeout_ms);
    if (check_deadline(deadline) != Result::success)
      return Result::timeout;
    if (ready < 0) {
      if (errno == EINTR)
        continue;
      return fail(Result::error, std::string("poll: ") + std::strerror(errno));
    }
    if (ready == 0)
      continue;
    // Read buffered bytes before handling HUP; EOF is detected by recv().
    if (descriptor.revents & events)
      return Result::success;
    return fail(Result::error, "Socket closed or failed while waiting for joint state");
  }
}

JointReader::Result JointReader::connect(const std::string &ip, int port, Deadline deadline) {
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<uint16_t>(port));
  ::inet_pton(AF_INET, ip.c_str(), &address.sin_addr);
  socket_ = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (socket_ < 0)
    return fail(Result::error, std::string("socket: ") + std::strerror(errno));
  if (::connect(socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    if (errno != EINPROGRESS && errno != EINTR)
      return fail(Result::error, std::string("connect: ") + std::strerror(errno));
    const Result ready = wait(POLLOUT, deadline);
    if (ready != Result::success)
      return ready;
    int socket_error = 0;
    socklen_t size = sizeof(socket_error);
    if (::getsockopt(socket_, SOL_SOCKET, SO_ERROR, &socket_error, &size) < 0)
      return fail(Result::error, std::string("getsockopt: ") + std::strerror(errno));
    if (socket_error != 0)
      return fail(Result::error, std::string("connect: ") + std::strerror(socket_error));
  }
  std::string greeting;
  const Result received = read_line(greeting, deadline);
  if (received != Result::success)
    return received;
  if (greeting.empty())
    return fail(Result::error, "Empty robot greeting");
  ip_ = ip;
  port_ = port;
  return Result::success;
}

JointReader::Result JointReader::check_idle(Deadline deadline, bool allow_closed) {
  while (true) {
    if (check_deadline(deadline) != Result::success)
      return Result::timeout;
    char byte;
    const ssize_t count = ::recv(socket_, &byte, 1, MSG_PEEK | MSG_DONTWAIT);
    if (count > 0)
      return fail(Result::error, "Unexpected extra data on joint state connection");
    if (count == 0) {
      if (allow_closed) {
        close();
        return Result::success;
      }
      return fail(Result::error, "Joint state connection closed");
    }
    if (errno == EINTR)
      continue;
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return Result::success;
    return fail(Result::error, std::string("recv peek: ") + std::strerror(errno));
  }
}

JointReader::Result JointReader::send_request(Deadline deadline) {
  const std::string request = "get joint rad\n";
  size_t sent = 0;
  while (sent < request.size()) {
    const Result ready = wait(POLLOUT, deadline);
    if (ready != Result::success)
      return ready;
    const ssize_t count =
        ::send(socket_, request.data() + sent, request.size() - sent, MSG_NOSIGNAL);
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
        continue;
      return fail(Result::error, std::string("send: ") + std::strerror(errno));
    }
    if (count == 0)
      return fail(Result::error, "No progress sending joint state request");
    sent += static_cast<size_t>(count);
  }
  return check_deadline(deadline);
}

JointReader::Result JointReader::read_line(std::string &line, Deadline deadline) {
  line.clear();
  while (true) {
    const Result ready = wait(POLLIN, deadline);
    if (ready != Result::success)
      return ready;
    char buffer[1024];
    const ssize_t count = ::recv(socket_, buffer, sizeof(buffer), 0);
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
        continue;
      return fail(Result::error, std::string("recv: ") + std::strerror(errno));
    }
    if (count == 0)
      return fail(Result::error, "EOF before complete joint state reply; received_bytes=" +
                                     std::to_string(line.size()));
    line.append(buffer, static_cast<size_t>(count));
    if (line.size() > 1024)
      return fail(Result::error, "Joint state line exceeds 1024 bytes; received_bytes=" +
                                     std::to_string(line.size()));
    const size_t newline = line.find('\n');
    if (newline == std::string::npos)
      continue;
    if (newline + 1 != line.size())
      return fail(Result::error, "Unexpected extra data after joint state line; trailing_bytes=" +
                                     std::to_string(line.size() - newline - 1));
    line.pop_back();
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    return check_deadline(deadline);
  }
}

bool JointReader::parse(const std::string &line, std::array<double, 6> &output) {
  const std::string prefix = "Joint radian ";
  if (line.compare(0, prefix.size(), prefix) != 0)
    return false;
  std::istringstream stream(line.substr(prefix.size()));
  stream.imbue(std::locale::classic());
  for (size_t joint = 0; joint < output.size(); ++joint) {
    if (!(stream >> output[joint]) || !std::isfinite(output[joint]))
      return false;
    if (joint + 1 < output.size()) {
      char comma;
      if (!(stream >> comma) || comma != ',')
        return false;
    }
  }
  stream >> std::ws;
  return stream.eof();
}

JointReader::Result JointReader::read(const std::string &ip, int port, double timeout,
                                      std::array<double, 6> &output) {
  error_.clear();
  const auto started = Clock::now();
  const auto diagnostic = [&](Result result, const char *phase) {
    if (result != Result::success) {
      const double elapsed = std::chrono::duration<double>(Clock::now() - started).count();
      error_ = "TCP joint read " + ip + ":" + std::to_string(port) + " phase=" + phase +
               ", timeout=" + std::to_string(timeout) + " s, elapsed=" + std::to_string(elapsed) +
               " s: " + error_;
    }
    return result;
  };
  in_addr address{};
  if (::inet_pton(AF_INET, ip.c_str(), &address) != 1 || port < 1 || port > 65535 ||
      !std::isfinite(timeout) || timeout <= 0.0 ||
      timeout > std::chrono::duration<double>(Clock::duration::max()).count() / 2.0)
    return diagnostic(fail(Result::error, "Invalid IPv4 address, port or read timeout"),
                      "validation");
  const Deadline deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                               std::chrono::duration<double>(timeout));
  if (socket_ >= 0 && (ip_ != ip || port_ != port))
    close();
  if (socket_ < 0) {
    const Result connected = connect(ip, port, deadline);
    if (connected != Result::success)
      return diagnostic(connected, "connect/greeting");
  }
  Result result = check_idle(deadline);
  if (result != Result::success)
    return diagnostic(result, "before request");
  result = send_request(deadline);
  if (result != Result::success)
    return diagnostic(result, "send get joint rad");
  std::string line;
  result = read_line(line, deadline);
  if (result != Result::success)
    return diagnostic(result, "receive joint reply");
  std::array<double, 6> parsed{};
  if (!parse(line, parsed)) {
    // Bound and sanitize external data before including it in ROS logs.
    std::string preview = line.substr(0, 160);
    for (char &byte : preview)
      if (static_cast<unsigned char>(byte) < 32 || static_cast<unsigned char>(byte) > 126)
        byte = '?';
    return diagnostic(
        fail(Result::error, "Malformed joint state reply: expected 'Joint radian ' and six finite "
                            "comma-separated radians; received_bytes=" +
                                std::to_string(line.size()) + ", reply='" + preview +
                                (line.size() > preview.size() ? "...'" : "'")),
        "parse joint reply");
  }
  result = check_idle(deadline, true);
  if (result != Result::success)
    return diagnostic(result, "after reply");
  result = check_deadline(deadline);
  if (result == Result::success)
    output = parsed;
  return diagnostic(result, "complete read");
}

} // namespace iimt_ros2_driver
