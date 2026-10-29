#include "joint_reader.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

using iimt_ros2_driver::JointReader;
using Result = JointReader::Result;
using namespace std::chrono_literals;

void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

void send_text(int fd, const std::string &text) {
  ::send(fd, text.data(), text.size(), MSG_NOSIGNAL);
}

void request(int fd) {
  std::string received;
  char byte;
  while (::recv(fd, &byte, 1, 0) == 1 && byte != '\n')
    received += byte;
  require(received == "get joint rad", "Unexpected wire request");
}

class Server {
public:
  explicit Server(std::function<void(int)> handler, int connections = 1) {
    listener_ = ::socket(AF_INET, SOCK_STREAM, 0);
    require(listener_ >= 0, "socket failed");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    require(::bind(listener_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0,
            "bind failed");
    socklen_t size = sizeof(address);
    require(::getsockname(listener_, reinterpret_cast<sockaddr *>(&address), &size) == 0,
            "getsockname failed");
    port = ntohs(address.sin_port);
    require(::listen(listener_, 2) == 0, "listen failed");
    worker_ = std::thread([this, handler, connections] {
      try {
        for (int index = 0; index < connections; ++index) {
          const int fd = ::accept(listener_, nullptr, nullptr);
          if (fd < 0)
            break;
          timeval timeout{2, 0};
          ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
          handler(fd);
          ::close(fd);
        }
      } catch (...) {
        error_ = std::current_exception();
      }
    });
  }
  ~Server() {
    ::shutdown(listener_, SHUT_RDWR);
    if (worker_.joinable())
      worker_.join();
    ::close(listener_);
  }
  void finish() {
    worker_.join();
    if (error_)
      std::rethrow_exception(error_);
  }
  int port{};

private:
  int listener_{-1};
  std::thread worker_;
  std::exception_ptr error_;
};

const std::string reply = "Joint radian 0.1,-0.2,0.3,-0.4,0.5,-0.6\r\n";
const std::array<double, 6> untouched{42, 42, 42, 42, 42, 42};

void expect_failure(const std::function<void(int)> &handler, Result expected, double timeout = 0.1,
                    const std::string &detail = "") {
  Server server(handler);
  JointReader reader;
  auto output = untouched;
  const auto started = std::chrono::steady_clock::now();
  require(reader.read("127.0.0.1", server.port, timeout, output) == expected,
          "Wrong failure result");
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  require(elapsed < timeout + 0.15, "Deadline was reset or unbounded");
  require(output == untouched, "Failed read changed output");
  require(reader.error().find("127.0.0.1:" + std::to_string(server.port)) != std::string::npos,
          "Missing server endpoint in failure diagnostic");
  for (const auto &field : {"phase=", "timeout=", "elapsed="})
    require(reader.error().find(field) != std::string::npos, "Missing read failure context");
  require(reader.error().find(detail) != std::string::npos, "Missing requested diagnostic detail");
  server.finish();
}

int main() {
  try {
    {
      Server server([](int fd) {
        send_text(fd, "Robot ready\n");
        for (int i = 0; i < 2; ++i) {
          request(fd);
          send_text(fd, reply.substr(0, 10));
          std::this_thread::sleep_for(5ms);
          send_text(fd, reply.substr(10));
        }
        std::this_thread::sleep_for(20ms);
      });
      JointReader reader;
      std::array<double, 6> output{};
      for (int i = 0; i < 2; ++i) {
        const auto result = reader.read("127.0.0.1", server.port, .5, output);
        require(result == Result::success, reader.error().c_str());
        require(output[0] == .1 && output[5] == -.6, "Incorrect parsed positions");
      }
      reader.close();
      server.finish();
    }
    expect_failure([](int) { std::this_thread::sleep_for(150ms); }, Result::timeout);
    expect_failure(
        [](int fd) {
          send_text(fd, "Ready\n");
          request(fd);
          std::this_thread::sleep_for(150ms);
        },
        Result::timeout);
    expect_failure(
        [](int fd) {
          // Greeting and fragmented reply share a single deadline.
          std::this_thread::sleep_for(60ms);
          send_text(fd, "Ready\n");
          request(fd);
          for (char byte : reply) {
            send_text(fd, std::string(1, byte));
            std::this_thread::sleep_for(5ms);
          }
        },
        Result::timeout);
    for (const std::string malformed :
         {"Wrong 1,2,3,4,5,6\n", "Joint radian 1,2,3,4,5\n", "Joint radian 1,2,3,4,5,6,7\n",
          "Joint radian nan,2,3,4,5,6\n", "Joint radian inf,2,3,4,5,6\n",
          "Joint radian 1e999,2,3,4,5,6\n", "Joint radian 1,2,3,4,5,6garbage\n",
          "Joint radian 1 2,3,4,5,6\n", "Joint radian 1,2,3,4,5,6\nextra\n"}) {
      expect_failure(
          [&](int fd) {
            send_text(fd, "Ready\n");
            request(fd);
            send_text(fd, malformed);
            std::this_thread::sleep_for(20ms);
          },
          Result::error);
    }
    expect_failure(
        [](int fd) {
          send_text(fd, "Ready\n");
          request(fd);
          send_text(fd, std::string(1025, 'a'));
          std::this_thread::sleep_for(20ms);
        },
        Result::error);
    expect_failure(
        [](int fd) {
          send_text(fd, "Ready\n");
          request(fd);
          send_text(fd, "Joint radian 1,2");
        },
        Result::error);
    expect_failure(
        [](int fd) {
          send_text(fd, "Ready\n");
          request(fd);
          send_text(fd, "Wrong\t" + std::string(200, 'x') + "\n");
          std::this_thread::sleep_for(20ms);
        },
        Result::error, .1, "reply='Wrong?" + std::string(154, 'x') + "...'");
    {
      int connection = 0;
      Server server(
          [&](int fd) {
            send_text(fd, "Ready\n");
            request(fd);
            if (connection++ == 0) {
              std::this_thread::sleep_for(80ms);
              send_text(fd, "Joint radian 9,9,9,9,9,9\n");
            } else {
              send_text(fd, reply);
              std::this_thread::sleep_for(20ms);
            }
          },
          2);
      JointReader reader;
      auto output = untouched;
      require(reader.read("127.0.0.1", server.port, .03, output) == Result::timeout,
              "First request did not time out");
      require(output == untouched, "Timeout changed output");
      require(reader.read("127.0.0.1", server.port, .5, output) == Result::success,
              "Reconnect failed");
      require(output[0] == .1, "Late old reply was accepted");
      reader.close();
      server.finish();
    }
    {
      JointReader reader;
      auto output = untouched;
      require(reader.read("localhost", 29999, .1, output) == Result::error, "DNS input accepted");
      require(reader.read("127.0.0.1", 0, .1, output) == Result::error, "Bad port accepted");
      require(reader.read("127.0.0.1", 29999, 0, output) == Result::error, "Bad timeout accepted");
    }
    std::cout << "PASS: bounded deadline, fragmentation, persistent connection, late reply "
                 "reconnect, malformed/nonfinite/oversized replies, EOF, validation\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
