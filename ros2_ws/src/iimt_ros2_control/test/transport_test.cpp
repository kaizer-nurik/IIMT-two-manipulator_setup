#include "iimt_ros2_control/core.hpp"
#include "fake_sdk.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
namespace iimt_ros2_control { std::unique_ptr<Transport> make_transport(); }
namespace {
void require(bool ok, const char * reason) { if (!ok) throw std::runtime_error(reason); }
bool close(double actual, double expected) { return std::abs(actual-expected) < 1e-6; }
}
int main(int argc, char ** argv) {
  using namespace iimt_ros2_control;
  try {
    require(argc == 2, "fake SDK path required");
    Config c; c.robot_ip = "192.0.2.1"; c.port = 30001; c.sdk_library_path = argv[1];
    c.servo_a = 1.25; c.servo_v = 0.75; c.period = 0.016; c.lookahead_time = 0.024; c.gain = 2;
    c.validate();
    fake_sdk_reset(); auto * state = fake_sdk_state(); auto transport = make_transport();
    require(transport->connect(c), "fake SDK connection failed");
    require(state->logins == 3 && state->port == c.port, "configured port or separate connections lost");
    for (int i = 0; i < 6; ++i) state->feedback.robotRad[i] = (i+1)*0.01;
    Joints q{};
    require(transport->read(q), "valid feedback failed");
    for (int i = 0; i < 6; ++i) require(close(q[i],(i+1)*0.01), "joint feedback not preserved");
    require(transport->send(q,c), "valid command failed");
    for (int i = 0; i < 6; ++i) require(close(state->sent.q[i],q[i]), "command joints not preserved");
    require(close(state->sent.a,c.servo_a) && close(state->sent.v,c.servo_v) &&
            close(state->sent.t,c.period) && close(state->sent.lookahead_time,c.lookahead_time) &&
            close(state->sent.gain,c.gain), "ServoJ parameters replaced by hardcoded values");
    require(state->read_handle != state->send_handle, "feedback and ServoJ share blocking connection");
    state->read_ok = false;
    require(!transport->read(q), "SDK read failure suppressed");
    state->send_ok = false;
    require(!transport->send(q,c), "SDK command failure suppressed");
    require(transport->stop(), "successful stop reported failure");
    require(state->stops == 1 && state->clears == 1, "stop must also clear pending trajectory");
    require(state->stop_handle == state->clear_handle && state->stop_handle != state->send_handle &&
            state->stop_handle != state->read_handle, "stop uses blocked feedback/command connection");
    state->stop_ok = false;
    require(!transport->stop(), "SDK Stop failure suppressed");
    require(state->clears == 2, "clear omitted after Stop failure");
    state->stop_ok = true; state->clear_ok = false;
    require(!transport->stop(), "SDK Clear failure suppressed");
    transport->disconnect(); transport->disconnect();
    require(state->logouts == 3, "disconnect leaks handles or double logs out");
    transport.reset();
    for (int failed_login : {1,2,3}) {
      fake_sdk_reset(); state->fail_login_at = failed_login; transport = make_transport();
      require(!transport->connect(c), "partial SDK login failure suppressed");
      transport->disconnect();
      require(state->logouts == failed_login-1, "partial login leaked successful handle");
      transport.reset();
    }
    fake_sdk_reset(); transport = make_transport();
    require(transport->connect(c), "logout failure fixture connection failed");
    state->logout_ok = false;
    bool logout_rejected = false;
    try { transport->disconnect(); }
    catch (const std::runtime_error &) { logout_rejected = true; }
    require(logout_rejected, "SDK Logout failure suppressed");
    require(state->logouts == 3, "failed logout skipped remaining handles");
    state->logout_ok = true;
    transport->disconnect();
    require(state->logouts == 6, "failed logout handles were discarded instead of retried");
    transport.reset();
    bool rejected = false;
    try { c.sdk_library_path = "/nonexistent/iimt-test-sdk.so"; transport = make_transport(); transport->connect(c); }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "missing SDK library accepted");
    std::cout << "PASS fake SDK parameter forwarding, separate connections, feedback, errors, stop, cleanup\n";
  } catch (const std::exception & e) { std::cerr << "FAIL " << e.what() << '\n'; return 1; }
}
