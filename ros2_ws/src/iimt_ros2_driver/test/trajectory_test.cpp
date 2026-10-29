#include "trajectory.hpp"
#include <functional>
#include <iostream>
#include <limits>

using namespace iimt_ros2_driver::trajectory;
void require(bool pass, const char *message) {
  if (!pass)
    throw std::runtime_error(message);
}
void near(double a, double b) {
  require(std::abs(a - b) < 1e-9, "interpolation mismatch");
}
void rejects(const std::vector<Point> &points) {
  bool rejected = false;
  try {
    validate(points);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "invalid trajectory accepted");
}
void test_joint_mapping() {
  const std::vector<std::string> expected{"joint1", "joint2", "joint3",
                                          "joint4", "joint5", "joint6"};
  auto received = expected;
  int permutations = 0;
  do {
    Point input;
    input.time = 1.5;
    input.derivatives = 2;
    for (size_t i = 0; i < 6; ++i) {
      const auto axis = std::find(expected.begin(), expected.end(), received[i]) - expected.begin();
      input.position[i] = axis + .1;
      input.velocity[i] = axis + .2;
      input.acceleration[i] = axis + .3;
    }
    const auto output = reorder(input, joint_order(expected, received));
    near(output.time, input.time);
    require(output.derivatives == 2, "derivative level changed");
    for (size_t axis = 0; axis < 6; ++axis) {
      near(output.position[axis], axis + .1);
      near(output.velocity[axis], axis + .2);
      near(output.acceleration[axis], axis + .3);
    }
    for (int field = 0; field < 3; ++field) {
      auto invalid = input;
      auto &values = field == 0   ? invalid.position
                     : field == 1 ? invalid.velocity
                                  : invalid.acceleration;
      values[2] = std::numeric_limits<double>::quiet_NaN();
      rejects({reorder(invalid, joint_order(expected, received))});
    }
    ++permutations;
  } while (std::next_permutation(received.begin(), received.end()));
  require(permutations == 720, "not all joint permutations tested");
  for (const auto &invalid : std::vector<std::vector<std::string>>{
           {},
           {"joint1", "joint2", "joint3", "joint4", "joint5"},
           {"joint1", "joint2", "joint3", "joint4", "joint5", "joint5"},
           {"joint1", "joint2", "joint3", "joint4", "joint5", "unknown"},
           {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "extra"}}) {
    bool rejected = false;
    try {
      joint_order(expected, invalid);
    } catch (const std::invalid_argument &error) {
      const std::string message = error.what();
      require(message.find("expected [joint1") != std::string::npos &&
                  message.find("received [") != std::string::npos,
              "joint rejection must show expected and received names");
      rejected = true;
    }
    require(rejected, "invalid joint names accepted");
  }
}

void test_endpoint() {
  Point final;
  final.derivatives = 2;
  final.velocity.fill(1e-17);
  final.acceleration.fill(-.1875);
  const Joints max_acceleration{1.875, 2, 2, 2, 2, 2};
  validate_endpoint(final, max_acceleration);
  for (int field = 0; field < 4; ++field) {
    auto invalid = final;
    if (field == 0)
      invalid.velocity[0] = .01;
    else if (field == 1)
      invalid.acceleration[0] = 2;
    else if (field == 2)
      invalid.acceleration[0] = std::numeric_limits<double>::quiet_NaN();
    else
      invalid.velocity[0] = std::numeric_limits<double>::infinity();
    bool rejected = false;
    try {
      validate_endpoint(invalid, max_acceleration);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "unsafe endpoint accepted");
  }
  // Endpoint acceleration describes the incoming segment, not the subsequent hold.
  const auto hold = sample({final}, final.time + .01);
  near(hold.velocity[0], 0);
  near(hold.acceleration[0], 0);
}

void test_positions_only() {
  Point a, b, c;
  b.time = 1;
  b.position.fill(.2);
  c.time = 3;
  c.position.fill(.1);
  const std::vector<Point> points{a, b, c};
  validate(points);
  // No artificial zero-velocity endpoint constraint for a positions-only message.
  validate_endpoint(c, Joints{1.875, 2, 2, 2, 2, 2});
  near(sample(points, .5).position[0], .1);
  near(sample(points, .5).velocity[0], .2);
  near(sample(points, 1).position[0], .2);
  near(sample(points, 2).position[0], .15);
  near(sample(points, 2).velocity[0], -.05);
  near(sample(points, 3).position[0], .1);
  near(sample(points, 3.01).position[0], .1);
  near(sample(points, 3.01).velocity[0], 0);
  // Linear segments have zero interior acceleration, not bounded acceleration at knots.
  near(sample(points, .5).acceleration[0], 0);
  near(sample(points, 2).acceleration[0], 0);
  c.derivatives = 1;
  rejects({a, b, c});
  c.derivatives = 2;
  rejects({a, b, c});
}

void test_advance_clock() {
  near(advance_seconds(0), 0);
  near(advance_seconds(330), .33);
  near(advance_seconds(1000), 1);
  for (double value : {-1., 1000.1, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected = false;
    try {
      advance_seconds(value);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "invalid advance accepted");
  }
  const double advance = advance_seconds(330);
  require(command_time(-.34, advance) < 0, "dispatch before pre-roll");
  near(command_time(-advance, advance), 0);
  Point first, last;
  last.time = 1.;
  last.position.fill(1.);
  const std::vector<Point> points{first, last};
  near(sample(points, command_time(-advance, advance)).position[0], 0);
  near(sample(points, command_time(0., advance)).position[0], .33);
  near(sample(points, 0.).position[0], 0); // Feedback stays on nominal clock.
  near(sample(points, command_time(.8, advance)).position[0], 1.);
  near(sample(points, .8).position[0], .8);
  near(sample(points, command_time(1., advance)).position[0], 1.);
  // Immediate goals anchor to the first tick, even if acceptance preceded it by a period.
  const double first_tick = .033;
  const double nominal_start = first_tick + advance;
  const double first_dispatch = command_time(first_tick - nominal_start, advance);
  const double second_dispatch = command_time(first_tick + .033 - nominal_start, advance);
  near(sample(points, first_dispatch).position[0], 0.);
  near(sample(points, second_dispatch).position[0], .033);
  // Explicit starts remain absolute: a late first tick is not forced back to q[0].
  near(sample(points, command_time(-advance + .01, advance)).position[0], .01);
}

int main() {
  const auto expect_diagnostic = [](const std::function<void()> &operation,
                                    const std::vector<std::string> &fields) {
    try {
      operation();
    } catch (const std::invalid_argument &error) {
      for (const auto &field : fields)
        require(std::string(error.what()).find(field) != std::string::npos,
                "Missing trajectory diagnostic context");
      return;
    }
    throw std::runtime_error("Expected trajectory validation failure");
  };
  Point bad;
  bad.time = 1;
  bad.position[2] = std::numeric_limits<double>::infinity();
  expect_diagnostic([&] { validate({bad}); },
                    {"point[0]", "time_from_start=1.", "SDK axis=3", "position=inf", "rad"});
  bad.position[2] = 0;
  expect_diagnostic([&] { validate({bad, bad}); },
                    {"point[1]", "strictly increasing", "previous=1."});
  bad.derivatives = 1;
  bad.velocity[4] = .2;
  expect_diagnostic([&] { validate_endpoint(bad, Joints{}); },
                    {"SDK axis=5", "velocity=0.2", "rad/s", "tolerance=1e-9"});

  test_advance_clock();
  test_positions_only();
  test_endpoint();
  test_joint_mapping();
  Point a, b;
  a.time = 0;
  b.time = 4;
  b.position.fill(1);
  validate({a, b});
  near(sample({a, b}, 2).position[0], .5);
  near(sample({a, b}, 2).velocity[0], .25);
  // Даже при пропусках тактов 4-секундная траектория заканчивается на elapsed=4, не 8.
  for (double t : {0., .033, .066, .163, 1., 2., 3.97, 4., 4.09}) {
    auto value = sample({a, b}, t);
    near(value.position[0], std::min(t / 4., 1.));
  }
  near(sample({a, b}, -1).position[0], 0);
  near(sample({a, b}, 5).velocity[0], 0);
  for (int order : {1, 2}) {
    a.derivatives = b.derivatives = order;
    near(sample({a, b}, 0).position[0], 0);
    near(sample({a, b}, 4).position[0], 1);
    near(sample({a, b}, 0).velocity[0], 0);
    near(sample({a, b}, 4).velocity[0], 0);
    near(sample({a, b}, 2).position[0], .5);
    if (order == 2) {
      near(sample({a, b}, 0).acceleration[0], 0);
      near(sample({a, b}, 4).acceleration[0], 0);
    }
  }
  // Ненулевые производные и неединичный период: проверка масштабирования h и h².
  a.velocity.fill(.3);
  b.velocity.fill(-.2);
  a.acceleration.fill(.4);
  b.acceleration.fill(-.1);
  near(sample({a, b}, 0).velocity[2], .3);
  near(sample({a, b}, 4).velocity[2], -.2);
  near(sample({a, b}, 0).acceleration[2], .4);
  near(sample({a, b}, 4).acceleration[2], -.1);
  Point c = b;
  c.time = 7;
  c.position.fill(2);
  near(sample({a, b, c}, 4).position[0], 1);
  near(sample({a, b, c}, 7).position[0], 2);
  near(sample({b}, 0).position[0], 1);
  rejects({});
  rejects({b, a});
  rejects({a, a});
  c = b;
  c.time = -1;
  rejects({c});
  c = b;
  c.time = std::numeric_limits<double>::quiet_NaN();
  rejects({a, c});
  c = b;
  c.position[1] = std::numeric_limits<double>::infinity();
  rejects({a, c});
  c = b;
  c.derivatives = 1;
  rejects({a, c});
  c = b;
  c.velocity[1] = std::numeric_limits<double>::infinity();
  rejects({a, c});
  c = b;
  c.acceleration[1] = std::numeric_limits<double>::infinity();
  rejects({a, c});
  std::cout << "PASS: 720 joint permutations, invalid joint names and mapped derivatives, "
               "absolute-time sampling, linear/cubic/quintic endpoints, derivatives, invalid "
               "input\n";
}
