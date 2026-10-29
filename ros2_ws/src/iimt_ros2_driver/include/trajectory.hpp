#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace iimt_ros2_driver::trajectory {
using Joints = std::array<double, 6>;

// Advance affects dispatch only; feedback and deadlines retain the nominal clock.
inline double advance_seconds(double milliseconds) {
  if (!std::isfinite(milliseconds) || milliseconds < 0 || milliseconds > 1000)
    throw std::invalid_argument("trajectory_advance_ms=" + std::to_string(milliseconds) +
                                " must be finite and within 0..1000 ms");
  return milliseconds / 1000.;
}

inline double command_time(double nominal_elapsed, double advance) {
  return nominal_elapsed + advance;
}

// Один узел исходной траектории; derivatives: 0=q, 1=q/dq, 2=q/dq/ddq.
struct Point {
  double time{};
  Joints position{}, velocity{}, acceleration{};
  int derivatives{};
};

// Для каждой оси SDK сохраняем индекс соответствующего сустава во входном сообщении.
inline std::array<size_t, 6> joint_order(const std::vector<std::string> &expected,
                                         const std::vector<std::string> &received) {
  const auto format_names = [](const std::vector<std::string> &names) {
    std::string result = "[";
    for (size_t i = 0; i < names.size(); ++i)
      result += (i ? ", " : "") + names[i];
    return result + "]";
  };
  const auto reject = [&]() {
    throw std::invalid_argument("Expected exactly six unique supported joints; expected " +
                                format_names(expected) + ", received " + format_names(received));
  };
  if (expected.size() != 6 || received.size() != 6)
    reject();
  std::array<size_t, 6> order{};
  for (size_t axis = 0; axis < order.size(); ++axis) {
    if (std::count(expected.begin(), expected.end(), expected[axis]) != 1 ||
        std::count(received.begin(), received.end(), expected[axis]) != 1)
      reject();
    order[axis] = std::find(received.begin(), received.end(), expected[axis]) - received.begin();
  }
  return order;
}

// q, dq и ddq переставляются вместе до проверок лимитов и начальной позиции.
inline Point reorder(const Point &input, const std::array<size_t, 6> &order) {
  Point result = input;
  for (size_t axis = 0; axis < order.size(); ++axis) {
    result.position[axis] = input.position.at(order[axis]);
    result.velocity[axis] = input.velocity.at(order[axis]);
    result.acceleration[axis] = input.acceleration.at(order[axis]);
  }
  return result;
}

inline void validate(const std::vector<Point> &points) {
  if (points.empty())
    throw std::invalid_argument(
        "empty trajectory: received 0 points; at least one point is required");
  const int derivatives = points.front().derivatives;
  if (derivatives < 0 || derivatives > 2)
    throw std::invalid_argument("point[0] derivative level=" + std::to_string(derivatives) +
                                "; expected 0, 1 or 2");
  double previous = -1.0;
  for (size_t index = 0; index < points.size(); ++index) {
    const auto &point = points[index];
    const std::string context = "point[" + std::to_string(index) +
                                "] time_from_start=" + std::to_string(point.time) + " s: ";
    if (!std::isfinite(point.time) || point.time < 0 || point.time <= previous)
      throw std::invalid_argument(
          context +
          "trajectory times must be finite, nonnegative and strictly increasing; previous=" +
          std::to_string(previous) + " s");
    if (point.derivatives != derivatives)
      throw std::invalid_argument(context +
                                  "derivative level=" + std::to_string(point.derivatives) +
                                  "; expected " + std::to_string(derivatives) + " from point[0]");
    for (std::size_t axis = 0; axis < 6; ++axis) {
      if (!std::isfinite(point.position[axis]) ||
          (derivatives >= 1 && !std::isfinite(point.velocity[axis])) ||
          (derivatives == 2 && !std::isfinite(point.acceleration[axis])))
        throw std::invalid_argument(
            context + "nonfinite trajectory value at SDK axis=" + std::to_string(axis + 1) +
            "; position=" + std::to_string(point.position[axis]) +
            " rad, velocity=" + std::to_string(point.velocity[axis]) +
            " rad/s, acceleration=" + std::to_string(point.acceleration[axis]) + " rad/s^2");
    }
    previous = point.time;
  }
}

// Ускорение конечной точки описывает входящий сегмент; после конца sample() удерживает
// позицию с нулевыми производными. Ненулевую конечную скорость удержание не допускает.
inline void validate_endpoint(const Point &point, const Joints &max_acceleration) {
  for (size_t axis = 0; axis < 6; ++axis) {
    if (point.derivatives >= 1 &&
        (!std::isfinite(point.velocity[axis]) || std::abs(point.velocity[axis]) > 1e-9))
      throw std::invalid_argument(
          "Final velocity must be zero; SDK axis=" + std::to_string(axis + 1) +
          ", time_from_start=" + std::to_string(point.time) +
          " s, velocity=" + std::to_string(point.velocity[axis]) +
          " rad/s, tolerance=1e-9 rad/s; provide explicit lead-out");
    if (point.derivatives == 2 && (!std::isfinite(point.acceleration[axis]) ||
                                   std::abs(point.acceleration[axis]) > max_acceleration[axis]))
      throw std::invalid_argument(
          "Final acceleration exceeds limits; SDK axis=" + std::to_string(axis + 1) +
          ", time_from_start=" + std::to_string(point.time) +
          " s, acceleration=" + std::to_string(point.acceleration[axis]) +
          " rad/s^2, absolute limit=" + std::to_string(max_acceleration[axis]) + " rad/s^2");
  }
}

// Коэффициенты в нормализованном времени u=(t-t0)/h: линейная, кубическая или квинтика.
inline std::array<double, 6> coefficients(const Point &left, const Point &right, std::size_t axis) {
  const double h = right.time - left.time;
  const double delta = right.position[axis] - left.position[axis];
  std::array<double, 6> c{left.position[axis], delta, 0, 0, 0, 0};
  if (left.derivatives == 0)
    return c;
  c[1] = left.velocity[axis] * h;
  if (left.derivatives == 1) {
    c[2] = 3 * delta - 2 * c[1] - right.velocity[axis] * h;
    c[3] = -2 * delta + c[1] + right.velocity[axis] * h;
    return c;
  }
  c[2] = 0.5 * left.acceleration[axis] * h * h;
  const double d = delta - c[1] - c[2];
  const double e = right.velocity[axis] * h - c[1] - 2 * c[2];
  const double f = right.acceleration[axis] * h * h - 2 * c[2];
  c[3] = 10 * d - 4 * e + 0.5 * f;
  c[4] = -15 * d + 7 * e - f;
  c[5] = 6 * d - 3 * e + 0.5 * f;
  return c;
}

// Вычисляем задание по абсолютному elapsed, а не по числу ранее отправленных пакетов.
// validate() вызывается при принятии цели, не на каждом такте.
inline Point sample(const std::vector<Point> &points, double elapsed) {
  if (points.empty() || !std::isfinite(elapsed))
    throw std::invalid_argument(
        "cannot sample trajectory: points=" + std::to_string(points.size()) +
        ", elapsed=" + std::to_string(elapsed) + " s; require nonempty points and finite elapsed");
  if (elapsed < points.front().time || elapsed > points.back().time || points.size() == 1) {
    Point result = elapsed < points.front().time ? points.front() : points.back();
    result.time = elapsed;
    result.velocity.fill(0);
    result.acceleration.fill(0);
    return result;
  }
  auto right = std::upper_bound(points.begin(), points.end(), elapsed,
                                [](double t, const Point &p) { return t < p.time; });
  if (right == points.end())
    right = points.end() - 1;
  const auto &left = *(right - 1);
  const double h = right->time - left.time;
  const double u = (elapsed - left.time) / h;
  Point result;
  result.time = elapsed;
  result.derivatives = left.derivatives;
  for (std::size_t axis = 0; axis < 6; ++axis) {
    const auto c = coefficients(left, *right, axis);
    result.position[axis] = ((((c[5] * u + c[4]) * u + c[3]) * u + c[2]) * u + c[1]) * u + c[0];
    result.velocity[axis] = (((5 * c[5] * u + 4 * c[4]) * u + 3 * c[3]) * u + 2 * c[2]) * u + c[1];
    result.acceleration[axis] = ((20 * c[5] * u + 12 * c[4]) * u + 6 * c[3]) * u + 2 * c[2];
    result.velocity[axis] /= h;
    result.acceleration[axis] /= h * h;
  }
  return result;
}
} // namespace iimt_ros2_driver::trajectory
