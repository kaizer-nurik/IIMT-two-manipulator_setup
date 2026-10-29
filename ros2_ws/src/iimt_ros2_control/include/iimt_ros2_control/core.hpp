// Защита заголовка от повторного включения в одну единицу трансляции.
#pragma once
// Фиксированный массив шести суставов.
#include <array>
// Монотонные часы и преобразования длительности.
#include <chrono>
// Проверка конечности чисел и вычисление модулей.
#include <cmath>
// Целочисленный тип счётчика команд фиксированной ширины.
#include <cstdint>
// Границы представления чисел SDK.
#include <limits>
// Мьютекс обмена между контроллером и потоком SDK.
#include <mutex>
// Исключения для недопустимой конфигурации.
#include <stdexcept>
// Строковые параметры и описание нарушенного ограничения.
#include <string>

// Общее пространство имён аппаратного интерфейса.
namespace iimt_ros2_control {
// Шесть углов в порядке осей SDK; положения задаются в радианах.
using Joints = std::array<double, 6>;
// Монотонное время не зависит от коррекции системных часов.
using Clock = std::chrono::steady_clock;
// Настройки транспорта, расписания и защит движения; значения здесь являются исходными defaults.
struct Config {
  // Адрес робота, путь к библиотеке SDK и каталог межпроцессной блокировки.
  std::string robot_ip, sdk_library_path, lock_directory{"/tmp"};
  // Префикс суставов, имя ROS-ноды и группа совместного отказа нескольких рук.
  std::string joint_prefix, hardware_node_name{"iimt_hardware"}, coordination_group;
  // Максимальное ожидание общей блокировки SDK, секунды.
  double sdk_lock_timeout{0.006};
  // Имя сервиса отмены action контроллера.
  std::string controller_cancel_service{
      // Стандартный адрес CancelGoal для одноручного FollowJointTrajectory.
      "/iimt_group_controller/follow_joint_trajectory/_action/cancel_goal"};
  // Таймаут ожидания ответа на отмену цели, секунды.
  double controller_cancel_timeout{1.0};
  // Порт подключения SDK по умолчанию.
  int port{29999};
  // Приоритет рабочего потока; ноль оставляет обычное планирование ОС.
  int worker_priority{0};
  // При true отсутствие требуемого RT-планирования считается ошибкой.
  bool require_realtime{false};
  // Движение изначально запрещено и требует явного включения.
  bool enable_motion{false};
  // Параметры ServoJ из конфигурации; period и lookahead_time задаются в секундах.
  double servo_a{7}, servo_v{3}, period{0.008}, lookahead_time{0.012}, gain{0};
  // Период чтения, бюджет одного вызова SDK и допустимый возраст команды, секунды.
  double state_period{0.008}, sdk_budget{0.006}, command_timeout{0.1},
      // Максимально допустимый возраст измеренного состояния, секунды.
      state_timeout{0.1};
  // Допустимое отклонение интервала команд от периода ServoJ, секунды.
  double command_jitter_tolerance{0.004};
  // Таймаут старта, допустимое опоздание и порог расхождения часов ROS, секунды.
  double startup_timeout{10}, max_lateness{0.016}, ros_clock_jump{0.1},
      // Период публикации диагностики, секунды.
      diagnostics_period{1};
  // Нижние границы углов CI05 по шести осям, радианы.
  Joints min_position{{-6.2832, -2.6167, -2.8783, -3.0528, -3.0528, -6.2832}};
  // Верхние границы углов CI05 по шести осям, радианы.
  Joints max_position{{6.2832, 2.6167, 2.8783, 3.0528, 3.0528, 6.2832}};
  // Ограничения модулей скоростей суставов, радианы в секунду.
  Joints max_velocity{{1, 1, 1, 1, 1, 1}},
      // Ограничения модулей ускорений суставов, радианы в секунду в квадрате.
      max_acceleration{{1.875, 2, 2, 2, 2, 2}};
  // Максимальные приращения угла между проверяемыми командами, радианы.
  Joints max_step{{0.02, 0.02, 0.02, 0.02, 0.02, 0.02}},
      // Максимальное отклонение команды от измеренного положения, радианы.
      tracking_tolerance{{0.2, 0.2, 0.2, 0.2, 0.2, 0.2}};
  // Проверка согласованности настроек перед работой оборудования.
  void validate() const {
    // Проверяем диапазон разрешённых приоритетов потока.
    if (worker_priority < 0 || worker_priority > 99 ||
        // Обязательный RT-режим несовместим с нулевым приоритетом.
        (require_realtime && worker_priority == 0))
      // Прерываем настройку при неверном приоритете.
      throw std::invalid_argument("worker_priority must be 0..99; "
                                  // Уточняем требование ненулевого приоритета для обязательного RT.
                                  "require_realtime needs nonzero priority");
    // Адрес робота и путь SDK не должны быть пустыми.
    if (robot_ip.empty() || sdk_library_path.empty() ||
        // Каталог блокировки обязателен, номер порта должен входить в диапазон TCP.
        lock_directory.empty() || port < 1 || port > 65535)
      // Отклоняем неполную конфигурацию подключения.
      throw std::invalid_argument(
          // Сообщаем обязательные параметры подключения.
          "robot_ip, sdk_library_path, lock_directory and port required");
    // Последовательно проверяем числовые параметры, которым запрещён ноль.
    for (double x :
         // Включаем настройки ServoJ, периода чтения, бюджета SDK и возраста команды.
         {servo_a, servo_v, period, state_period, sdk_budget, command_timeout,
          // Включаем таймауты состояния, блокировки, запуска, опоздания и часов ROS.
          state_timeout, sdk_lock_timeout, startup_timeout, max_lateness, ros_clock_jump,
          // Завершаем перечень периодом диагностики, отменой цели и допустимым джиттером.
          diagnostics_period, controller_cancel_timeout, command_jitter_tolerance})
      // Отбрасываем NaN, бесконечность, отрицательные значения и ноль.
      if (!std::isfinite(x) || x <= 0)
        // Создаём исключение для некорректного положительного параметра.
        throw std::invalid_argument(
            // Объясняем требование конечного положительного значения.
            "timing/limit parameters must be finite and positive");
    // Рассчитываем безопасную верхнюю границу длительности.
    const double max_duration =
        // Оставляем четырёхкратный запас относительно максимума монотонных часов.
        std::chrono::duration<double>(Clock::duration::max()).count() / 4;
    // Проверяем представимость временных настроек в тиках часов.
    for (double x :
         // Проверяем периоды, бюджет вызова и таймауты команд и состояния.
         {period, state_period, sdk_budget, command_timeout, state_timeout,
          // Проверяем таймаут старта, блокировку, опоздание, часы ROS и диагностику.
          startup_timeout, sdk_lock_timeout, max_lateness, ros_clock_jump, diagnostics_period,
          // Включаем таймаут отмены action.
          controller_cancel_timeout})
      // Отбрасываем слишком большие длительности либо начинаем проверку разрешения часов.
      if (x > max_duration ||
          std::chrono::duration_cast<Clock::duration>(
              // Переводим секунды параметра в целочисленную длительность монотонных часов.
              std::chrono::duration<double>(x))
                  // Значение, округлившееся до нуля тиков, непригодно для расписания.
                  .count() <= 0)
        // Сообщаем о выходе длительности за представимый диапазон.
        throw std::invalid_argument("timing outside clock representation");
    // Джиттер не должен быть равен периоду или превышать его.
    if (command_jitter_tolerance >= period)
      // Отклоняем допуск, допускающий нулевой интервал соседних команд.
      throw std::invalid_argument("command_jitter_tolerance must be below servo.period");
    // Проверяем все значения, передаваемые в float-поля ServoJ.
    for (double x : {servo_a, servo_v, period, lookahead_time, gain})
      // Отбрасываем нечисловые, отрицательные и слишком большие значения.
      if (!std::isfinite(x) || x < 0 || x > std::numeric_limits<float>::max() ||
          // Отбрасываем ненулевые значения, исчезающие при преобразовании во float.
          (x != 0 && static_cast<float>(x) == 0))
        // Создаём исключение при потере представимости параметра SDK.
        throw std::invalid_argument(
            // Указываем, что значение ServoJ не представимо типом float.
            "ServoJ value exceeds SDK float representation");
    // Дополнительно проверяем разрешение периода ServoJ.
    if (std::chrono::duration_cast<Clock::duration>(
            // Переводим период команды из секунд в длительность часов.
            std::chrono::duration<double>(period))
                // Нулевое число тиков означает слишком короткий период.
                .count() <= 0 ||
        // Аналогично проверяем период чтения состояния.
        std::chrono::duration_cast<Clock::duration>(
            // Переводим период чтения в длительность монотонных часов.
            std::chrono::duration<double>(state_period))
                // Нулевое число тиков чтения также запрещено.
                .count() <= 0)
      // Сообщаем о периоде ниже разрешения используемых часов.
      throw std::invalid_argument("period below clock resolution");
    // Проверяем конечность gain и lookahead_time и неотрицательность gain.
    if (!std::isfinite(gain) || gain < 0 || !std::isfinite(lookahead_time) ||
        // Запрещаем отрицательное время упреждения.
        lookahead_time < 0)
      // Отклоняем недопустимые настройки ServoJ без предположений о диапазонах производителя.
      throw std::invalid_argument("invalid ServoJ tuning");
    // Проверяем ограничения каждой из шести осей отдельно.
    for (size_t i = 0; i < 6; ++i) {
      // Обе границы положения должны быть конечными.
      if (!std::isfinite(min_position[i]) || !std::isfinite(max_position[i]) ||
          // Нижняя граница должна быть строго меньше верхней.
          min_position[i] >= max_position[i])
        // Отклоняем некорректный диапазон углов сустава.
        throw std::invalid_argument("invalid position limits");
      // Проверяем скорость, ускорение и максимальный шаг текущей оси.
      for (double x : {max_velocity[i], max_acceleration[i], max_step[i],
                       // Включаем допустимую ошибку относительно измеренного положения.
                       tracking_tolerance[i]})
        // Каждое ограничение должно быть конечным и строго положительным.
        if (!std::isfinite(x) || x <= 0)
          // Отклоняем недопустимое ограничение движения сустава.
          throw std::invalid_argument("invalid joint limits");
      // Завершаем проверку ограничений текущего сустава.
    }
    // Завершаем проверку всей конфигурации.
  }
  // Завершаем описание структуры настроек.
};
// Проверяем, что заданные углы допустимы для всех осей.
inline bool valid_position(const Joints &q, const Config &c) {
  // Обходим все шесть положений.
  for (size_t i = 0; i < 6; ++i)
    // Проверяем конечность угла и нижнюю границу.
    if (!std::isfinite(q[i]) || q[i] < c.min_position[i] ||
        // Проверяем верхнюю границу угла.
        q[i] > c.max_position[i])
      // Любой недопустимый сустав делает всю позицию непригодной.
      return false;
  // Все суставы прошли проверку положения.
  return true;
  // Завершаем проверку позиции.
}
// Проверяем переход от предыдущей команды к новой позиции.
inline bool valid_step(
    const Joints &q, const Joints &previous,
    // Принимаем предыдущую скорость и измеренное положение для проверки ускорения и слежения.
    const Joints &velocity, const Joints &measured,
    // Интервал задаётся вызывающим кодом; reason необязателен и заполняется при нарушении лимитов шага.
    double dt, const Config &c, std::string *reason = nullptr) {
  // Сначала проверяем новую позицию и конечный положительный интервал времени.
  if (!valid_position(q, c) || !std::isfinite(dt) || dt <= 0)
    // Без корректной позиции или dt расчёт шага запрещён.
    return false;
  // Проверяем динамические ограничения каждого сустава.
  for (size_t i = 0; i < 6; ++i) {
    // Предыдущие положение и скорость должны быть конечными.
    if (!std::isfinite(previous[i]) || !std::isfinite(velocity[i]) ||
        // Измеренное положение также не должно содержать NaN или бесконечность.
        !std::isfinite(measured[i]))
      // Некорректные исходные данные делают переход недопустимым.
      return false;
    // Вычисляем среднюю скорость между командами на предоставленном интервале.
    const double v = (q[i] - previous[i]) / dt;
    // Проверяем максимальное изменение угла за один шаг.
    if (std::abs(q[i] - previous[i]) > c.max_step[i] ||
        // Проверяем модуль вычисленной скорости.
        std::abs(v) > c.max_velocity[i] ||
        // Проверяем модуль изменения скорости за интервал, то есть дискретное ускорение.
        std::abs(v - velocity[i]) / dt > c.max_acceleration[i] ||
        // Проверяем расстояние от новой команды до измеренного положения.
        std::abs(q[i] - measured[i]) > c.tracking_tolerance[i]) {
      // Подробное описание формируем только при переданном указателе reason.
      if (reason)
        // Начинаем описание с номера сустава от единицы и интервала времени.
        *reason = " J" + std::to_string(i + 1) + " dt=" + std::to_string(dt) +
                  // Добавляем фактический шаг положения.
                  " step=" + std::to_string(q[i] - previous[i]) +
                  // Добавляем вычисленную скорость перехода.
                  " velocity=" + std::to_string(v) +
                  // Добавляем скорость предыдущего перехода.
                  " previous_velocity=" + std::to_string(velocity[i]) +
                  // Добавляем установленный предел скорости.
                  " velocity_limit=" + std::to_string(c.max_velocity[i]) +
                  // Добавляем вычисленное ускорение со знаком.
                  " acceleration=" + std::to_string((v - velocity[i]) / dt) +
                  // Добавляем установленный предел модуля ускорения.
                  " acceleration_limit=" + std::to_string(c.max_acceleration[i]) +
                  // Добавляем установленный предел шага положения.
                  " step_limit=" + std::to_string(c.max_step[i]) +
                  // Добавляем отклонение команды от измеренного положения.
                  " tracking=" + std::to_string(q[i] - measured[i]) +
                  // Завершаем описание установленным допуском слежения.
                  " tracking_limit=" + std::to_string(c.tracking_tolerance[i]);
      // Нарушение любого динамического ограничения запрещает переход.
      return false;
      // Завершаем обработку нарушения ограничений.
    }
    // Завершаем проверку текущего сустава.
  }
  // Все шесть суставов допускают этот переход.
  return true;
  // Завершаем функцию проверки шага.
}
// Проверяем возраст записи по монотонным меткам времени.
inline bool stale(Clock::time_point stamp, Clock::time_point now,
                  // Принимаем предельный возраст в секундах.
                  double timeout) {
  // Неинициализированная временная метка сразу считается устаревшей.
  return stamp == Clock::time_point{} ||
         // Иначе сравниваем фактический возраст с таймаутом.
         std::chrono::duration<double>(now - stamp).count() > timeout;
  // Завершаем проверку свежести.
}
// Вычисляем следующий слот относительно предыдущего планового времени.
inline Clock::time_point
next_slot(Clock::time_point previous,
          // Принимаем текущий момент для пропуска уже прошедших слотов.
          Clock::time_point now,
          // Используем длительность периода и накопительный счётчик пропусков.
          Clock::duration period, uint64_t &skipped) {
  // Период расписания должен быть строго положительным.
  if (period <= Clock::duration::zero())
    // Отклоняем нулевой или отрицательный шаг расписания.
    throw std::invalid_argument("nonpositive scheduling period");
  // Сначала рассматриваем непосредственно следующий слот.
  auto next = previous + period;
  // Если слот уже наступил, переносим его в будущее.
  if (next <= now) {
    // Вычисляем число пропускаемых слотов, включая совпадающий с текущим временем.
    const auto n = (now - next) / period + 1;
    // Увеличиваем диагностический счётчик пропущенных слотов.
    skipped += static_cast<uint64_t>(n);
    // Сдвигаем плановое время сразу на нужное число периодов, без догоняющих вызовов.
    next += period * n;
    // Завершаем перенос просроченного слота.
  }
  // Возвращаем строго будущий слот расписания.
  return next;
  // Завершаем расчёт слота.
}
// Once a controller owns commands, its writes are the only polling clock,
// including the interval before its first write. Idle and post-stop polling
// remain independent. A read already inside the SDK cannot be interrupted.
// Определяем, разрешён ли опрос состояния в этой итерации рабочего потока.
inline bool feedback_poll_allowed(bool claimed, bool sent_this_iteration) {
  // Без владельца команд опрос независим; при владении разрешён только после новой отправки.
  return !claimed || sent_this_iteration;
  // Завершаем правило согласования чтения и записи.
}

// Одна команда положения с данными о её свежести и последовательности.
struct Command {
  // Целевые углы шести суставов, изначально нулевые.
  Joints position{};
  // Монотонная метка поступления команды в mailbox.
  Clock::time_point stamp{};
  // Номер записи позволяет отличить новую команду от повторного чтения старой.
  uint64_t sequence{0};
  // Временная метка источника команды в секундах, отдельно от монотонного времени приёма.
  double source_time{0};
  // Завершаем структуру одной команды.
};
// Хранилище только последней команды, а не очередь накопленных точек.
class Mailbox {
  // Открываем операции записи и чтения mailbox.
public:
  // Записываем новые углы, момент поступления и время источника.
  void put(const Joints &q, Clock::time_point now, double source_time = 0) {
    // Захватываем мьютекс на время согласованного обновления всех полей.
    std::lock_guard<std::mutex> l(m_);
    // Сохраняем новые целевые углы.
    value_.position = q;
    // Сохраняем монотонную метку получения.
    value_.stamp = now;
    // Сохраняем отдельное время источника команды.
    value_.source_time = source_time;
    // Увеличиваем последовательность даже при неизменных углах новой записи.
    ++value_.sequence;
    // Завершаем запись; lock_guard автоматически освобождает мьютекс.
  }
  // Получаем согласованную копию последней команды.
  Command get() const {
    // Блокируем одновременную запись на время копирования.
    std::lock_guard<std::mutex> l(m_);
    // Возвращаем копию команды, не ссылку на изменяемую память.
    return value_;
    // Завершаем чтение; мьютекс автоматически освобождается.
  }

  // Скрываем внутреннее состояние mailbox от прямого доступа.
private:
  // mutable позволяет защищать мьютексом логически константную операцию get.
  mutable std::mutex m_;
  // Единственная последняя команда; промежуточные записи не образуют очередь.
  Command value_;
  // Завершаем класс обмена командами.
};
// Абстрактный транспорт отделяет рабочий поток от реального SDK и тестовой реализации.
class Transport {
  // Открываем интерфейс транспортных операций.
public:
  // Виртуальный деструктор обеспечивает удаление реализации через базовый указатель.
  virtual ~Transport() = default;
  // Реализация должна подключиться с переданной конфигурацией.
  virtual bool connect(const Config &) = 0;
  // Реализация должна заполнить шесть измеренных углов и сообщить успех.
  virtual bool read(Joints &) = 0;
  // Реализация должна отправить заданные углы с параметрами ServoJ.
  virtual bool send(const Joints &, const Config &) = 0;
  // Реализация должна запросить остановку и сообщить результат.
  virtual bool stop() = 0;
  // Реализация должна освободить подключения транспорта.
  virtual void disconnect() = 0;
  // Завершаем абстрактный интерфейс транспорта.
};
// namespace iimt_ros2_control // Завершаем пространство имён аппаратного интерфейса.
} // namespace iimt_ros2_control
