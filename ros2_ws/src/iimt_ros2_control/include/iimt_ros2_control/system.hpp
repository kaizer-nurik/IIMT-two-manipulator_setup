// Защищает заголовок от повторного включения в одну единицу трансляции.
#pragma once
// Подключает конфигурацию, типы суставов, почтовый ящик и интерфейс SDK-транспорта.
#include "iimt_ros2_control/core.hpp"
// Подключает ROS-сервис отмены целей action-контроллера.
#include <action_msgs/srv/cancel_goal.hpp>
// Подключает атомарные переменные для обмена между потоками.
#include <atomic>
// Подключает ожидание событий без постоянного опроса процессором.
#include <condition_variable>
// Подключает сообщение массива диагностических статусов.
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
// Подключает базовый аппаратный интерфейс ros2_control.
#include <hardware_interface/system_interface.hpp>
// Подключает умные указатели и управление временем жизни объектов.
#include <memory>
// Подключает необязательное значение для ожидающего ответа на отмену.
#include <optional>
// Подключает C++ API ROS 2: ноды, время, клиенты и исполнители.
#include <rclcpp/rclcpp.hpp>
// Подключает отдельные потоки SDK и ROS-исполнителя.
#include <thread>

// Открывает пространство имён аппаратного пакета.
namespace iimt_ros2_control {
// Объявляет аппаратную систему с контрактом SystemInterface ros2_control.
class IimtSystem : public hardware_interface::SystemInterface {
  // Начинает публичные методы, вызываемые ros2_control.
public:
  // Переопределяет деструктор для завершения потоков и освобождения ресурсов.
  ~IimtSystem() override;
  // Задаёт тип результата lifecycle-инициализации.
  hardware_interface::CallbackReturn
  // Принимает описание оборудования и проверяет объявленные интерфейсы.
  on_init(const hardware_interface::HardwareInfo &) override;
  // Задаёт тип результата конфигурирования оборудования.
  hardware_interface::CallbackReturn
  // Объявляет настройку параметров, ресурсов и соединения SDK.
  on_configure(const rclcpp_lifecycle::State &) override;
  // Задаёт тип результата активации оборудования.
  hardware_interface::CallbackReturn
  // Объявляет переход оборудования в активное состояние.
  on_activate(const rclcpp_lifecycle::State &) override;
  // Задаёт тип результата деактивации оборудования.
  hardware_interface::CallbackReturn
  // Объявляет снятие активности и запрос остановки при необходимости.
  on_deactivate(const rclcpp_lifecycle::State &) override;
  // Задаёт тип результата очистки ресурсов.
  hardware_interface::CallbackReturn
  // Объявляет очистку после деактивации.
  on_cleanup(const rclcpp_lifecycle::State &) override;
  // Задаёт тип результата окончательного завершения.
  hardware_interface::CallbackReturn
  // Объявляет завершение работы аппаратной системы.
  on_shutdown(const rclcpp_lifecycle::State &) override;
  // Задаёт тип результата обработки lifecycle-ошибки.
  hardware_interface::CallbackReturn
  // Объявляет фиксацию ошибки без ожидания зависшего SDK-вызова.
  on_error(const rclcpp_lifecycle::State &) override;
  // Задаёт список экспортируемых интерфейсов измеренного состояния.
  std::vector<hardware_interface::StateInterface>
  // Предоставляет ros2_control доступ к массивам положения и скорости.
  export_state_interfaces() override;
  // Задаёт список экспортируемых командных интерфейсов.
  std::vector<hardware_interface::CommandInterface>
  // Предоставляет контроллеру шесть команд положения суставов.
  export_command_interfaces() override;
  // Объявляет чтение сохранённых измерений в цикле controller_manager.
  hardware_interface::return_type
  read(const rclcpp::Time &,
       // Принимает длительность цикла; метод переопределяет аппаратный контракт.
       const rclcpp::Duration &) override;
  // Объявляет передачу команды из controller_manager в почтовый ящик.
  hardware_interface::return_type
  write(const rclcpp::Time &,
        // Принимает длительность цикла; сам write не вызывает сетевой SDK.
        const rclcpp::Duration &) override;
  // Задаёт код успешности предварительной проверки переключения интерфейсов.
  hardware_interface::return_type
  // Принимает список запускаемых командных интерфейсов для проверки.
  prepare_command_mode_switch(
      const std::vector<std::string> &,
      // Принимает список останавливаемых интерфейсов; состояние ещё не переключается.
      const std::vector<std::string> &) override;
  // Задаёт код успешности фактического переключения интерфейсов.
  hardware_interface::return_type
  // Принимает список командных интерфейсов, передаваемых контроллеру.
  perform_command_mode_switch(
      const std::vector<std::string> &,
      // Принимает список освобождаемых интерфейсов и завершает объявление метода.
      const std::vector<std::string> &) override;

  // Начинает внутренние методы и состояние реализации.
private:
  // Локальная история единственного SDK-потока; не разделяется с ROS callbacks.
  struct WorkerState;
  // Пишем причину перед каждым lifecycle ERROR.
  hardware_interface::CallbackReturn callback_error(const std::string &reason) const;
  // Настройка планировщика потока и Login.
  void connect_worker();
  // Учёт длительности и проверка завершённого вызова.
  bool check_sdk_budget(Clock::time_point begin, bool is_read, bool armed_at_start, bool succeeded);
  // Проверка периода источника и фактической отправки.
  bool valid_command_interval(double dt) const;
  // Обработка отложенного Stop.
  bool service_worker_stop(WorkerState &state);
  // Проверка и отправка свежей команды.
  bool dispatch_worker_command(WorkerState &state, bool &sent_this_iteration);
  // Чтение и публикация проверенного измерения.
  bool poll_worker_feedback(WorkerState &state, bool sent_this_iteration);
  // Ожидание срока либо события.
  void wait_worker_event(const WorkerState &state);
  // Финальный Stop и отключение SDK.
  void disconnect_worker();
  // Объявляет рабочий цикл единственного владельца вызовов SDK.
  void run();
  // Объявляет завершение потоков и освобождение ресурсов; SDK может задержать выход.
  void shutdown();
  // Объявляет асинхронный запрос остановки для рабочего потока.
  void request_stop();
  // Объявляет фиксацию аппаратного отказа с текстовой причиной.
  void fault(const std::string &);
  // Объявляет загрузку и проверку стартовых параметров ROS-ноды.
  void parameters();
  // Объявляет публикацию диагностик и обслуживание запроса отмены цели.
  void diagnostics();
  // Выбирает из списка только интерфейсы суставов этой аппаратной системы.
  std::vector<std::string> owned_keys(const std::vector<std::string> &) const;
  // Проверяет наличие полного набора шести командных интерфейсов положения.
  bool complete_claim(const std::vector<std::string> &) const;
  // Вводит короткое имя типа клиента сервиса отмены action-целей.
  using CancelClient = rclcpp::Client<action_msgs::srv::CancelGoal>;
  // Хранит клиента отмены цели траекторного контроллера.
  CancelClient::SharedPtr cancel_client_;
  // Хранит незавершённый запрос отмены и его идентификатор, если запрос отправлен.
  std::optional<CancelClient::FutureAndRequestId> cancel_pending_;
  // Хранит монотонное время отправки отмены для контроля её таймаута.
  Clock::time_point cancel_sent_at_{};
  // Хранит факт запроса и принятия отмены; это не подтверждение физической остановки.
  bool cancel_requested_{false}, cancel_accepted_{false};
  // Задаёт начальный текстовый статус отмены для диагностики.
  std::string cancel_status_{"not_requested"};
  // Объявляет неблокирующее обслуживание отмены цели контроллера.
  void cancel_controller();
  // Объявляет проверку общего отказа координационной группы.
  bool group_faulted() const;
  // Разделяет атомарный флаг отказа между аппаратными системами группы.
  std::shared_ptr<std::atomic<bool>> group_fault_;
  // Хранит проверенную конфигурацию соединения, таймингов и ограничений.
  Config config_;
  // Хранит экспортируемое состояние, команды и последнее измерение SDK.
  Joints position_{}, velocity_{}, command_{}, feedback_{},
      // Хранит скорость, вычисленную по последовательным измерениям положения.
      feedback_velocity_{};
  // Хранит время последнего успешного измерения для проверки свежести.
  Clock::time_point feedback_stamp_{};
  // Разделяет блокировки измерений, текста отказа и ожидания команд.
  std::mutex state_mutex_, fault_mutex_, wait_mutex_;
  // Позволяет будить рабочий поток при команде, остановке или завершении.
  std::condition_variable wake_;
  // Хранит единственный актуальный образец команды вместо очереди траектории.
  Mailbox mailbox_;
  // Владеет реализацией адаптера SDK через абстрактный Transport.
  std::unique_ptr<Transport> transport_;
  // Хранит поток SDK и отдельный поток обслуживания ROS-ноды.
  std::thread worker_, executor_thread_;
  // Объявляет атомарные признаки завершения, отказа и активности оборудования.
  std::atomic<bool> terminate_{false}, faulted_{false}, active_{false},
      // Хранит владение командами, запрос и ожидание остановки, наличие новой команды.
      claimed_{false}, stop_requested_{false}, stop_pending_{false}, command_ready_{false};
  // Различает неудачу остановки и необходимость выполнить Stop после владения командами.
  std::atomic<bool> stop_failed_{false}, needs_stop_{false};
  // Хранит последнюю и максимальную измеренные длительности SDK-вызова в секундах.
  std::atomic<double> last_sdk_duration_{0}, max_sdk_duration_{0},
      // Хранит интервал отправок, возраст команды при отправке и задержку после проверок.
      last_send_interval_{0}, last_command_age_at_send_{0}, last_guard_to_send_delay_{0},
      // Хранит максимальный наблюдавшийся возраст команды при проверке отправки.
      max_lateness_observed_{0};
  // Хранит последнюю и максимальную длительности чтения состояния SDK.
  std::atomic<double> last_read_duration_{0}, max_read_duration_{0},
      // Хранит последнюю и максимальную длительности вызова ServoJ.
      last_servoj_duration_{0}, max_servoj_duration_{0};
  // Считает чтения SDK и превышения их временного бюджета.
  std::atomic<uint64_t> read_calls_{0}, read_budget_overruns_{0},
      // Считает медленные чтения без владения командами, вызовы ServoJ и их превышения.
      unclaimed_read_overruns_{0}, servoj_calls_{0}, servoj_budget_overruns_{0};
  // Считает пропущенные слоты чтения, успешные отправки и потерянные образцы команд.
  std::atomic<uint64_t> skipped_{0}, sent_{0}, dropped_{0};
  // Хранит текст зафиксированного отказа под защитой fault_mutex_.
  std::string fault_text_;
  // Хранит дескриптор межпроцессной блокировки; -1 означает отсутствие открытого файла.
  int lock_fd_{-1};
  // Хранит последнюю и опорную ROS-метки для обнаружения скачков часов.
  double last_ros_time_{0}, anchor_ros_time_{0};
  // Хранит соответствующие монотонные метки записи команды.
  Clock::time_point last_write_{}, anchor_write_{};
  // Владеет вспомогательной ROS-нодой параметров и диагностики оборудования.
  std::shared_ptr<rclcpp::Node> node_;
  // Владеет однопоточным исполнителем callback-функций вспомогательной ноды.
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  // Задаёт тип издателя диагностического массива ROS.
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      // Хранит экземпляр издателя аппаратной диагностики.
      diagnostic_pub_;
  // Хранит периодический таймер публикации диагностик.
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  // Завершает объявление аппаратной системы.
};
// namespace iimt_ros2_control // Закрывает пространство имён пакета.
} // namespace iimt_ros2_control
