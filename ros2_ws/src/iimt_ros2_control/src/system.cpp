// Объявление класса аппаратного интерфейса и его внутренних типов.
#include "iimt_ros2_control/system.hpp"
// Алгоритмы копирования, подсчёта и поиска максимума.
#include <algorithm>
// Проверка символов IP при построении имени файла блокировки.
#include <cctype>
// Преобразование кодов ошибок POSIX в текст через strerror.
#include <cstring>
// Флаги открытия файла исключительного владения роботом.
#include <fcntl.h>
// Стандартные определения типов интерфейсов ros2_control.
#include <hardware_interface/types/hardware_interface_type_values.hpp>
// Значение NaN для ещё не полученных измерений.
#include <limits>
// Таблица общих аварийных флагов по имени группы.
#include <map>
// Макрос регистрации аппаратного плагина для pluginlib.
#include <pluginlib/class_list_macros.hpp>
// Настройка приоритета рабочего потока через POSIX Threads.
#include <pthread.h>
// Межпроцессная блокировка файла функцией flock.
#include <sys/file.h>
// Закрытие файлового дескриптора функцией close.
#include <unistd.h>

// Пространство имён аппаратного драйвера IIMT.
namespace iimt_ros2_control {
// Фабрика транспорта: реализация SDK создаётся отдельно от ROS-интерфейса.
std::unique_ptr<Transport> make_transport();
// Локальные для единицы трансляции средства синхронизации.
namespace {
// SDK thread safety is undocumented, including calls on different robot handles.
// Общий мьютекс сериализует SDK-вызовы всех рук в этом процессе.
std::timed_mutex sdk_gate;
// Защищает создание и поиск записей групп координации.
std::mutex groups_mutex;
// Имя группы связывается с общим атомарным флагом аварии.
std::map<std::string, std::shared_ptr<std::atomic<bool>>> groups;
// Получить или создать сохраняемый флаг аварии указанной группы.
std::shared_ptr<std::atomic<bool>> coordination_latch(const std::string &name) {
  // Пустое имя отключает координацию с другими экземплярами.
  if (name.empty())
    return {};
  // Блокируем таблицу на время поиска и возможного добавления записи.
  std::lock_guard<std::mutex> lock(groups_mutex);
  // Получаем ссылку на запись; отсутствующая запись создаётся пустой.
  auto &value = groups[name];
  // Новая группа начинает работу без зафиксированной аварии.
  if (!value)
    value = std::make_shared<std::atomic<bool>>(false);
  // Retained until process exit; lifecycle cleanup cannot reset it. // Возвращаем общий флаг; таблица хранит его до завершения процесса.
  return value;
  // Завершение получения флага координации.
}
// Попытка захватить общий доступ к SDK за заданное число секунд.
std::unique_lock<std::timed_mutex> sdk_lock(double seconds) {
  // Создаём RAII-блокировку, пока не захватывая сам мьютекс.
  std::unique_lock<std::timed_mutex> lock(sdk_gate, std::defer_lock);
  // Ограничиваем ожидание захвата, но не длительность последующего SDK-вызова.
  lock.try_lock_for(std::chrono::duration<double>(seconds));
  // Вызывающий код обязан проверить owns_lock перед обращением к SDK.
  return lock;
  // Завершение функции ограниченного ожидания SDK.
}
// Конец локального пространства имён средств синхронизации.
} // namespace
// Проверка общей аварии без изменения состояния группы.
bool IimtSystem::group_faulted() const {
  // Без группы аварии нет; иначе читаем её атомарный флаг.
  return group_fault_ && group_fault_->load();
  // Завершение проверки групповой аварии.
}
// Короткое имя результата callback жизненного цикла.
using Callback = hardware_interface::CallbackReturn;
// Короткое имя результата циклических методов ros2_control.
using Return = hardware_interface::return_type;
// Каждый lifecycle ERROR получает отдельное сообщение.
Callback IimtSystem::callback_error(const std::string &reason) const {
  // Логирование работает и до создания ноды.
  RCLCPP_ERROR(node_ ? node_->get_logger() : rclcpp::get_logger("iimt_hardware"), "%s",
               reason.c_str());
  // Единственная точка возврата аппаратной lifecycle-ошибки.
  return Callback::ERROR;
}
// Деструктор останавливает потоки и освобождает ресурсы.
IimtSystem::~IimtSystem() {
  shutdown();
}
// Инициализация структуры интерфейса из описания ros2_control.
Callback IimtSystem::on_init(const hardware_interface::HardwareInfo &info) {
  // Базовый класс сначала должен успешно принять HardwareInfo.
  if (SystemInterface::on_init(info) != Callback::SUCCESS)
    // Ошибка базовой инициализации запрещает загрузку интерфейса.
    return callback_error("on_init: SystemInterface base initialization failed");
  // Читаем идентичность экземпляра до проверки имён суставов.
  for (const auto *key : {"joint_prefix", "hardware_node_name", "coordination_group"}) {
    // Ищем очередной идентификационный параметр в HardwareInfo.
    auto it = info_.hardware_parameters.find(key);
    // Если параметр отсутствует, сохраняем значение конфигурации по умолчанию.
    if (it == info_.hardware_parameters.end())
      continue;
    // Префикс отличает имена суставов левой и правой рук.
    if (std::string(key) == "joint_prefix")
      config_.joint_prefix = it->second;
    // Имя отдельной ROS-ноды определяет область параметров и диагностики.
    else if (std::string(key) == "hardware_node_name")
      config_.hardware_node_name = it->second;
    // Оставшийся ключ задаёт имя группы взаимной аварийной блокировки.
    else
      config_.coordination_group = it->second;
    // Завершение чтения идентичности экземпляра.
  }
  // Поддерживается ровно шесть осей SDK для одного манипулятора.
  if (info_.joints.size() != 6)
    // Иное число суставов отвергается до подключения к роботу.
    return callback_error("on_init: expected exactly six joints");
  // Первые два имени фиксируют соответствие суставов осям J1 и J2.
  const std::array<std::string, 6> axes{{"shoulder_joint", "upper_arm_joint",
                                         // Остальные имена задают порядок J3–J6 без перестановок.
                                         "forearm_joint", "wrist_1_joint", "wrist_2_joint",
                                         "wrist_3_joint"}};
  // Проверяем описание каждой оси в порядке массива SDK.
  for (size_t axis = 0; axis < axes.size(); ++axis) {
    // Берём описание текущего сустава из HardwareInfo.
    const auto &j = info_.joints[axis];
    // Имя должно совпадать с ожидаемым именем вместе с префиксом.
    if (j.name != config_.joint_prefix + axes[axis]) {
      // Начинаем сообщение об ошибке отображения суставов.
      RCLCPP_ERROR(rclcpp::get_logger("iimt_hardware"),
                   // В сообщении показываем имя, нарушающее порядок SDK.
                   "Joint order must match SDK axes J1..J6; unexpected %s", j.name.c_str());
      // Неправильное отображение осей делает интерфейс неработоспособным.
      return callback_error("on_init: joint order does not match SDK axes: " + j.name);
      // Завершение обработки несовпадения имени сустава.
    }
    // Для сустава допускается единственный командный интерфейс.
    if (j.command_interfaces.size() != 1 ||
        // Этот командный интерфейс обязан управлять положением.
        j.command_interfaces[0].name != "position" ||
        // Интерфейсов измеренного состояния должно быть ровно два.
        j.state_interfaces.size() != 2)
      // Несовместимое описание интерфейсов отвергается.
      return callback_error(
          "on_init: joint requires one position command and two state interfaces: " + j.name);
    // Флаги обнаружения состояния положения и скорости.
    bool p = false, v = false;
    // Перебираем заявленные интерфейсы состояния текущего сустава.
    for (const auto &s : j.state_interfaces) {
      // Отмечаем наличие измеренного положения.
      p |= s.name == "position";
      // Отмечаем наличие измеренной скорости.
      v |= s.name == "velocity";
      // Завершение обхода интерфейсов состояния.
    }
    // Оба требуемых вида состояния должны присутствовать.
    if (!p || !v)
      // Отсутствие положения или скорости считается ошибкой описания.
      return callback_error("on_init: missing position or velocity state interface: " + j.name);
    // Завершение проверки всех шести суставов.
  }
  // До первого измерения положения неизвестны, а не равны нулю.
  position_.fill(std::numeric_limits<double>::quiet_NaN());
  // Начальные значения скорости также помечаются неизвестными.
  velocity_ = position_;
  // Команды остаются неопределёнными до получения исходного положения.
  command_ = position_;
  // Структура интерфейса проверена; SDK ещё не подключён.
  return Callback::SUCCESS;
  // Завершение первичной инициализации интерфейса.
}
// Создание ROS-ноды и загрузка параметров аппаратного интерфейса.
void IimtSystem::parameters() {
  // Отдельные параметры создания ноды этого экземпляра.
  rclcpp::NodeOptions options;
  // Не наследуем глобальные аргументы controller_manager автоматически.
  options.use_global_arguments(false);
  // Ищем путь к YAML аппаратных параметров в HardwareInfo.
  auto file = info_.hardware_parameters.find("hardware_parameters_file");
  // Подключаем файл только при наличии непустого пути.
  if (file != info_.hardware_parameters.end() && !file->second.empty())
    // Передаём YAML через локальные ROS-аргументы создаваемой ноды.
    options.arguments({"--ros-args", "--params-file", file->second});
  // Собираем явные переопределения параметров из описания оборудования.
  std::vector<rclcpp::Parameter> overrides;
  // Перебираем строковые настройки, которые можно передать через HardwareInfo.
  for (const auto *key : {"robot_ip", "sdk_library_path", "joint_prefix", "hardware_node_name",
                          "coordination_group"}) {
    // Ищем значение очередной настройки в описании оборудования.
    auto it = info_.hardware_parameters.find(key);
    // Добавляем переопределение только для реально указанного ключа.
    if (it != info_.hardware_parameters.end())
      // Сохраняем пару имени и строкового значения ROS-параметра.
      overrides.emplace_back(key, it->second);
    // Завершение сбора строковых переопределений.
  }
  // Разрешение движения обрабатывается отдельно как логическое значение.
  auto enabled = info_.hardware_parameters.find("enable_motion");
  // При отсутствии ключа далее останется безопасное значение false.
  if (enabled != info_.hardware_parameters.end()) {
    // Допускаем только точные строки true и false.
    if (enabled->second != "true" && enabled->second != "false")
      // Неоднозначное разрешение движения вызывает ошибку конфигурации.
      throw std::invalid_argument("enable_motion must be true/false");
    // Преобразуем проверенную строку в логический ROS-параметр.
    overrides.emplace_back("enable_motion", enabled->second == "true");
    // Завершение обработки разрешения движения.
  }
  // Передаём собранные переопределения создаваемой ноде.
  options.parameter_overrides(overrides);
  // Создаём аппаратную ноду с выбранным именем в корневом пространстве ROS.
  node_ = std::make_shared<rclcpp::Node>(config_.hardware_node_name, "/", options);
  // Дескриптор будет общим для объявляемых аппаратных параметров.
  rcl_interfaces::msg::ParameterDescriptor d;
  // Параметры доступны для чтения, но не меняются на работающем интерфейсе.
  d.read_only = true;
  // Identity must be selected before node creation and SDK-axis validation.
  // Перебираем идентификационные настройки и уже выбранные значения.
  for (const auto &entry : std::vector<std::pair<std::string, std::string>>{
           // Сопоставляем префикс суставов и имя аппаратной ноды.
           {"joint_prefix", config_.joint_prefix},
           {"hardware_node_name", config_.hardware_node_name},
           // Добавляем группу координации в список проверяемых настроек.
           {"coordination_group", config_.coordination_group}})
    // Проверяем, что итоговое ROS-значение совпадает с HardwareInfo.
    if (node_->declare_parameter<std::string>(entry.first, entry.second, d) != entry.second)
      // Не разрешаем YAML менять идентичность после создания ноды и проверки осей.
      throw std::invalid_argument(entry.first + " must be set via HardwareInfo, not YAML alone");
  // Подключаем общий аварийный флаг выбранной группы.
  group_fault_ = coordination_latch(config_.coordination_group);
  // Зафиксированная ранее групповая авария требует перезапуска процесса.
  if (group_faulted())
    throw std::runtime_error("coordination group fault is latched; restart required");
  // Вспомогательная лямбда объявляет вещественный параметр с текущим значением по умолчанию.
  auto real = [&](const std::string &n, double &value) {
    // Записываем полученное значение double обратно в конфигурацию.
    value = node_->declare_parameter<double>(n, value, d);
    // Завершение объявления загрузчика вещественных параметров.
  };
  // Вспомогательная лямбда загружает массив значений для шести суставов.
  auto array = [&](const std::string &n, Joints &value) {
    // Объявляем параметр как вектор вещественных чисел.
    auto v = node_->declare_parameter<std::vector<double>>(
        // Текущий массив служит значением по умолчанию; параметр доступен только для чтения.
        n, std::vector<double>(value.begin(), value.end()), d);
    // Размер массива должен соответствовать шести осям.
    if (v.size() != 6)
      // Иное число значений отвергается с указанием имени параметра.
      throw std::invalid_argument(n + " needs six values");
    // Копируем проверенный вектор в фиксированный массив конфигурации.
    std::copy(v.begin(), v.end(), value.begin());
    // Завершение объявления загрузчика суставных массивов.
  };
  // Загружаем IP робота; пустое значение далее отклонит проверка конфигурации.
  config_.robot_ip = node_->declare_parameter<std::string>("robot_ip", "", d);
  // Начинаем присваивание пути к динамической библиотеке SDK.
  config_.sdk_library_path =
      // Загружаем строковый путь к библиотеке производителя.
      node_->declare_parameter<std::string>("sdk_library_path", "", d);
  // Объявляем каталог файлов межпроцессной блокировки роботов.
  config_.lock_directory = node_->declare_parameter<std::string>(
      // Используем текущий каталог конфигурации как значение по умолчанию.
      "lock_directory", config_.lock_directory, d);
  // Загружаем порт подключения к SDK.
  config_.port = node_->declare_parameter<int>("port", config_.port, d);
  // Объявляем целочисленный приоритет рабочего потока.
  config_.worker_priority = node_->declare_parameter<int>(
      // Сохраняем настроенный приоритет или значение конфигурации по умолчанию.
      "worker_priority", config_.worker_priority, d);
  // Объявляем обязательность режима планирования реального времени.
  config_.require_realtime = node_->declare_parameter<bool>(
      // Эта настройка определяет, фатален ли отказ установки требуемого RT-режима.
      "require_realtime", config_.require_realtime, d);
  // Начинаем присваивание разрешения команд движения.
  config_.enable_motion =
      // Без явного разрешения отправка команд движения выключена.
      node_->declare_parameter<bool>("enable_motion", false, d);
  // Загружаем параметр ускорения a, передаваемый в ServoJ.
  real("servo.a", config_.servo_a);
  // Загружаем параметр скорости v, передаваемый в ServoJ.
  real("servo.v", config_.servo_v);
  // Загружаем период команд ServoJ в секундах.
  real("servo.period", config_.period);
  // Загружаем параметр упреждения ServoJ без изменения его значения.
  real("servo.lookahead_time", config_.lookahead_time);
  // Загружаем усиление ServoJ; его трактовку определяет SDK.
  real("servo.gain", config_.gain);
  // Задаём целевой период чтения состояния в секундах.
  real("state_period", config_.state_period);
  // Задаём бюджет длительности одного SDK-вызова, проверяемый после возврата.
  real("sdk_budget", config_.sdk_budget);
  // Задаём максимальное ожидание общего мьютекса SDK.
  real("sdk_lock_timeout", config_.sdk_lock_timeout);
  // Задаём предельный возраст командного образца.
  real("command_timeout", config_.command_timeout);
  // Задаём предельный возраст обратной связи.
  real("state_timeout", config_.state_timeout);
  // Задаём срок ожидания подключения и начальной обратной связи.
  real("startup_timeout", config_.startup_timeout);
  // Задаём максимально допустимое опоздание отправки команды.
  real("max_lateness", config_.max_lateness);
  // Задаём допустимое отклонение интервала команд от периода.
  real("command_jitter_tolerance", config_.command_jitter_tolerance);
  // Задаём предел расхождения ROS-времени с монотонными часами.
  real("ros_clock_jump", config_.ros_clock_jump);
  // Задаём период публикации диагностических сообщений.
  real("diagnostics_period", config_.diagnostics_period);
  // Задаём ожидание ответа сервиса отмены цели контроллера.
  real("controller_cancel_timeout", config_.controller_cancel_timeout);
  // Объявляем имя сервиса отмены целей JointTrajectoryController.
  config_.controller_cancel_service = node_->declare_parameter<std::string>(
      // Загружаем настроенное имя сервиса; пустое значение отключает клиент отмены.
      "controller_cancel_service", config_.controller_cancel_service, d);
  // Загружаем нижние пределы положения шести осей.
  array("limits.min_position", config_.min_position);
  // Загружаем верхние пределы положения шести осей.
  array("limits.max_position", config_.max_position);
  // Загружаем ограничения скорости изменения команд по осям.
  array("limits.max_velocity", config_.max_velocity);
  // Загружаем ограничения ускорения изменения команд по осям.
  array("limits.max_acceleration", config_.max_acceleration);
  // Загружаем максимальные допустимые шаги между соседними командами.
  array("limits.max_step", config_.max_step);
  // Загружаем предел отклонения команды от измеренного положения каждой оси.
  array("limits.tracking_tolerance", config_.tracking_tolerance);
  // Проверяем допустимость и согласованность всех загруженных настроек.
  config_.validate();
  // Ищем заявленную частоту цикла controller_manager.
  auto rate = info_.hardware_parameters.find("controller_update_rate");
  // При наличии частоты проверяем согласованность с ServoJ.
  if (rate != info_.hardware_parameters.end()) {
    // Преобразуем строковую частоту в число герц.
    const double hz = std::stod(rate->second);
    // Частота должна быть конечной и строго положительной.
    if (!std::isfinite(hz) || hz <= 0 ||
        // Произведение частоты на период должно быть единицей с указанной точностью.
        std::abs(hz * config_.period - 1) > 1e-6)
      // Несогласованные часы контроллера и ServoJ запрещают запуск.
      throw std::invalid_argument("controller update rate must match servo.period");
    // Завершение проверки частоты controller_manager.
  }
  // Завершение загрузки и проверки параметров.
}
// Настройка ресурсов аппаратного интерфейса и запуск его потоков.
Callback IimtSystem::on_configure(const rclcpp_lifecycle::State &) {
  // Проверяем, не зафиксирована ли авария предыдущей работы.
  if (faulted_) {
    // Начинаем сообщение о запрете повторной настройки после аварии.
    RCLCPP_ERROR(rclcpp::get_logger("iimt_hardware"),
                 // Объясняем необходимость перезапуска процесса, а не lifecycle-перехода.
                 "Hardware fault is latched; process restart required");
    // Не позволяем сбросить аварийную блокировку повторным configure.
    return callback_error("on_configure: hardware fault is latched; process restart required");
    // Завершение проверки ранее зафиксированной аварии.
  }
  // Исключения настройки и запуска преобразуются в аппаратную ошибку.
  try {
    // Загружаем и проверяем параметры до открытия SDK.
    parameters();
    // Создаём изменяемую копию адреса для имени файла блокировки.
    std::string ip = config_.robot_ip;
    // Обрабатываем каждый символ адреса робота.
    for (char &c : ip)
      // Небуквенно-цифровые символы не должны попадать в имя файла как разделители.
      if (!std::isalnum(static_cast<unsigned char>(c)))
        // Заменяем такие символы подчёркиванием.
        c = '_';
    // Строим отдельный путь блокировки для данного адреса робота.
    auto path = config_.lock_directory + "/iimt_" + ip + ".lock";
    // Сохраняем файловый дескриптор для дальнейшего удержания блокировки.
    lock_fd_ =
        // Открываем файл без следования символьной ссылке и с правами только владельца.
        open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    // Проверяем открытие и немедленный захват эксклюзивной блокировки без ожидания.
    if (lock_fd_ < 0 || flock(lock_fd_, LOCK_EX | LOCK_NB) != 0)
      // Уже занятый или недоступный файл запрещает подключение к роботу.
      throw std::runtime_error("cannot acquire exclusive robot lock " + path);
    // Начинаем предупреждение об ограничениях программной защиты.
    RCLCPP_WARN(node_->get_logger(),
                // Файловая блокировка не останавливает сторонние старые SDK-клиенты.
                "Lock cannot exclude legacy SDK clients. SDK calls have no "
                // SDK-вызов нельзя прервать по таймауту, поэтому завершение может зависнуть.
                "interruptible timeout; shutdown can block. No hardware "
                // Этот драйвер не предоставляет аппаратный watchdog.
                "watchdog is provided.");
    // Отдельная область блокировки для сброса состояния обратной связи.
    {
      // Защищаем состояние от одновременного доступа других потоков.
      std::lock_guard<std::mutex> l(state_mutex_);
      // Сбрасываем отметку времени: свежего измерения пока нет.
      feedback_stamp_ = {};
      // Обнуляем оценку скорости перед первым измерением.
      feedback_velocity_.fill(0);
      // Освобождаем мьютекс состояния при выходе из области.
    }
    // Отдельная область блокировки для текста диагностики аварии.
    {
      // Защищаем сообщение об ошибке от конкурентного доступа.
      std::lock_guard<std::mutex> l(fault_mutex_);
      // Удаляем старый текст при допустимой новой конфигурации.
      fault_text_.clear();
      // Освобождаем мьютекс сообщения об аварии.
    }
    // Интерфейс пока не активирован lifecycle-переходом.
    active_ = false;
    // Контроллер ещё не владеет командными интерфейсами.
    claimed_ = false;
    // Сбрасываем запрос рабочему потоку на остановку.
    stop_requested_ = false;
    // Неподтверждённая остановка пока не ожидается.
    stop_pending_ = false;
    // Новый командный образец ещё не подготовлен.
    command_ready_ = false;
    // Сбрасываем признак неудачного Stop.
    stop_failed_ = false;
    // До владения командами нет обязательства отправлять Stop в SDK.
    needs_stop_ = false;
    // Обнуляем счётчик пропущенных слотов чтения.
    skipped_ = 0;
    // Обнуляем число успешно учтённых отправок команд.
    sent_ = 0;
    // Обнуляем счётчик потерянных командных образцов.
    dropped_ = 0;
    // Сбрасываем длительность последнего измеренного SDK-вызова.
    last_sdk_duration_ = 0;
    // Сбрасываем максимум длительности SDK-вызова.
    max_sdk_duration_ = 0;
    // Сбрасываем последнюю и максимальную длительности чтения.
    last_read_duration_ = max_read_duration_ = 0;
    // Сбрасываем последнюю и максимальную длительности ServoJ.
    last_servoj_duration_ = max_servoj_duration_ = 0;
    // Сбрасываем счётчики чтений и превышений бюджета при чтении.
    read_calls_ = read_budget_overruns_ = unclaimed_read_overruns_ = 0;
    // Сбрасываем счётчики ServoJ и превышений его бюджета.
    servoj_calls_ = servoj_budget_overruns_ = 0;
    // Пока нет измеренного интервала между отправками.
    last_send_interval_ = 0;
    // Обнуляем задержку между проверкой команды и входом в SDK.
    last_guard_to_send_delay_ = 0;
    // Обнуляем возраст команды на момент отправки.
    last_command_age_at_send_ = 0;
    // Сбрасываем максимальное наблюдаемое опоздание команды.
    max_lateness_observed_ = 0;
    // Для этого допустимого configure исходный аварийный флаг снят.
    faulted_ = false;
    // Разрешаем новому рабочему потоку выполнять цикл.
    terminate_ = false;
    // Создаём реализацию транспортного слоя производителя.
    transport_ = make_transport();
    // Запускаем единственный поток, выполняющий SDK-вызовы.
    worker_ = std::thread(&IimtSystem::run, this);
    // Отмена цели контроллера ещё не запрашивалась.
    cancel_requested_ = false;
    // Подтверждения принятой отмены пока нет.
    cancel_accepted_ = false;
    // Устанавливаем исходный диагностический статус отмены.
    cancel_status_ = "not_requested";
    // Создаём клиент отмены только при непустом имени сервиса.
    if (!config_.controller_cancel_service.empty())
      // Клиент использует стандартный ROS action-сервис CancelGoal.
      cancel_client_ = node_->create_client<action_msgs::srv::CancelGoal>(
          // Передаём полное настроенное имя сервиса контроллера.
          config_.controller_cancel_service);
    // Сохраняем издатель диагностики экземпляра.
    diagnostic_pub_ =
        // Диагностика публикуется стандартным сообщением DiagnosticArray.
        node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            // Приватный топик принадлежит аппаратной ноде; глубина очереди равна десяти.
            "~/diagnostics", 10);
    // Создаём таймер диагностики по настенным, а не ROS-часам.
    diagnostic_timer_ = node_->create_wall_timer(
        // Переводим настроенный период диагностики в длительность chrono.
        std::chrono::duration<double>(config_.diagnostics_period),
        // Каждый тик таймера вызывает метод diagnostics данного экземпляра.
        [this] { diagnostics(); });
    // Создаём отдельный однопоточный executor для ROS-ноды оборудования.
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // Регистрируем аппаратную ноду в этом executor.
    executor_->add_node(node_);
    // Запускаем обработку таймеров и ответов ROS в отдельном потоке.
    executor_thread_ = std::thread([this] { executor_->spin(); });
    // Вычисляем предельный момент завершения начальной настройки.
    const auto deadline =
        // Срок отсчитывается монотонными часами от текущего момента.
        Clock::now() + std::chrono::duration<double>(config_.startup_timeout);
    // Ждём первую свежую обратную связь, пока не истёк срок и нет аварии.
    while (Clock::now() < deadline && !faulted_) {
      // Короткая область синхронизации проверки измерения.
      {
        // Блокируем структуру состояния на время чтения отметки времени.
        std::lock_guard<std::mutex> l(state_mutex_);
        // Проверяем, успел ли SDK-поток получить свежее измерение.
        if (!stale(feedback_stamp_, Clock::now(), config_.state_timeout))
          // При свежем измерении настройка завершена успешно.
          return Callback::SUCCESS;
        // Освобождаем мьютекс перед ожиданием следующей проверки.
      }
      // Не тратим процессор на непрерывный опрос состояния запуска.
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      // Завершение ожидания начальной обратной связи.
    }
    // Отсутствие успешного выхода из ожидания фиксирует ошибку запуска.
    fault("startup feedback timeout");
    // Перехватываем стандартные исключения настройки и создания ресурсов.
  } catch (const std::exception &e) {
    // Сохраняем первичную причину исключения через общий механизм аварии.
    fault(e.what());
    // Завершение блока обработки исключения configure.
  }
  // Освобождаем частично созданные ресурсы после неудачного запуска.
  shutdown();
  // Сообщаем lifecycle-менеджеру об ошибке настройки.
  return callback_error("on_configure: setup failed; see preceding hardware fault diagnostics");
  // Завершение настройки аппаратного интерфейса.
}
// Активация допускается только со свежим состоянием и без аварий.
Callback IimtSystem::on_activate(const rclcpp_lifecycle::State &) {
  // Проверяем локальный и групповой аварийные флаги.
  if (faulted_ || group_faulted())
    // Авария запрещает активацию и не сбрасывается здесь.
    return callback_error("on_activate: local or coordination hardware fault is latched");
  // Незавершённый Stop временно препятствует активации.
  if (stop_pending_)
    // Retry after acknowledged Stop; not lifecycle ERROR. // Возвращаем отказ перехода без нового lifecycle ERROR; можно повторить после Stop.
    return Callback::FAILURE;
  // Удерживаем согласованное измерение при подготовке начальной команды.
  std::lock_guard<std::mutex> l(state_mutex_);
  // Проверяем возраст текущей обратной связи монотонными часами.
  if (stale(feedback_stamp_, Clock::now(), config_.state_timeout)) {
    // Фиксируем невозможность безопасной активации по устаревшему состоянию.
    fault("activation requires fresh feedback");
    // Возвращаем аппаратную ошибку активации.
    return callback_error("on_activate: fresh feedback is required");
    // Завершение проверки свежести перед активацией.
  }
  // Начальные команда и экспортируемая позиция равны текущему измерению.
  command_ = position_ = feedback_;
  // На активации экспортируемая скорость начинается с нуля.
  velocity_.fill(0);
  // Кладём исходное положение в mailbox с текущей монотонной отметкой.
  mailbox_.put(command_, Clock::now());
  // Сбрасываем историю ROS-времени перед первым write.
  last_write_ = {};
  // Помечаем hardware активным; владение командами задаётся отдельно.
  active_ = true;
  // Подтверждаем успешную активацию жизненного цикла.
  return Callback::SUCCESS;
  // Завершение активации аппаратного интерфейса.
}
// Деактивация прекращает разрешение команд и запрашивает остановку.
Callback IimtSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  // Запрещаем работу активного командного пути.
  active_ = false;
  // Снимаем внутренний признак владения командными интерфейсами.
  claimed_ = false;
  // Асинхронно запрашиваем обработку остановки рабочим потоком.
  request_stop();
  // Будим рабочий поток, чтобы он увидел изменение состояния.
  wake_.notify_all();
  // Подтверждаем переход; физический Stop ещё может быть не завершён.
  return Callback::SUCCESS;
  // Завершение callback деактивации.
}
// Callback очистки полностью освобождает ресурсы драйвера.
Callback IimtSystem::on_cleanup(const rclcpp_lifecycle::State &) {
  // Выполняем общий путь остановки потоков и освобождения соединений.
  shutdown();
  // Подтверждаем завершение очистки после возврата shutdown.
  return Callback::SUCCESS;
  // Завершение callback очистки.
}
// Callback финального завершения жизненного цикла.
Callback IimtSystem::on_shutdown(const rclcpp_lifecycle::State &) {
  // Используем тот же общий путь освобождения ресурсов.
  shutdown();
  // Подтверждаем завершение после возврата из shutdown.
  return Callback::SUCCESS;
  // Завершение callback shutdown жизненного цикла.
}
// Реакция на аппаратную ошибку без ожидания заблокированного SDK.
Callback IimtSystem::on_error(const rclcpp_lifecycle::State &) {
  // Фиксируем аварийный запрет отправок; полная очистка отложена.
  fault("hardware lifecycle error; SDK sends latched off, cleanup deferred");
  // Do not join an uninterruptible SDK call from the controller-manager thread.
  // Keep diagnostics and exclusive robot ownership until explicit
  // cleanup/destruction.
  // Возвращаем ERROR, сохраняя диагностику и ресурсы для последующей очистки.
  return callback_error(
      "on_error: hardware lifecycle error; SDK sends latched off, cleanup deferred");
  // Завершение callback обработки аппаратной ошибки.
}
// Ставим объединённый запрос остановки единственному SDK-потоку.
void IimtSystem::request_stop() {
  // Multiple lifecycle/fault requests for the same outstanding stop are one
  // operation. Reclaim remains blocked until its successful post-stop read.
  // Атомарно отмечаем ожидающий Stop; новый запрос нужен только при первом переходе.
  if (!stop_pending_.exchange(true))
    // Сообщаем рабочему потоку, что нужно обработать остановку.
    stop_requested_ = true;
  // Прерываем ожидание рабочего потока для скорой обработки Stop.
  wake_.notify_all();
  // Завершение постановки асинхронного запроса остановки.
}
// Единый путь фиксации первой аварии аппаратного интерфейса.
void IimtSystem::fault(const std::string &text) {
  // При наличии группы делаем аварию видимой другим рукам процесса.
  if (group_fault_)
    group_fault_->store(true);
  // Запрещаем активную отправку команд.
  active_ = false;
  // Сбрасываем внутреннее владение командами при аварии.
  claimed_ = false;
  // CAS должен сработать только при переходе из состояния без аварии.
  bool expected = false;
  // Только первый поток фиксирует причину и инициирует первичную остановку.
  if (faulted_.compare_exchange_strong(expected, true)) {
    // Защищаем сохранение текста первичной ошибки.
    std::lock_guard<std::mutex> l(fault_mutex_);
    // Запоминаем первую причину отказа для диагностики.
    fault_text_ = text;
    // Если нода уже создана, используем её логгер.
    RCLCPP_ERROR(node_ ? node_->get_logger()
                       // До создания ноды используем резервный логгер драйвера.
                       : rclcpp::get_logger("iimt_hardware"),
                 // Публикуем исходное сообщение об аварии без подмены причины.
                 "%s", text.c_str());
    // Запрашиваем остановку SDK асинхронно, не выполняя SDK-вызов здесь.
    request_stop();
    // Завершение однократной фиксации первичной аварии.
  }
  // Будим рабочий поток даже при повторном сообщении об аварии.
  wake_.notify_all();
  // Завершение обработки аварии.
}
// Полное освобождение потоков, ROS-ресурсов и блокировки робота.
void IimtSystem::shutdown() {
  // Запрещаем активные команды на время завершения.
  active_ = false;
  // Снимаем владение командными интерфейсами.
  claimed_ = false;
  // Просим рабочий поток выполнить необходимую остановку.
  request_stop();
  // Устанавливаем флаг выхода рабочего потока.
  terminate_ = true;
  // Будим поток, если он ожидал следующую команду или чтение.
  wake_.notify_all();
  // Присоединяем только реально запущенный и ещё не присоединённый поток.
  if (worker_.joinable())
    // Ожидание может зависнуть, если SDK-вызов не возвращает управление.
    worker_.join();
  // Проверяем наличие созданного ROS-executor.
  if (executor_)
    // Просим executor прекратить обработку событий.
    executor_->cancel();
  // Проверяем, был ли запущен отдельный поток executor.
  if (executor_thread_.joinable())
    // Дожидаемся завершения ROS-callback перед освобождением ноды.
    executor_thread_.join();
  // Удаляем незавершённый запрос отмены, если клиент ещё существует.
  if (cancel_pending_ && cancel_client_) {
    // Убираем запрос из таблицы ожидающих ответов клиента.
    cancel_client_->remove_pending_request(*cancel_pending_);
    // Освобождаем хранимый future запроса отмены.
    cancel_pending_.reset();
    // Завершение очистки ожидающего запроса CancelGoal.
  }
  // Уничтожаем диагностический таймер.
  diagnostic_timer_.reset();
  // Освобождаем издатель диагностического топика.
  diagnostic_pub_.reset();
  // Освобождаем остановленный executor.
  executor_.reset();
  // Освобождаем клиент отмены целей контроллера.
  cancel_client_.reset();
  // Освобождаем аппаратную ROS-ноду.
  node_.reset();
  // Уничтожаем транспорт после завершения SDK-потока.
  transport_.reset();
  // Файловую блокировку освобождаем только при действительном дескрипторе.
  if (lock_fd_ >= 0) {
    // Явно снимаем межпроцессное владение файлом робота.
    flock(lock_fd_, LOCK_UN);
    // Закрываем дескриптор файла блокировки.
    close(lock_fd_);
    // Помечаем дескриптор недействительным для повторного shutdown.
    lock_fd_ = -1;
    // Завершение освобождения файловой блокировки.
  }
  // Завершение полной очистки интерфейса.
}
// Результат метода — список интерфейсов измеренного состояния.
std::vector<hardware_interface::StateInterface>
// Передаём ros2_control ссылки на хранилище состояний суставов.
IimtSystem::export_state_interfaces() {
  // Создаём список экспортируемых интерфейсов.
  std::vector<hardware_interface::StateInterface> out;
  // Добавляем состояние всех шести осей в порядке SDK.
  for (size_t i = 0; i < 6; ++i) {
    // Интерфейс position указывает на элемент массива измеренных положений.
    out.emplace_back(info_.joints[i].name, "position", &position_[i]);
    // Интерфейс velocity указывает на элемент массива оценок скорости.
    out.emplace_back(info_.joints[i].name, "velocity", &velocity_[i]);
    // Завершение добавления пары интерфейсов каждой оси.
  }
  // Возвращаем двенадцать интерфейсов состояния.
  return out;
  // Завершение экспорта состояния.
}
// Результат метода — список командных интерфейсов.
std::vector<hardware_interface::CommandInterface>
// Передаём ros2_control ссылки на хранилище команд положения.
IimtSystem::export_command_interfaces() {
  // Создаём список командных интерфейсов экземпляра.
  std::vector<hardware_interface::CommandInterface> out;
  // Для каждой оси экспортируется единственная команда положения.
  for (size_t i = 0; i < 6; ++i)
    // Контроллер будет записывать желаемое положение прямо в command_[i].
    out.emplace_back(info_.joints[i].name, "position", &command_[i]);
  // Возвращаем шесть командных интерфейсов.
  return out;
  // Завершение экспорта команд.
}
// Циклическое чтение для controller_manager копирует кеш, а не вызывает SDK.
Return IimtSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
  // Переносим аварию соседней руки в локальную аппаратную ошибку.
  if (group_faulted() && !faulted_)
    fault("coordination peer fault; process restart required");
  // Проверяем уже зафиксированную локальную аварию.
  if (faulted_)
    // Не выдаём успешное чтение после аппаратного отказа.
    return Return::ERROR;
  // Пробуем захватить состояние без блокирующего ожидания в цикле контроллера.
  std::unique_lock<std::mutex> l(state_mutex_, std::try_to_lock);
  // Если SDK-поток сейчас обновляет кеш, не задерживаем controller_manager.
  if (!l.owns_lock())
    // Сохраняем предыдущие экспортированные значения до следующего цикла.
    return Return::OK;
  // Проверяем давность последнего SDK-измерения.
  if (stale(feedback_stamp_, Clock::now(), config_.state_timeout)) {
    // Для невооружённой координированной руки допускаем временное отсутствие данных.
    if (group_fault_ && !claimed_ && !needs_stop_) {
      // Вместо устаревших положений экспортируем неопределённые значения.
      position_.fill(std::numeric_limits<double>::quiet_NaN());
      // Скорости в этом случае тоже помечаем неизвестными.
      velocity_ = position_;
      // No stale measurement is exported as current feedback. // Цикл остаётся успешным, но NaN не позволяет считать состояние свежим.
      return Return::OK;
      // Завершение специальной обработки простойной координированной руки.
    }
    // Освобождаем мьютекс состояния перед фиксацией аварии.
    l.unlock();
    // Устаревшая обратная связь блокирует дальнейшую работу.
    fault("stale joint feedback");
    // Сообщаем controller_manager об ошибке чтения.
    return Return::ERROR;
    // Завершение проверки возраста обратной связи.
  }
  // Переносим последний согласованный массив положений в экспортируемое состояние.
  position_ = feedback_;
  // Переносим оценённые SDK-потоком скорости суставов.
  velocity_ = feedback_velocity_;
  // Подтверждаем успешное обновление состояния из кеша.
  return Return::OK;
  // Завершение циклического чтения ros2_control.
}
// Циклическая запись принимает образец контроллера без прямого SDK-вызова.
Return IimtSystem::write(const rclcpp::Time &time, const rclcpp::Duration &) {
  // Авария другой руки запрещает новые команды этого экземпляра.
  if (group_faulted() && !faulted_)
    fault("coordination peer fault; process restart required");
  // Проверяем локальный аварийный запрет.
  if (faulted_)
    // Возвращаем аппаратную ошибку вместо принятия очередной команды.
    return Return::ERROR;
  // Для отправки нужны активация, владение интерфейсами и разрешение движения.
  if (!active_ || !claimed_ || !config_.enable_motion)
    // Без этих условий ничего не отправляем и не считаем это ошибкой цикла.
    return Return::OK;
  // Фиксируем монотонное время поступления образца.
  auto now = Clock::now();
  // Читаем ROS-время цикла отдельно от монотонной отметки.
  const double ros = time.seconds();
  // Между соседними записями сравниваем часы только после первого образца.
  if (last_write_ != Clock::time_point{} &&
      // Вычисляем приращение ROS-времени с предыдущего write.
      std::abs((ros - last_ros_time_) -
               // Вычитаем соответствующее приращение монотонного времени в секундах.
               std::chrono::duration<double>(now - last_write_).count()) >
          // Расхождение не должно превышать настроенный предел скачка часов.
          config_.ros_clock_jump) {
    // Фиксируем скачок ROS-времени как нарушение временной траектории.
    fault("ROS clock jump");
    // Не принимаем команду с нарушенной временной основой.
    return Return::ERROR;
    // Завершение проверки скачка между соседними записями.
  }
  // Первый образец задаёт общую точку отсчёта двух часов.
  if (last_write_ == Clock::time_point{}) {
    // Сохраняем начальную монотонную отметку потока команд.
    anchor_write_ = now;
    // Сохраняем соответствующее начальное ROS-время.
    anchor_ros_time_ = ros;
    // Завершение установки временных якорей.
  }
  // Сравниваем накопленное ROS-время от первого образца.
  if (std::abs((ros - anchor_ros_time_) -
               // Вычитаем накопленное монотонное время с той же начальной точки.
               std::chrono::duration<double>(now - anchor_write_).count()) >
      // Так обнаруживаем медленный дрейф, остановку и масштабирование ROS-часов.
      config_.ros_clock_jump) {
    // Фиксируем недопустимое накопленное расхождение часов траектории.
    fault("ROS clock stopped, scaled or diverged from steady trajectory clock");
    // Не допускаем отправку команды после нарушения временной шкалы.
    return Return::ERROR;
    // Завершение проверки накопленного расхождения часов.
  }
  // Запоминаем ROS-время текущего образца для следующей проверки.
  last_ros_time_ = ros;
  // Запоминаем монотонное время текущего образца.
  last_write_ = now;
  // Проверяем конечность и допустимые пределы всех заданных положений.
  if (!valid_position(command_, config_)) {
    // Некорректное положение фиксирует аппаратную ошибку.
    fault("invalid position command");
    // Команда с недопустимым положением не попадёт в mailbox.
    return Return::ERROR;
    // Завершение проверки положений команды.
  }
  // Короткая критическая секция передачи образца рабочему потоку.
  {
    // Согласуем запись mailbox с потреблением и ожиданием SDK-потока.
    std::lock_guard<std::mutex> l(wait_mutex_);
    // Предыдущая команда ещё не принята рабочим потоком.
    if (command_ready_) {
      // Учитываем образец, который нельзя безопасно поставить вместо предыдущего.
      ++dropped_;
      // Запрещаем молча перезаписывать неотправленный образец траектории.
      fault("unconsumed controller sample would be overwritten");
      // Возвращаем ошибку без замены содержимого mailbox.
      return Return::ERROR;
      // Завершение защиты от потери неотправленной команды.
    }
    // Копируем положения, время прихода и исходное ROS-время в mailbox.
    mailbox_.put(command_, now, ros);
    // Отмечаем готовность нового образца к однократному потреблению.
    command_ready_ = true;
    // Освобождаем мьютекс передачи команды.
  }
  // Будим SDK-поток сразу после появления нового образца.
  wake_.notify_all();
  // Подтверждаем принятие образца; фактическая отправка выполняется асинхронно.
  return Return::OK;
  // Завершение циклической записи команды.
}
// Проверяем полный и недублированный набор команд одной руки.
bool IimtSystem::complete_claim(const std::vector<std::string> &keys) const {
  // Набор должен содержать ровно шесть интерфейсов.
  if (keys.size() != 6)
    // Иное количество не соответствует полному владению рукой.
    return false;
  // Проверяем наличие командного интерфейса каждого сустава экземпляра.
  for (const auto &j : info_.joints)
    // Каждый ключ имя/position должен встречаться строго один раз.
    if (std::count(keys.begin(), keys.end(), j.name + "/position") != 1)
      // Отсутствующий или повторённый ключ отвергает весь набор.
      return false;
  // Все шесть команд присутствуют без дубликатов.
  return true;
  // Завершение проверки полного владения.
}
// Результат фильтрации — список строковых ключей интерфейсов.
std::vector<std::string>
// Выбираем из общего запроса только интерфейсы суставов данного экземпляра.
IimtSystem::owned_keys(const std::vector<std::string> &keys) const {
  // Создаём список принадлежащих этой руке ключей.
  std::vector<std::string> result;
  // Перебираем все интерфейсы общего запроса controller_manager.
  for (const auto &key : keys)
    // Сравниваем ключ с именами суставов данной руки.
    for (const auto &joint : info_.joints) {
      // Совпадение должно начинаться с полного имени сустава и разделителя /.
      if (key.rfind(joint.name + "/", 0) == 0) {
        // Добавляем интерфейс этого сустава в локальный набор.
        result.push_back(key);
        // Один ключ добавляется один раз; остальные суставы уже не проверяем.
        break;
        // Завершение обработки совпавшего ключа.
      }
      // Завершение поиска сустава для текущего ключа.
    }
  // Возвращаем только относящуюся к этой руке часть запроса.
  return result;
  // Завершение фильтрации интерфейсов.
}
// Тип результата предварительной проверки переключения команд.
Return
// start содержит интерфейсы, которые контроллер хочет захватить.
IimtSystem::prepare_command_mode_switch(
    const std::vector<std::string> &start,
    // stop содержит интерфейсы, от которых контроллер отказывается.
    const std::vector<std::string> &stop) {
  // Отфильтровываем чужие интерфейсы из обеих частей общего запроса.
  const auto own_start = owned_keys(start), own_stop = owned_keys(stop);
  // Stop is asynchronous: handover must wait for Stop and post-stop feedback.
  // Одновременную передачу владения этой рукой не допускаем.
  if (!own_start.empty() && !own_stop.empty())
    // Для передачи требуется отдельное завершение остановки, а не мгновенный handover.
    return Return::ERROR;
  // Ограничения захвата проверяются только при запросе локальных команд.
  if (!own_start.empty() &&
      // Запрещаем захват без разрешения, при аварии, ожидающем Stop, текущем владении или неполном наборе.
      (!config_.enable_motion || faulted_ || group_faulted() || stop_pending_ || claimed_ ||
       !complete_claim(own_start)))
    // Возвращаем отказ предварительной проверки захвата.
    return Return::ERROR;
  // Отказ от владения тоже должен включать все шесть командных интерфейсов.
  if (!own_stop.empty() && !complete_claim(own_stop))
    // Частичное освобождение интерфейсов одной руки не допускается.
    return Return::ERROR;
  // Запрос прошёл предварительную проверку без изменения состояния.
  return Return::OK;
  // Завершение проверки переключения командных интерфейсов.
}
// Тип результата фактического переключения команд.
Return
// Применяем запрос захвата интерфейсов из списка start.
IimtSystem::perform_command_mode_switch(
    const std::vector<std::string> &start,
    // Второй аргумент задаёт интерфейсы, которые необходимо освободить.
    const std::vector<std::string> &stop) {
  // Повторяем предварительную проверку перед изменением состояния.
  if (prepare_command_mode_switch(start, stop) != Return::OK)
    // Некорректный запрос не должен изменять владение или запускать движение.
    return Return::ERROR;
  // Выбираем локальные части списков захвата и освобождения.
  const auto own_start = owned_keys(start), own_stop = owned_keys(stop);
  // Обрабатываем освобождение интерфейсов этой руки.
  if (!own_stop.empty()) {
    // Снимаем признак владения, запрещая новые командные отправки.
    claimed_ = false;
    // Запрашиваем асинхронный Stop перед возможным новым захватом.
    request_stop();
    // Будим SDK-поток для обработки освобождения.
    wake_.notify_all();
    // Завершение освобождения командных интерфейсов.
  }
  // Обрабатываем получение владения командными интерфейсами.
  if (!own_start.empty()) {
    // Захват допустим только на активном hardware без локальной и групповой аварии.
    if (faulted_ || group_faulted() || !active_)
      // Аварийный или неактивный hardware отклоняет захват.
      return Return::ERROR;
    // Защищаем исходное измерение на время подготовки командного состояния.
    std::lock_guard<std::mutex> l(state_mutex_);
    // Для начала управления необходима свежая обратная связь.
    if (stale(feedback_stamp_, Clock::now(), config_.state_timeout))
      // Устаревшее измерение запрещает захват без инициализации команды.
      return Return::ERROR;
    // Начальное желаемое положение совпадает с измеренным, исключая произвольную цель.
    command_ = feedback_;
    // Обновляем mailbox исходной позой и монотонной отметкой времени.
    mailbox_.put(command_, Clock::now());
    // Сбрасываем историю write для новой последовательности команд.
    last_write_ = {};
    // После захвата потребуется Stop при освобождении или отказе, даже до первой отправки.
    needs_stop_ = true;
    // Исходная поза не считается новым образцом write для немедленной отправки.
    command_ready_ = false;
    // Включаем владение; дальнейшие write могут передавать команды SDK-потоку.
    claimed_ = true;
    // Завершение подготовки вновь захваченных командных интерфейсов.
  }
  // Подтверждаем успешное изменение режима владения.
  return Return::OK;
  // Завершение переключения командных интерфейсов.
}
// История команд принадлежит только рабочему SDK-потоку.
struct IimtSystem::WorkerState {
  // Сохраняем прежний порядок инициализации часов цикла.
  explicit WorkerState(double seconds)
      : period(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds))),
        // Опрос разрешён сразу после подключения.
        next_read(Clock::now()) {}
  // Номинальный период отправки.
  const Clock::duration period;
  // Следующий слот чтения.
  Clock::time_point next_read;
  // История монотонных меток.
  Clock::time_point previous_send_stamp{}, previous_sdk_entry{}, previous_arrival{};
  // ROS-время предыдущей команды.
  double previous_source_time = 0;
  // Номер предыдущего образца.
  uint64_t previous_sequence = 0;
  // История положений и двух оценок скорости.
  Joints previous{}, previous_velocity{}, emitted_velocity{};
  // Состояние отправки и подтверждения Stop.
  bool streaming = false, awaiting_stop_feedback = false;
  // Состояние уничтожается при выходе рабочего потока.
};

// Настраиваем планировщик текущего потока и подключаем SDK.
void IimtSystem::connect_worker() {
  // Результат подключения SDK.
  bool connected = false;
  // Запрашиваем политику реального времени только при заданном положительном приоритете.
  if (config_.worker_priority > 0) {
    // Обнуляем структуру параметров планировщика POSIX.
    sched_param scheduling{};
    // Берём приоритет потока из проверенной конфигурации.
    scheduling.sched_priority = config_.worker_priority;
    // Сохраняем код результата установки политики планирования.
    const int error =
        // Просим ОС назначить текущему потоку FIFO-планирование с выбранным приоритетом.
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &scheduling);
    // Обрабатываем отказ ОС в настройке планировщика.
    if (error) {
      // Проверяем, запрещён ли запуск без политики реального времени.
      if (config_.require_realtime)
        // Прерываем запуск исключением при обязательном, но недоступном SCHED_FIFO.
        throw std::runtime_error(
            // Начинаем сообщение о недоступной обязательной политике планирования.
            std::string("required SCHED_FIFO unavailable: ") +
            // Добавляем системное описание ошибки POSIX.
            std::strerror(error));
      // При необязательном RT выводим предупреждение в журнал ноды.
      RCLCPP_WARN(node_->get_logger(),
                  // Указываем, что поток продолжит работу без гарантированного RT-планирования.
                  "SCHED_FIFO unavailable, best-effort scheduling: %s",
                  // Подставляем расшифровку кода ошибки планировщика.
                  std::strerror(error));
      // Завершаем обработку отказа установки SCHED_FIFO.
    }
    // При нулевом приоритете вообще не запрашиваем SCHED_FIFO.
  } else
    // Предупреждаем об использовании обычного планировщика ОС.
    RCLCPP_WARN(
        // Направляем предупреждение в журнал hardware-ноды.
        node_->get_logger(),
        // Уточняем, что обычное планирование не является жёстким реальным временем.
        "SDK worker uses best-effort OS scheduling, not hard real-time");
  // Запоминаем начало подключения по монотонным часам.
  const auto login_start = Clock::now();
  // Ограничиваем время владения общей блокировкой SDK этапом подключения.
  {
    // Ожидаем доступ к SDK не дольше разрешённого времени запуска.
    auto lock = sdk_lock(config_.startup_timeout);
    // Не вызываем SDK, если общую блокировку получить не удалось.
    if (!lock.owns_lock())
      throw std::runtime_error("SDK startup gate timeout");
    // Не подключаемся после зафиксированной ошибки другой руки в группе.
    if (group_faulted())
      throw std::runtime_error("coordination peer fault before login");
    // Подключаем транспорт с заданными IP, портом и путём к SDK.
    connected = transport_->connect(config_);
    // Освобождаем общую блокировку SDK после подключения.
  }
  // Проверяем результат подключения транспорта.
  if (!connected)
    // Преобразуем неуспешный вход SDK в ошибку рабочего потока.
    throw std::runtime_error("SDK login failed");
  // Вычисляем полную длительность подключения, включая ожидание блокировки.
  if (std::chrono::duration<double>(Clock::now() - login_start).count() >
      // Сравниваем длительность с предельным временем запуска.
      config_.startup_timeout)
    // Фиксируем превышение лишь после возврата блокирующего подключения.
    throw std::runtime_error("SDK login exceeded startup timeout");
  // Завершение процедуры.
}

// Измеряем завершившийся вызов SDK; таймаут не прерывает вызов.
bool IimtSystem::check_sdk_budget(Clock::time_point begin, bool is_read, bool armed_at_start,
                                  bool succeeded) {
  // Определяем проверку времени SDK с учётом вида операции, владения и её результата.
  // Объявляем фактическую длительность завершившегося вызова.
  const double elapsed =
      // Измеряем длительность монотонными часами и переводим в секунды.
      std::chrono::duration<double>(Clock::now() - begin).count();
  // Обновляем длительность последнего вызова SDK для диагностики.
  last_sdk_duration_ = elapsed;
  // Сохраняем наибольшую наблюдавшуюся длительность вызова SDK.
  max_sdk_duration_ = std::max(max_sdk_duration_.load(), elapsed);
  // Разделяем статистику чтения и отправки ServoJ.
  if (is_read) {
    // Учитываем очередной завершившийся вызов чтения.
    ++read_calls_;
    // Сохраняем длительность последнего чтения.
    last_read_duration_ = elapsed;
    // Обновляем максимальную длительность чтения.
    max_read_duration_ = std::max(max_read_duration_.load(), elapsed);
    // Для отправки обновляем отдельную статистику ServoJ.
  } else {
    // Учитываем очередной завершившийся вызов ServoJ.
    ++servoj_calls_;
    // Сохраняем длительность последней отправки.
    last_servoj_duration_ = elapsed;
    // Обновляем максимальную длительность отправки ServoJ.
    max_servoj_duration_ = std::max(max_servoj_duration_.load(), elapsed);
    // Завершаем обновление статистики соответствующей операции.
  }
  // Проверяем, уложился ли вызов в заданный бюджет SDK.
  if (elapsed <= config_.sdk_budget)
    // Разрешаем продолжение при соблюдённом бюджете времени.
    return true;
  // Увеличиваем счётчик превышений бюджета чтением.
  if (is_read)
    ++read_budget_overruns_;
  // Либо увеличиваем отдельный счётчик превышений при ServoJ.
  else
    ++servoj_budget_overruns_;
  // Проверяем, вернула ли сама операция SDK ошибку.
  if (!succeeded)
    // Preserve the caller's specific SDK failure as the primary fault. // Не подменяем ошибку SDK ошибкой времени: конкретную причину зафиксирует вызывающий код.
    return true;
  // Выбираем имя API для предупреждения или сообщения об отказе.
  const char *operation = is_read ? "API_GetRobotJointRad" : "API_Servoj";
  // Checking both ends prevents claim/release during an SDK read from
  // bypassing the armed budget. ServoJ is always subject to the budget.
  // Считаем управление задействованным при владении до/после вызова либо необходимости Stop.
  const bool armed = armed_at_start || claimed_.load() || needs_stop_.load();
  // Разрешаем отдельную политику только для успешного чтения без владения управлением.
  if (is_read && !armed) {
    // Считаем такие превышения, но печатаем предупреждение только при первом.
    if (++unclaimed_read_overruns_ == 1)
      // Сообщаем о медленном чтении без аварийного отключения диагностического режима.
      RCLCPP_WARN(
          node_->get_logger(),
          // Формируем часть предупреждения с фактическим временем и бюджетом вызова.
          "%s wall time %.3f ms exceeds motion sdk_budget %.3f ms "
          // Добавляем период ServoJ и указание на отсутствие владельца команд.
          "(servo.period %.3f ms), but no commands are claimed. "
          // Предупреждаем, что продолжение диагностики не подтверждает пригодность для движения.
          "Continuing measured-state diagnostics; motion timing is NOT qualified. "
          // Указываем топик с подробной статистикой длительностей и превышений.
          "See read duration/overrun fields in ~/diagnostics.",
          // Подставляем имя операции и времена в миллисекундах.
          operation, elapsed * 1000, config_.sdk_budget * 1000, config_.period * 1000);
    // Successful idle measurements remain observable. // Сохраняем возможность получать успешные измерения в режиме без управления.
    return true;
    // Завершаем исключение для медленного диагностического чтения.
  }
  // Фиксируем аварийный отказ с именем операции и фактической длительностью.
  fault(std::string(operation) + " wall time " + std::to_string(elapsed * 1000) +
        // Включаем в причину настроенный бюджет SDK в миллисекундах.
        " ms exceeded sdk_budget=" + std::to_string(config_.sdk_budget * 1000) +
        // Включаем в причину настроенный период ServoJ в миллисекундах.
        " ms, servo.period=" + std::to_string(config_.period * 1000) +
        // Уточняем, что превышение обнаружено при владении управлением и только после возврата SDK.
        " ms while commands are owned (detected only after return)");
  // Запрещаем обработку результата просроченного вызова как нормального.
  return false;
  // Завершение процедуры.
}

// Проверяем период без изменения допуска джиттера.
bool IimtSystem::valid_command_interval(double dt) const {
  // Определяем проверку интервала между последовательными командами.
  // Отбрасываем NaN и бесконечные интервалы времени.
  return std::isfinite(dt) &&
         // Требуем отклонение от заданного периода не больше разрешённого джиттера.
         std::abs(dt - config_.period) <= config_.command_jitter_tolerance;
  // Завершение процедуры.
}

// Обслуживаем запрос остановки, сохраняя повтор при занятом SDK.
bool IimtSystem::service_worker_stop(WorkerState &state) {
  // Атомарно забираем и сбрасываем запрос остановки для обработки этим потоком.
  if (stop_requested_.exchange(false)) {
    // Вызываем аппаратную остановку только если управление ранее требовало её.
    if (needs_stop_) {
      // Получаем общий доступ к SDK с ограниченным ожиданием.
      auto lock = sdk_lock(config_.sdk_lock_timeout);
      // Проверяем неудачу получения блокировки перед Stop.
      if (!lock.owns_lock()) {
        // Отмечаем, что попытка остановки не завершилась успешно.
        stop_failed_ = true;
        // Фиксируем отказ с явным отсутствием подтверждения физической остановки.
        fault("SDK gate timeout during Stop; physical stop not confirmed");
        // Повторно ставим запрос Stop, чтобы следующая итерация попробовала снова.
        stop_requested_ = true;
        // Делаем паузу 1 мс, не загружая процессор непрерывными попытками блокировки.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // Переходим к следующей попытке обслуживания остановки.
        return false;
        // Завершаем обработку недоступной блокировки при Stop.
      }
      // Вызываем транспортную остановку и проверяем её результат.
      if (!transport_->stop()) {
        // Отмечаем ошибку Stop или очистки очереди SDK.
        stop_failed_ = true;
        // Фиксируем отказ остановки как аппаратную ошибку.
        fault("SDK Stop or ClearUnfinishedTrajectory failed");
        // При успешном возврате остановки снимаем необходимость повторного Stop.
      } else
        // Сбрасываем признак требуемой аппаратной остановки.
        needs_stop_ = false;
      // Завершаем ветку аппаратной остановки ранее задействованного управления.
    }
    // Сбрасываем историю непрерывной отправки команд.
    state.streaming = false;
    // Отбрасываем готовность ранее ожидавшей команды.
    command_ready_ = false;
    // При отсутствии аварии требуем новое измерение после Stop перед повторным захватом.
    state.awaiting_stop_feedback = !faulted_;
    // Назначаем ближайшее чтение без дополнительной задержки.
    state.next_read = Clock::now();
    // Завершаем обработку запроса остановки.
  }
  // Обработка Stop позволяет продолжить текущую итерацию.
  return true;
  // Завершение процедуры.
}

// Принимаем один свежий образец и проверяем его до единственного ServoJ.
bool IimtSystem::dispatch_worker_command(WorkerState &state, bool &sent_this_iteration) {
  // Отправляем только при активном hardware, владельце команд и разрешённом движении без Stop.
  if (active_ && claimed_ && config_.enable_motion && !stop_pending_) {
    // Создаём локальную копию команды из почтового ящика.
    Command cmd;
    // Объявляем признак наличия нового, ещё не потреблённого образца.
    bool ready;
    // Ограничиваем критическую секцию чтением состояния почтового ящика.
    {
      // Согласуем доступ с методом write и сигнализацией рабочего потока.
      std::lock_guard<std::mutex> l(wait_mutex_);
      // Забираем признак готовности нового образца и атомарно сбрасываем его.
      ready = command_ready_.exchange(false);
      // Копируем последний образец вместе с временными метками и номером.
      cmd = mailbox_.get();
      // Освобождаем мьютекс почтового ящика до проверок и SDK.
    }
    // Снимаем текущую монотонную временную метку для проверки свежести.
    auto now = Clock::now();
    // Проверяем общий таймаут поступившей команды.
    if (stale(cmd.stamp, now, config_.command_timeout)) {
      // Фиксируем отказ при устаревшем образце контроллера.
      fault("stale controller command");
      // Не отправляем устаревшую команду и переходим к обработке отказа.
      return false;
      // Завершаем проверку общего возраста команды.
    }
    // В уже начавшемся потоке проверяем отсутствие следующего нового образца.
    if (state.streaming && !ready &&
        // Измеряем время от поступления предыдущего отправленного образца.
        std::chrono::duration<double>(now - state.previous_arrival).count() >
            // Сравниваем ожидание следующей команды с периодом плюс допустимым джиттером.
            config_.period + config_.command_jitter_tolerance) {
      // Фиксируем пропуск срока; старое задание повторно не отправляется.
      fault("controller command deadline missed; no old setpoint replayed");
      // Переходим к следующей итерации для обработки отказа.
      return false;
      // Завершаем проверку пропущенного образца потока команд.
    }
    // Обрабатываем отправку лишь при наличии нового образца.
    if (ready) {
      // Ожидаем общий доступ к SDK перед ServoJ.
      auto sdk_guard = sdk_lock(config_.sdk_lock_timeout);
      // Проверяем, удалось ли получить блокировку SDK вовремя.
      if (!sdk_guard.owns_lock()) {
        // Фиксируем отказ по таймауту доступа к SDK перед отправкой.
        fault("SDK gate timeout before ServoJ");
        // Пропускаем отправку при недоступном SDK.
        return false;
        // Завершаем обработку таймаута общей блокировки.
      }
      // Повторно проверяем отказ группы после ожидания блокировки.
      if (group_faulted()) {
        // Запрещаем отправку из-за ошибки другой руки.
        fault("coordination peer fault before ServoJ");
        // Выходим из текущей итерации без вызова ServoJ.
        return false;
        // Завершаем повторную проверку координационной ошибки.
      }
      // All freshness and dispatch guards include gate wait. // Обновляем время после ожидания SDK, чтобы включить это ожидание в проверки свежести.
      now = Clock::now();
      // Вычисляем возраст команды на момент получения SDK-блокировки.
      const double age = std::chrono::duration<double>(now - cmd.stamp).count();
      // Сохраняем максимальную наблюдавшуюся задержку доставки команды к отправке.
      max_lateness_observed_ = std::max(max_lateness_observed_.load(), age);
      // Сравниваем задержку с допустимым опозданием команды.
      if (age > config_.max_lateness) {
        // Фиксируем пропущенный срок отправки образца.
        fault("controller command dispatch deadline missed");
        // Не отправляем запоздавший образец.
        return false;
        // Завершаем проверку опоздания перед ServoJ.
      }
      // Во время потока требуем строго следующий номер образца без пропусков.
      if (state.streaming && cmd.sequence != state.previous_sequence + 1) {
        // При скачке номера вперёд учитываем число потерянных образцов.
        if (cmd.sequence > state.previous_sequence + 1)
          // Добавляем количество пропущенных номеров в счётчик потерь.
          dropped_ += cmd.sequence - state.previous_sequence - 1;
        // Фиксируем разрыв последовательности без попытки проигрывать накопившуюся очередь.
        fault("controller samples overwritten; no backlog replay allowed");
        // Не продолжаем прерывистую траекторию.
        return false;
        // Завершаем проверку непрерывности последовательности образцов.
      }
      // Создаём локальный массив измеренных положений шести суставов.
      Joints measured;
      // Создаём локальную временную метку измерения.
      Clock::time_point stamp;
      // Ограничиваем блокировку совместным копированием измерений и их времени.
      {
        // Защищаем снимок обратной связи от конкурентного обновления.
        std::lock_guard<std::mutex> l(state_mutex_);
        // Копируем последние принятые положения суставов.
        measured = feedback_;
        // Копируем соответствующий момент завершения чтения SDK.
        stamp = feedback_stamp_;
        // Освобождаем мьютекс обратной связи.
      }
      // Проверяем возраст измерений перед расчётом защитных ограничений.
      if (stale(stamp, now, config_.state_timeout)) {
        // Фиксируем отказ из-за устаревшей обратной связи.
        fault("stale feedback before ServoJ");
        // Запрещаем ServoJ без достаточно свежего измерения.
        return false;
        // Завершаем проверку свежести обратной связи.
      }
      // Для первой отправки инициализируем историю от измеренного положения.
      if (!state.streaming) {
        // Используем реальное текущее положение как исходное задание.
        state.previous = measured;
        // Предполагаем нулевую начальную скорость для проверки исходной последовательности.
        state.previous_velocity.fill(0);
        // Предполагаем нулевую начальную скорость для проверки фактических отправок.
        state.emitted_velocity.fill(0);
        // Задаём начальную историю отправки на один номинальный период раньше.
        state.previous_send_stamp = now - state.period;
        // Задаём начальную историю ROS-времени на один период раньше первой команды.
        state.previous_source_time = cmd.source_time - config_.period;
        // Завершаем инициализацию истории нового потока.
      }
      // Вычисляем интервал команды по временным меткам контроллера.
      const double dt = cmd.source_time - state.previous_source_time;
      // После первой отправки проверяем регулярность исходного потока.
      if (state.streaming && !valid_command_interval(dt)) {
        // Фиксируем слишком короткий или длинный интервал в шкале источника.
        fault("bunched or late source command interval: " + std::to_string(dt));
        // Не отправляем образец с нарушенной периодичностью источника.
        return false;
        // Завершаем проверку исходного временного интервала.
      }
      // Создаём строку для подробного описания нарушенного ограничения.
      std::string violation;
      // Проверяем положения, шаг, скорости, ускорения и рассогласование относительно измерений.
      if (!valid_step(cmd.position, state.previous, state.previous_velocity, measured,
                      // Для этой проверки используем интервал источника и ограничения конфигурации.
                      dt, config_, &violation)) {
        // Фиксируем конкретное нарушение исходной команды.
        fault("source command guard exceeded:" + violation);
        // Пропускаем отправку команды, не прошедшей защиту источника.
        return false;
        // Завершаем проверку ограничений исходного потока.
      }
      // Use one consistent dispatch timestamp for both endpoints of the
      // emitted secant. A timestamp before feedback locking/source checks
      // mixed with an SDK-entry timestamp would manufacture acceleration.
      // Фиксируем единый момент отправки для согласованного расчёта фактических интервалов.
      const auto dispatch = Clock::now();
      // Повторно проверяем общий возраст команды после предыдущих вычислений.
      if (stale(cmd.stamp, dispatch, config_.command_timeout) ||
          // Также повторно проверяем допустимое опоздание отправки.
          stale(cmd.stamp, dispatch, config_.max_lateness) ||
          // Проверяем, не устарели ли измерения за время проверки команды.
          stale(stamp, dispatch, config_.state_timeout)) {
        // Фиксируем истечение срока свежести непосредственно во время защитных проверок.
        fault("freshness deadline expired during ServoJ guards");
        // Не отправляем команду, ставшую просроченной в процессе обработки.
        return false;
        // Завершаем повторную проверку свежести перед отправкой.
      }
      // При первой отправке нормируем начальный фактический интервал.
      if (!state.streaming)
        // Выбираем предыдущий момент ровно на один период раньше текущей проверки.
        state.previous_send_stamp = dispatch - state.period;
      // Объявляем фактический интервал между моментами проверки отправок.
      const double emitted_dt =
          // Переводим разность монотонных временных меток отправки в секунды.
          std::chrono::duration<double>(dispatch - state.previous_send_stamp).count();
      // В установившемся потоке проверяем периодичность фактической отправки.
      if (state.streaming && !valid_command_interval(emitted_dt)) {
        // Фиксируем сближенные или запоздавшие отправки с измеренным интервалом.
        fault("bunched or late SDK dispatch interval: " + std::to_string(emitted_dt));
        // Не отправляем образец, нарушающий периодичность SDK-диспетчеризации.
        return false;
        // Завершаем проверку фактического интервала отправок.
      }
      // Повторяем защиту шага и производных для фактического времени отправок.
      if (!valid_step(cmd.position, state.previous, state.emitted_velocity, measured,
                      // Используем измеренный интервал отправки вместо интервала ROS-источника.
                      emitted_dt, config_, &violation)) {
        // Фиксируем нарушение ограничений фактически выдаваемого потока ServoJ.
        fault("emitted ServoJ command guard exceeded:" + violation);
        // Не отправляем образец, не прошедший защиту выходного потока.
        return false;
        // Завершаем проверку фактически выдаваемой команды.
      }
      // Непосредственно перед SDK повторно проверяем разрешение управления и отсутствие отказа/Stop.
      if (active_ && claimed_ && !faulted_ && !group_faulted() && !stop_requested_) {
        // Запоминаем момент входа в транспортную отправку для измерения её длительности.
        const auto begin = Clock::now();
        // Отмечаем необходимость Stop при дальнейшем освобождении управления или отказе.
        needs_stop_ = true;
        // Отправляем шесть заданных положений через ServoJ с параметрами конфигурации.
        const bool ok = transport_->send(cmd.position, config_);
        // Измеряем время ServoJ и применяем строгий бюджет независимо от владения после вызова.
        const bool within_budget = check_sdk_budget(begin, false, true, ok);
        // Сохраняем задержку между проверкой фактического интервала и началом вызова SDK.
        last_guard_to_send_delay_ = std::chrono::duration<double>(begin - dispatch).count();
        // Для первой отправки публикуем нулевой интервал; для последующих вычисляем разность.
        last_send_interval_ =
            state.previous_sdk_entry == Clock::time_point{}
                ? 0.0
                :
                // Измеряем интервал между реальными началами последовательных вызовов отправки.
                std::chrono::duration<double>(begin - state.previous_sdk_entry).count();
        // Сохраняем возраст образца именно на момент входа в отправку SDK.
        last_command_age_at_send_ = std::chrono::duration<double>(begin - cmd.stamp).count();
        // Проверяем результат выполнения транспортной отправки.
        if (!ok) {
          // Фиксируем ошибку вызова ServoJ.
          fault("SDK ServoJ failed");
          // Не обновляем историю успешно отправленных команд при ошибке SDK.
          return false;
          // Завершаем обработку ошибки отправки.
        }
        // Проверяем результат временной защиты завершившегося ServoJ.
        if (!within_budget)
          // После превышения бюджета переходим к обработке уже зафиксированного отказа.
          return false;
        // Обновляем оценки скоростей для каждого из шести суставов.
        for (size_t i = 0; i < 6; ++i) {
          // Считаем скорость между заданиями по интервалу исходного контроллера.
          state.previous_velocity[i] = (cmd.position[i] - state.previous[i]) / dt;
          // Считаем скорость между заданиями по фактическому интервалу отправки.
          state.emitted_velocity[i] = (cmd.position[i] - state.previous[i]) / emitted_dt;
          // Завершаем обновление производных для шести суставов.
        }
        // Сохраняем ROS-время успешно обработанного образца.
        state.previous_source_time = cmd.source_time;
        // Сохраняем номер успешно обработанного образца для контроля непрерывности.
        state.previous_sequence = cmd.sequence;
        // Сохраняем момент поступления образца для срока ожидания следующей команды.
        state.previous_arrival = cmd.stamp;
        // Сохраняем единый момент проверки отправки для следующей фактической производной.
        state.previous_send_stamp = dispatch;
        // Сохраняем начало вызова SDK для диагностического интервала отправок.
        state.previous_sdk_entry = begin;
        // Сохраняем заданные положения как исходные для следующего шага.
        state.previous = cmd.position;
        // Отмечаем, что поток успешно отправляемых команд начался.
        state.streaming = true;
        // Разрешаем последующее чтение состояния в этой итерации.
        sent_this_iteration = true;
        // Увеличиваем счётчик команд, успешно прошедших отправку и проверку бюджета.
        ++sent_;
        // Завершаем отправку при подтверждённых условиях управления.
      }
      // Завершаем обработку нового образца из почтового ящика.
    }
    // Если управление сейчас не разрешено, сбрасываем состояние непрерывного потока.
  } else
    // Следующая разрешённая отправка начнёт новую историю производных.
    state.streaming = false;
  // Можно перейти к чтению и ожиданию.
  return true;
  // Завершение процедуры.
}

// Читаем состояние после команды либо в незадействованном режиме.
bool IimtSystem::poll_worker_feedback(WorkerState &state, bool sent_this_iteration) {
  // Ownership, not first successful send, selects the command clock.
  // An idle poll between claim and the first write can delay that write,
  // compressing the next dispatch interval even with a regular CM clock.
  // Разрешаем чтение лишь без отказа и с соблюдением порядка относительно команд.
  if (!faulted_ && feedback_poll_allowed(claimed_, sent_this_iteration) &&
      // Дополнительно дожидаемся назначенного слота опроса состояния.
      Clock::now() >= state.next_read) {
    // Получаем общую блокировку перед обращением к SDK за состоянием.
    auto sdk_guard = sdk_lock(config_.sdk_lock_timeout);
    // Обрабатываем истечение времени ожидания SDK-блокировки.
    if (!sdk_guard.owns_lock()) {
      // При владении, требуемой остановке или одиночной руке считаем таймаут ошибкой.
      if (claimed_ || needs_stop_ || !group_fault_)
        // Фиксируем отказ получения доступа к SDK для чтения.
        fault("SDK gate timeout during feedback read");
      // Для незадействованной руки в координационной группе допускаем отложенный опрос.
      else
        // Another arm may still be logging in. // Переносим чтение на период вперёд, позволяя другой руке закончить подключение.
        state.next_read = Clock::now() + state.period;
      // Заканчиваем итерацию без чтения при недоступной SDK-блокировке.
      return false;
      // Завершаем обработку таймаута доступа к SDK перед чтением.
    }
    // Не читаем SDK после отказа другой руки; фиксируем локальную ошибку и выходим из итерации.
    if (group_faulted()) {
      fault("coordination peer fault before feedback read");
      return false;
    }
    // Claim may have arrived while waiting for the shared SDK gate.
    // Повторно проверяем порядок чтения после ожидания SDK: владение могло измениться.
    if (!feedback_poll_allowed(claimed_, sent_this_iteration))
      return false;
    // Обнуляем локальный массив для результата чтения суставов.
    Joints measured{};
    // Запоминаем факт задействованного управления до начала блокирующего чтения.
    const bool armed_at_read_start = claimed_.load() || needs_stop_.load();
    // Запоминаем начало чтения для измерения времени SDK.
    const auto begin = Clock::now();
    // Синхронно получаем положения суставов через транспорт.
    const bool ok = transport_->read(measured);
    // Метим измерение временем возврата SDK, а не неизвестным временем аппаратного замера.
    const auto stamp = Clock::now();
    // Проверяем длительность чтения с учётом владения до вызова и после него.
    const bool within_budget = check_sdk_budget(begin, true, armed_at_read_start, ok);
    // Проверяем, вернуло ли чтение SDK успешный результат.
    if (!ok) {
      // Фиксируем ошибку чтения, не освежая метку старого измерения.
      fault("SDK joint read failed; feedback timestamp not refreshed");
      // Не публикуем результат неуспешного чтения.
      return false;
      // Завершаем обработку ошибки чтения SDK.
    }
    // Проверяем, разрешила ли временная защита использовать измерение.
    if (!within_budget)
      // После нарушения бюджета не обновляем обратную связь этим результатом.
      return false;
    // Проверяем конечность измеренных углов и заданные пределы положений.
    if (!valid_position(measured, config_)) {
      // Фиксируем отказ при некорректных или выходящих за пределы измерениях.
      fault("invalid or out-of-bounds measured joints");
      // Не передаём недопустимые измерения контроллеру.
      return false;
      // Завершаем проверку диапазонов измеренных положений.
    }
    // Ограничиваем критическую секцию обновлением общего снимка состояния.
    {
      // Защищаем обновление положений, скоростей и времени измерения одним мьютексом.
      std::lock_guard<std::mutex> l(state_mutex_);
      // Оцениваем скорость лишь при наличии предыдущего измерения.
      if (feedback_stamp_ != Clock::time_point{}) {
        // Вычисляем интервал между моментами возврата двух успешных чтений SDK.
        const double dt = std::chrono::duration<double>(stamp - feedback_stamp_).count();
        // Исключаем деление на нулевой или отрицательный интервал.
        if (dt > 0)
          // Вычисляем разностную скорость отдельно для каждого из шести суставов.
          for (size_t i = 0; i < 6; ++i)
            // Оцениваем скорость по изменению измеренного угла и интервалу чтений.
            feedback_velocity_[i] = (measured[i] - feedback_[i]) / dt;
        // Завершаем расчёт скорости относительно предыдущего измерения.
      }
      // Сохраняем новый массив измеренных положений.
      feedback_ = measured;
      // Обновляем метку свежести только для успешно проверенного измерения.
      feedback_stamp_ = stamp;
      // Освобождаем мьютекс обратной связи после согласованного обновления.
    }
    // Проверяем, ожидалось ли это измерение для завершения процедуры Stop.
    if (state.awaiting_stop_feedback) {
      // Снимаем ожидание первого успешного измерения после остановки.
      state.awaiting_stop_feedback = false;
      // Разрешаем дальнейший захват управления после завершения Stop и обновления состояния.
      stop_pending_ = false;
      // Завершаем подтверждение обратной связи после Stop.
    }
    // Обнуляем локальный счётчик пропущенных слотов опроса.
    uint64_t poll_skipped = 0;
    // Выбираем следующий будущий слот относительно прежнего расписания и завершения чтения.
    state.next_read = next_slot(
        state.next_read, stamp,
        // Используем отдельно настроенный период опроса состояния в единицах монотонных часов.
        std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(config_.state_period)),
        // Получаем число пропущенных слотов без воспроизведения накопленных опросов.
        poll_skipped);
    // Добавляем пропуски опроса в общий диагностический счётчик.
    skipped_ += poll_skipped;
    // Завершаем итерационное чтение обратной связи.
  }
  // Чтение завершено либо слот ещё не наступил.
  return true;
  // Завершение процедуры.
}

// Ожидаем следующую команду или срок опроса без активного вращения.
void IimtSystem::wait_worker_event(const WorkerState &state) {
  // По умолчанию планируем пробуждение на следующий опрос состояния.
  auto deadline = state.next_read;
  // При захваченном управлении до первой отправки выбираем отдельное ожидание команды.
  if (claimed_ && !state.streaming) {
    // next_read can already be in the past while polling is suppressed.
    // Sleep rather than spin; a new write wakes us immediately, and the
    // normal stale-command check above still bounds startup waiting.
    // Ждём не более периода вместо активного вращения на просроченном слоте чтения.
    deadline = Clock::now() + state.period;
    // Завершаем настройку ожидания первой команды.
  }
  // В установившемся потоке ориентируем пробуждение на следующую команду контроллера.
  if (state.streaming) {
    // Начинаем вычислять крайний срок от поступления предыдущего образца.
    const auto command_deadline =
        state.previous_arrival +
        // Переводим допустимую длительность ожидания в тип монотонных часов.
        std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(
            // Разрешаем один номинальный период плюс настроенный джиттер команд.
            config_.period + config_.command_jitter_tolerance));
    // Назначаем командный срок вместо независимого срока опроса состояния.
    deadline = command_deadline;
    // Завершаем выбор времени пробуждения в режиме потока команд.
  }
  // Захватываем мьютекс для безопасного ожидания условной переменной.
  std::unique_lock<std::mutex> l(wait_mutex_);
  // Спим до выбранного срока либо до значимого события, освобождая мьютекс на время сна.
  wake_.wait_until(l, deadline, [this] {
    // Просыпаемся при завершении потока, запросе Stop или готовности нового образца.
    return terminate_.load() || stop_requested_.load() || command_ready_.load();
    // Завершаем ожидание и возвращаемся к обработке очередной итерации.
  });
  // Завершение процедуры.
}

// Останавливаем задействованное управление и закрываем SDK в том же потоке.
void IimtSystem::disconnect_worker() {
  // All SDK operations, including final cleanup, remain on this worker thread.
  // These calls may block indefinitely: the SDK exposes no
  // cancellation/timeout.
  // Отдельно защищаем окончательную остановку при выходе из потока.
  try {
    // Не вызываем Stop, если движение/владение никогда не требовали остановки.
    if (needs_stop_) {
      // Ожидаем общий доступ к SDK перед заключительным Stop.
      auto lock = sdk_lock(config_.sdk_lock_timeout);
      // Не вызываем Stop без блокировки; сообщаем о невозможности получить её вовремя.
      if (!lock.owns_lock())
        throw std::runtime_error("SDK gate timeout during final Stop");
      // Проверяем результат завершающей остановки и очистки очереди.
      if (!transport_->stop()) {
        // Отмечаем неуспешность окончательной остановки.
        stop_failed_ = true;
        // Фиксируем ошибку Stop на этапе завершения процесса.
        fault("SDK Stop or ClearUnfinishedTrajectory failed during shutdown");
        // При успешном возврате остановки снимаем оставшуюся обязанность Stop.
      } else
        // Сбрасываем флаг необходимости аппаратной остановки.
        needs_stop_ = false;
      // Завершаем условную остановку задействованного управления.
    }
    // Перехватываем любые исключения заключительного Stop.
  } catch (...) {
    // Отмечаем, что успешное выполнение остановки не подтверждено.
    stop_failed_ = true;
    // Фиксируем исключение в процедуре остановки.
    fault("exception during SDK stop");
    // Завершаем защиту заключительной остановки от исключений.
  }
  // Отдельно защищаем отключение транспорта от исключений.
  try {
    // Получаем общий доступ к SDK для закрытия соединений.
    auto lock = sdk_lock(config_.sdk_lock_timeout);
    // Не закрываем соединения параллельно другому SDK-вызову при таймауте блокировки.
    if (!lock.owns_lock())
      throw std::runtime_error("SDK gate timeout during disconnect; connections not closed");
    // Закрываем SDK-соединения в том же рабочем потоке, который ими владеет.
    transport_->disconnect();
    // Обрабатываем стандартное исключение при отключении транспорта.
  } catch (const std::exception &e) {
    // Выводим подробную причину неуспешного отключения в журнал.
    RCLCPP_ERROR(node_->get_logger(), "SDK disconnect failed: %s", e.what());
    // Фиксируем ошибку отключения, сохраняя первичную причину, если отказ уже был.
    fault(std::string("SDK disconnect failed: ") + e.what());
    // Обрабатываем исключения отключения без стандартного текстового описания.
  } catch (...) {
    // Пишем в журнал, что тип исключения отключения неизвестен.
    RCLCPP_ERROR(node_->get_logger(), "SDK disconnect failed: unknown exception");
    // Фиксируем неизвестную ошибку отключения транспорта.
    fault("SDK disconnect failed: unknown exception");
    // Завершаем обработку ошибок закрытия SDK-соединений.
  }
  // Завершение процедуры.
}

// Главная функция единственного рабочего потока, владеющего SDK-вызовами.
void IimtSystem::run() {
  // Подключение и завершение остаются в единственном SDK-потоке.
  // Перехватываем ошибки настройки и основного цикла SDK-потока.
  try {
    // Настраиваем поток и выполняем Login.
    connect_worker();
    // Создаём локальную историю цикла после Login.
    WorkerState state(config_.period);
    // Работаем до запроса завершения SDK-потока.
    while (!terminate_) {
      // Переносим зафиксированный отказ другой руки в локальную аппаратную ошибку.
      if (group_faulted() && !faulted_)
        fault("coordination peer fault; process restart required");
      // Повторяем цикл при занятом SDK на Stop.
      if (!service_worker_stop(state))
        continue;
      // При зафиксированном отказе прекращаем обычные чтения и команды.
      if (faulted_) {
        // Захватываем мьютекс условной переменной ожидания.
        std::unique_lock<std::mutex> l(wait_mutex_);
        // Спим до 10 мс либо до события завершения или новой остановки.
        wake_.wait_for(l, std::chrono::milliseconds(10), [this] {
          // Проверяем признаки, позволяющие досрочно разбудить аварийное ожидание.
          return terminate_.load() || stop_requested_.load();
          // Завершаем ожидание с автоматическим восстановлением блокировки.
        });
        // Начинаем новую итерацию без отправки ServoJ после отказа.
        continue;
        // Завершаем режим ожидания при аварии.
      }
      // Запоминаем начало работы итерации для общего бюджета отправки и чтения.
      const auto work_begin = Clock::now();
      // Пока в этой итерации не отправлено ни одной новой команды.
      bool sent_this_iteration = false;
      // CM is the only command clock. Never resend an old mailbox sequence.
      // Sending before polling removes feedback-query latency from dispatch.
      // Ошибка команды завершает итерацию.
      if (!dispatch_worker_command(state, sent_this_iteration))
        continue;
      // Сохраняем прежнее завершение итерации при ошибке чтения.
      if (!poll_worker_feedback(state, sent_this_iteration))
        continue;
      // Проверяем полный бюджет итерации только если она действительно отправила команду.
      if (sent_this_iteration &&
          // Сравниваем суммарное время обработки, отправки и возможного чтения с периодом ServoJ.
          std::chrono::duration<double>(Clock::now() - work_begin).count() > config_.period) {
        // Фиксируем превышение общего периода даже при допустимых отдельных вызовах SDK.
        fault("combined SDK send/read cycle exceeded servo.period");
        // Переходим к обработке отказа по суммарному времени цикла.
        continue;
        // Завершаем проверку общего бюджета итерации.
      }
      // Просыпаемся по команде, Stop или сроку.
      wait_worker_event(state);
      // Завершаем основной цикл рабочего потока.
    }
    // Перехватываем стандартные исключения настройки, транспорта и цикла управления.
  } catch (const std::exception &e) {
    // Фиксируем текст исключения как аппаратный отказ.
    fault(e.what());
    // Завершаем защиту основного тела SDK-потока от исключений.
  }
  // Финальная очистка также принадлежит SDK-потоку.
  disconnect_worker();
  // Завершаем работу функции SDK-потока.
}
// Обслуживаем неблокирующий запрос отмены целей настроенного контроллера.
void IimtSystem::cancel_controller() {
  // Сначала проверяем уже отправленный запрос отмены, если он существует.
  if (cancel_pending_) {
    // Проверяем готовность future без ожидания, с нулевым таймаутом.
    if (cancel_pending_->wait_for(std::chrono::seconds(0)) ==
        // Продолжаем обработку только если ответ службы уже получен.
        std::future_status::ready) {
      // Получаем ответ завершённого запроса отмены.
      auto result = cancel_pending_->get();
      // Сохраняем признак принятия запроса, а не завершения движения.
      cancel_accepted_ =
          // Требуем успешный код ответа и хотя бы одну цель, переходящую к отмене.
          result->return_code == 0 && !result->goals_canceling.empty();
      // При принятии отмечаем, что конечный статус цели ещё не подтверждён.
      cancel_status_ = cancel_accepted_
                           ? "accepted_not_terminal"
                           // Иначе отмечаем отказ отмены либо отсутствие активной цели.
                           : "not_accepted_or_no_active_goal";
      // Освобождаем future уже обработанного запроса.
      cancel_pending_.reset();
      // Если ответа нет, проверяем возраст отправленного запроса отмены.
    } else if (stale(cancel_sent_at_, Clock::now(),
                     // Используем настроенный таймаут ответа службы отмены.
                     config_.controller_cancel_timeout)) {
      // Удаляем просроченный запрос из списка ожиданий ROS-клиента.
      cancel_client_->remove_pending_request(*cancel_pending_);
      // Сбрасываем локальное ожидание ответа отмены.
      cancel_pending_.reset();
      // Явно помечаем отсутствие подтверждения отмены из-за таймаута.
      cancel_status_ = "timeout_not_confirmed";
      // Завершаем обработку завершённого или просроченного запроса.
    }
    // Завершаем обслуживание ранее отправленной отмены.
  }
  // Не отправляем отмену без отказа, в режиме без движения или повторно после запроса.
  if (!faulted_ || !config_.enable_motion || cancel_requested_ ||
      // Также исключаем отправку при отсутствующем клиенте службы отмены.
      !cancel_client_)
    // Выходим, если условия для нового запроса отмены не выполнены.
    return;
  // Проверяем наличие сервера настроенной службы отмены.
  if (!cancel_client_->service_is_ready()) {
    // Отмечаем недоступность сервиса, сохраняя возможность проверить его позднее.
    cancel_status_ = "service_unavailable";
    // Не отправляем запрос в отсутствующую службу.
    return;
    // Завершаем обработку недоступности сервиса отмены.
  }
  // A zero goal ID and timestamp request cancellation of all goals on this
  // configured JTC only.
  // Создаём запрос с нулевыми ID и временем, означающий отмену всех целей этого action-сервера.
  auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
  // Асинхронно отправляем запрос и сохраняем future для последующего опроса.
  cancel_pending_.emplace(cancel_client_->async_send_request(request));
  // Отмечаем факт отправки, чтобы не дублировать отмену на каждом диагностическом тике.
  cancel_requested_ = true;
  // Запоминаем монотонное время отправки для контроля таймаута ответа.
  cancel_sent_at_ = Clock::now();
  // Публикуем промежуточный статус без утверждения об успешной остановке.
  cancel_status_ = "request_sent_not_confirmed";
  // Завершаем обслуживание отмены целей контроллера.
}
// Проверяем состояние и формируем периодическое диагностическое сообщение.
void IimtSystem::diagnostics() {
  // Отражаем отказ координационной группы в локальной аппаратной ошибке.
  if (group_faulted() && !faulted_)
    fault("coordination peer fault; process restart required");
  // Ограничиваем блокировку проверкой метки обратной связи.
  {
    // Защищаем чтение метки от обновления SDK-потоком.
    std::lock_guard<std::mutex> l(state_mutex_);
    // Контролируем просрочку имеющегося измерения, исключая незадействованную руку группы.
    if ((!group_fault_ || claimed_ || needs_stop_) && feedback_stamp_ != Clock::time_point{} &&
        // Проверяем возраст измерения по настроенному таймауту состояния.
        stale(feedback_stamp_, Clock::now(), config_.state_timeout))
      // Фиксируем срабатывание программного контроля свежести.
      fault(
          // Уточняем, что этот контроль не может прервать зависший SDK-вызов.
          "feedback watchdog expired; blocking SDK call cannot be interrupted");
    // Освобождаем мьютекс состояния после проверки свежести.
  }
  // Защищаем обслуживание ROS-отмены от исключений.
  try {
    // Проверяем ответ отмены и при необходимости отправляем новый запрос.
    cancel_controller();
    // Перехватываем стандартные ошибки запроса отмены.
  } catch (const std::exception &e) {
    // Блокируем повторную отправку после исключения процедуры отмены.
    cancel_requested_ = true;
    // Сохраняем текст исключения в диагностическом статусе отмены.
    cancel_status_ = std::string("request_exception: ") + e.what();
    // Записываем ошибку запроса отмены JTC в журнал ноды.
    RCLCPP_ERROR(node_->get_logger(), "JTC cancel request failed: %s",
                 // Подставляем подробное описание возникшего исключения.
                 e.what());
    // Завершаем обработку ошибок процедуры отмены.
  }
  // Создаём стандартный ROS-массив диагностических статусов.
  diagnostic_msgs::msg::DiagnosticArray message;
  // Ставим ROS-временную метку публикации диагностического сообщения.
  message.header.stamp = node_->now();
  // Создаём отдельный статус этого hardware-интерфейса.
  diagnostic_msgs::msg::DiagnosticStatus status;
  // Используем имя hardware-ноды как имя диагностического источника.
  status.name = config_.hardware_node_name;
  // Идентифицируем физический контроллер его IP-адресом.
  status.hardware_id = config_.robot_ip;
  // При зафиксированном аппаратном отказе выставляем уровень ERROR.
  status.level = faulted_ ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                          // Иначе начинаем с уровня OK, который ниже может быть повышен до WARN.
                          : diagnostic_msgs::msg::DiagnosticStatus::OK;
  // Ограничиваем блокировку копированием текста первичной ошибки.
  {
    // Защищаем строку ошибки от конкурентной записи рабочего потока.
    std::lock_guard<std::mutex> l(fault_mutex_);
    // Выбираем человекочитаемое описание текущего состояния.
    status.message =
        // Передаём первичную ошибку либо напоминание, что рабочий поток не заменяет аппаратный watchdog.
        faulted_ ? fault_text_ : "worker active; not a hardware watchdog";
    // Освобождаем блокировку текста отказа.
  }
  // Определяем помощник добавления строковой пары ключ–значение в диагностику.
  auto add = [&](const std::string &key, const std::string &value) {
    // Создаём элемент стандартного сообщения диагностического значения.
    diagnostic_msgs::msg::KeyValue item;
    // Заполняем имя диагностического поля.
    item.key = key;
    // Заполняем строковое представление диагностического значения.
    item.value = value;
    // Добавляем поле к списку значений текущего статуса.
    status.values.push_back(item);
    // Завершаем локальный помощник формирования диагностических полей.
  };
  // Создаём локальную копию временной метки последнего измерения.
  Clock::time_point stamp;
  // Ограничиваем блокировку копированием метки состояния.
  {
    // Защищаем согласованное чтение временной метки обратной связи.
    std::lock_guard<std::mutex> l(state_mutex_);
    // Копируем момент последнего успешно принятого измерения.
    stamp = feedback_stamp_;
    // Освобождаем мьютекс состояния перед форматированием диагностики.
  }
  // При отсутствии аварии отдельно предупреждаем об отсутствующей или старой обратной связи.
  if (!faulted_ && stale(stamp, Clock::now(), config_.state_timeout)) {
    // Повышаем уровень состояния до предупреждения.
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    // Объясняем, что для активации обязательно свежее измерение.
    status.message = "idle feedback unavailable/stale; activation requires fresh measurement";
    // Завершаем предупреждение об устаревшем состоянии без аварийного отказа.
  }
  // Проверяем историю превышений бюджета чтения в незадействованном режиме без текущей аварии.
  if (!faulted_ && unclaimed_read_overruns_ > 0 &&
      // Не заменяем уже выбранное более важное предупреждение.
      status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    // Отмечаем предупреждением неподтверждённую временную пригодность SDK.
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    // Сообщаем о медленных чтениях без владения, не объявляя движение проверенным.
    status.message = "observed unclaimed SDK read overruns; motion timing not qualified";
    // Завершаем предупреждение по историческим превышениям бюджета чтения.
  }
  // Публикуем настроенный бюджет одного SDK-вызова в секундах.
  add("sdk_budget_s", std::to_string(config_.sdk_budget));
  // Публикуем номинальный период ServoJ в секундах.
  add("servo_period_s", std::to_string(config_.period));
  // Публикуем длительность последнего завершившегося чтения SDK.
  add("last_read_duration_s", std::to_string(last_read_duration_.load()));
  // Публикуем максимальную наблюдавшуюся длительность чтения SDK.
  add("max_read_duration_s", std::to_string(max_read_duration_.load()));
  // Публикуем число завершившихся вызовов чтения, включая неуспешные.
  add("read_calls", std::to_string(read_calls_.load()));
  // Публикуем число чтений, превысивших временной бюджет.
  add("read_budget_overruns", std::to_string(read_budget_overruns_.load()));
  // Публикуем число успешных медленных чтений без владения управлением.
  add("unclaimed_read_budget_overruns", std::to_string(unclaimed_read_overruns_.load()));
  // Публикуем длительность последнего завершившегося вызова ServoJ.
  add("last_servoj_duration_s", std::to_string(last_servoj_duration_.load()));
  // Публикуем максимальную наблюдавшуюся длительность ServoJ.
  add("max_servoj_duration_s", std::to_string(max_servoj_duration_.load()));
  // Публикуем число завершившихся вызовов ServoJ, включая неуспешные.
  add("servoj_calls", std::to_string(servoj_calls_.load()));
  // Публикуем число превышений временного бюджета вызовами ServoJ.
  add("servoj_budget_overruns", std::to_string(servoj_budget_overruns_.load()));
  // Публикуем имя координационной группы, пустое для независимой работы.
  add("coordination_group", config_.coordination_group);
  // Публикуем состояние общего фиксируемого отказа группы.
  add("coordination_fault", group_faulted() ? "true" : "false");
  // Начинаем формирование поля возраста последнего измерения.
  add("feedback_age_s",
      // Проверяем наличие хотя бы одной временной метки измерения.
      stamp == Clock::time_point{}
          // При отсутствии измерения указываем недоступность вместо фиктивного возраста.
          ? "unavailable"
          // Иначе преобразуем численное значение возраста в строку.
          : std::to_string(
                // Вычисляем секунды от последнего успешного чтения до текущего момента.
                std::chrono::duration<double>(Clock::now() - stamp).count()));
  // Получаем последний образец команды для диагностики его возраста.
  auto cmd = mailbox_.get();
  // Начинаем поле возраста команды с проверки наличия временной метки.
  add("command_age_s",
      cmd.stamp == Clock::time_point{} // При отсутствии команды публикуем признак недоступности.
          ? "unavailable"
          // Иначе переводим монотонную длительность возраста команды в строку.
          : std::to_string(std::chrono::duration<double>(
                               // Вычисляем разность текущего времени и момента поступления команды.
                               Clock::now() - cmd.stamp)
                               // Извлекаем численное значение длительности в секундах.
                               .count()));
  // Публикуем длительность последнего измеренного вызова чтения или ServoJ.
  add("last_sdk_call_duration_s", std::to_string(last_sdk_duration_.load()));
  // Публикуем максимум длительности измеренных вызовов чтения и ServoJ.
  add("max_sdk_call_duration_s", std::to_string(max_sdk_duration_.load()));
  // Публикуем интервал между началами двух последних вызовов отправки.
  add("last_send_interval_s", std::to_string(last_send_interval_.load()));
  // Публикуем задержку от временной защитной отметки до начала последней отправки.
  add("last_guard_to_send_delay_s", std::to_string(last_guard_to_send_delay_.load()));
  // Начинаем поле возраста команды на момент её отправки в SDK.
  add("last_command_age_at_send_s",
      // Преобразуем сохранённый возраст последнего отправленного образца в строку.
      std::to_string(last_command_age_at_send_.load()));
  // Публикуем максимальный возраст команды, наблюдавшийся перед защитой опоздания.
  add("max_lateness_observed_s", std::to_string(max_lateness_observed_.load()));
  // Публикуем факт запроса отмены целей, не подтверждение остановки.
  add("cancel_requested", cancel_requested_ ? "true" : "false");
  // Публикуем принятие отмены action-сервером, не конечный статус цели.
  add("cancel_accepted", cancel_accepted_ ? "true" : "false");
  // Публикуем подробный строковый статус процедуры отмены.
  add("cancel_status", cancel_status_);
  // Публикуем признак неуспешной попытки Stop.
  add("stop_failed", stop_failed_ ? "true" : "false");
  // Публикуем число потерянных или не допускаемых к перезаписи образцов почтового ящика.
  add("dropped_mailbox_commands", std::to_string(dropped_));
  // Публикуем число команд, прошедших отправку и проверку временного бюджета.
  add("sent", std::to_string(sent_));
  // Публикуем число пропущенных слотов опроса обратной связи.
  add("skipped_feedback_slots", std::to_string(skipped_));
  // Публикуем признак незавершённой процедуры Stop и подтверждения состояния после неё.
  add("stop_pending", stop_pending_ ? "true" : "false");
  // Публикуем текущий факт владения командными интерфейсами контроллером.
  add("controller_claimed", claimed_ ? "true" : "false");
  // Публикуем настройку разрешения отправки команд движения.
  add("enable_motion", config_.enable_motion ? "true" : "false");
  // Добавляем сформированный статус hardware в диагностический массив.
  message.status.push_back(status);
  // Публикуем массив в приватный топик diagnostics этой ноды.
  diagnostic_pub_->publish(message);
  // Завершаем формирование и публикацию диагностики.
}
// namespace iimt_ros2_control // Закрываем пространство имён пакета аппаратного интерфейса.
} // namespace iimt_ros2_control
// Регистрируем класс IimtSystem как загружаемый плагин pluginlib.
PLUGINLIB_EXPORT_CLASS(iimt_ros2_control::IimtSystem,
                       // Указываем SystemInterface как базовый интерфейс плагина ros2_control.
                       hardware_interface::SystemInterface)
