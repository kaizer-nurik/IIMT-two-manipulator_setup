# Параметры и единицы

Таблицы описывают исходники этой версии, а не универсальные параметры производителя.
Параметры direct driver, координатора и workspace объявлены **read-only**: задавайте
их до старта в YAML/launch и перезапускайте стек. `ros2 param set` не является
поддерживаемой горячей настройкой. Изменение файла в `src` не меняет уже запущенный
процесс; при Docker-сборке исходники встроены в образ, поэтому нужен rebuild.

## 1. Direct driver

Источник: `iimt_ros2_driver/src/iimt_ros2_driver.cpp` → `configure()` и
`iimt_ros2_driver/config/driver.yaml`. Single launch загружает YAML; dual использует
`iimt_dual_control/config/{left,right}_driver.yaml` плюс launch overrides.

| Параметр | Single YAML / dual profile | Единицы и назначение |
|---|---|---|
| `robot_ip` | `192.168.50.148` / задан launch | IPv4/адрес контроллера; Compose использует примерные IP, но по умолчанию mock |
| `port` | 29999 | TCP-порт dashboard/SDK login, 1…65535 |
| `joint_prefix` | `""` / `left_`, `right_` | Префикс имён всех шести суставов; буквы, цифры, `_` |
| `enable_motion` | false / false | Разрешение принимать движения; **не** включает питание/тормоза |
| `use_mock_hardware` | false / true | В true SDK-соединения не открываются; single Compose явно переопределяет в true |
| `command_rate` | 30 / 125 | Hz, 1…125; частота таймера команд |
| `read_rate` | 30 / 125 | Hz, 1…125; целевая частота независимого reader |
| `trajectory_advance_ms` | 0 / 0 | ms, 0…1000; выдача команд раньше номинального времени |
| `feedback_transport` | tcp / tcp | `tcp` — deadline reader; `sdk` — блокирующий API_GetRobotJointRad |
| `read_timeout` | 0.1 / 0.1 | s, >0 и ≤ state_timeout; deadline TCP-транзакции, **не** SDK timeout |
| `state_timeout` | 0.2 / 0.2 | s, >0; допустимый возраст последнего успешного состояния |
| `max_lateness` | 0.1 / 0.1 | s, >0; мягкий allowance timing-overrun, WARN без retiming |
| `clock_jump_tolerance` | 0.1 / 0.1 | s, >0; строгая проверка расхождения ROS/steady clock |
| `diagnostics_period` | 1 / 1 | s, >0; период DiagnosticArray |
| `start_tolerance` | 0.02 / 0.02 | rad, >0; несовпадение первой точки t=0 с измерением |
| `path_tolerance` | **3.0 / 3.0** | rad, >0; позиционный допуск по умолчанию во время пути |
| `goal_tolerance` | 0.003 / 0.003 | rad, >0; допуск конечной позиции |
| `goal_time_tolerance` | 2 / 2 | s, >0; дополнительное ожидание конечной позиции |
| `servo.a` | 5 / 5 | Параметр ускорения SDK, положительный; единицы/реальный эффект уточняются по firmware |
| `servo.v` | 3 / 3 | Параметр скорости SDK, положительный; не per-point velocity |
| `servo.period` | 1/command_rate | s; должен точно соответствовать командному периоду (проверка 1e−9) |
| `servo.lookahead_time` | 0.012 / 0.012 | s, ≥0; параметр SDK, не гарантированная полная задержка |
| `servo.gain` | 0 / 0 | ≥0; SDK gain, не доказанный универсальный регулятор компенсации lag |
| `use_sim_time` | false | ROS-параметр; real hardware запрещает true |

**3 rad ≈172° — очень широкий path tolerance**, не показатель высокой точности и
не безопасная универсальная рекомендация. Не увеличивайте его для маскировки
`SDK Stop/Clear failed`, устаревшего состояния или неверных имён суставов.

Вызов executable без профиля имеет иные C++ fallback: `feedback_transport=sdk`,
`servo.a=7`, `path_tolerance=0.05`, `goal_time_tolerance=0.25`,
`limits.max_velocity=[1,1,1,1,1,1]`,
`limits.max_acceleration=[1.875,2,2,2,2,2]`. Остальные перечисленные scalar fallback
соответствуют single-столбцу. Для воспроизводимости запускайте через launch с профилем.

### Массивы ограничений

Все содержат **шесть** конечных чисел, порядок:
`shoulder_joint`, `upper_arm_joint`, `forearm_joint`, `wrist_1_joint`,
`wrist_2_joint`, `wrist_3_joint` (до добавления префикса).

| Параметр | Значения активных YAML (single и dual) | Единицы |
|---|---|---|
| `limits.min_position` | `[-6.2832,-2.6167,-2.8783,-3.0528,-3.0528,-6.2832]` | rad |
| `limits.max_position` | `[6.2832,2.6167,2.8783,3.0528,3.0528,6.2832]` | rad |
| `limits.max_velocity` | `[3.1415,3.1415,3.1415,3.1415,3.1415,3.1415]` | rad/s, >0 |
| `limits.max_acceleration` | `[1.875,2.5,3.75,3.125,3.75,3.75]` | rad/s², >0 |
| `limits.max_step` | `[0.02,0.02,0.02,0.02,0.02,0.02]` | rad/командный шаг, >0 |

Минимум должен быть меньше максимума. Это программные commissioning-ограничения,
не сертифицированные пределы стенда. `max_step=0.02` при 30 Hz даёт номинальную
планировочную границу 0.6 rad/s до применения planning scale; при 125 Hz — 2.5 rad/s.
Поэтому изменение command_rate меняет не только гладкость дискретизации.

### Допуски action-сообщения

`path_tolerance` и `goal_tolerance` в FollowJointTrajectory могут задавать
позиционный допуск по имени каждого сустава. Ноль использует default драйвера;
положительное значение переопределяет его. Отрицательные значения и ненулевые
velocity/acceleration tolerances не поддерживаются. Не предполагайте, что ROS-конвенция
«−1 отключает проверку» реализована здесь. Положительный `goal_time_tolerance`
сообщения переопределяет default; отсутствие/ноль использует параметр.

## 2. Launch одной руки

`iimt_ros2_driver/launch/iimt_ros2_driver.launch.py`:

| Аргумент | По умолчанию | Правило |
|---|---|---|
| `parameters_file` | установленный `iimt_ros2_driver/config/driver.yaml` | YAML с `/iimt_ros2_driver: ros__parameters:` |
| `robot_ip`, `joint_prefix` | пусто | Непустое значение переопределяет YAML |
| `command_rate`, `read_rate`, `read_timeout`, `feedback_transport`, `trajectory_advance_ms` | пусто | Пусто сохраняет YAML, непусто переопределяет |
| `enable_motion`, `use_mock_hardware` | false | Launch явно задаёт bool поверх YAML |

`iimt_ros2_moveit_config/launch/iimt_driver_moveit.launch.py` добавляет:

| Аргумент | По умолчанию | Назначение |
|---|---|---|
| `robot_type` | 5 | Только CI-05; другие типы этот direct MoveIt launch отвергает |
| `launch_rviz` | true | true/false; Compose делает false для headless |
| `planning_limit_scale` | 0.5 | (0,1]; запас planning velocity/acceleration |

Этот launch передаёт перечисленные driver arguments **кроме joint_prefix**:
модель и SRDF однорукого стека используют непрефиксованные имена.

## 3. Launch двух рук и координатор

`iimt_dual_control/launch/dual_control.launch.py`:

| Аргумент | Default | Назначение |
|---|---|---|
| `use_mock_hardware` | true | Один режим для обеих рук |
| `enable_motion` | false | Одно разрешение движения для обеих рук |
| `robot_type` | 5 | Только CI-05 |
| `left_robot_ip`, `right_robot_ip` | пусто | В mock заменяются на loopback; real требует два разных валидных IPv4 |
| `left_parameters_file`, `right_parameters_file` | профили `config/{left,right}_driver.yaml` | Ровно один ROS mapping; допустим wildcard `/**` |
| `{left,right}_command_rate` | 125 | Hz; override профиля |
| `{left,right}_read_rate` | 125 | Hz; override профиля |
| `{left,right}_read_timeout` | 0.1 | s; override профиля |
| `{left,right}_feedback_transport` | tcp | override профиля |
| `{left,right}_trajectory_advance_ms` | 0 | ms; override драйвера и согласованное значение координатора |
| `dispatch_lead_time_s` | 1.0 | Запас для zero-stamp целей MoveIt |
| `dispatch_margin_s` | 0.2 | Резерв завершения handshake до раннего pre-roll |

В отличие от single launch, непустые defaults dual override действуют **всегда**.
Одной правки частоты в arm YAML недостаточно: изменяйте соответствующий launch argument.

Полный набор read-only параметров `/iimt_dual_coordinator`:

| Параметр | Default | Значение |
|---|---|---|
| `dispatch_lead_time_s` | 1.0 s | ≥ max(advance)/1000 + dispatch_margin_s |
| `dispatch_margin_s` | 0.2 s | Резерв перед первой допустимой отправкой |
| `left_trajectory_advance_ms`, `right_trajectory_advance_ms` | 0 ms | Должны совпадать с обоими драйверами |
| `acceptance_timeout_s` | 2.0 s | Ожидание принятия дочерних целей, также ограничено расписанием |
| `cancel_timeout_s` | 3.0 s | Ожидание подтверждения завершения отмены |
| `completion_margin_s` | 5.0 s | Запас ожидания общего завершения |
| `diagnostics_timeout_s` | 3.0 s | Свежесть diagnostics для допуска цели |

Все конечны и неотрицательны. Последние четыре сейчас **не exposed как launch
arguments**: не передавайте их в `dual_moveit.launch.py`, ожидая настройки ноды.
Их изменение требует явной настройки Node parameters в bringup; документируем
интерфейс как реализован, без обещания несуществующего CLI-проброса.

## 4. Dual MoveIt, базы, workspace и пол

`iimt_dual_moveit_config/launch/dual_moveit.launch.py` включает все аргументы dual control:

| Аргумент | Default | Единицы/система |
|---|---|---|
| `launch_rviz` | true | bool |
| `planning_limit_scale` | 0.5 | 0.000001…1 |
| `left_base_xyz` | `-0.0008297 -0.5380000 0.0818739` | m, world → левая база |
| `left_base_rpy` | `0 0 -0.6981317008` | rad, roll pitch yaw |
| `right_base_xyz` | `-0.345440489323 0.411679944176 -0.082000000000` | m, **левая база → правая база** |
| `right_base_rpy` | `0 0 0` | rad, относительно левой базы |
| `workspace_margin` | 0.5 | m; общий отступ от охватывающего обе базы прямоугольника |
| `workspace_x_min_margin`, `workspace_x_max_margin` | наследуют workspace_margin | m, стороны −X/+X в world |
| `workspace_y_min_margin`, `workspace_y_max_margin` | наследуют workspace_margin | m, стороны −Y/+Y в world |
| `floor_z` | auto | world z в m; auto = меньший z двух баз |

XYZ/RPY — три конечных числа через пробел. Положительные margins допускаются до100 m
launch-валидатором; это диапазон ввода, не рекомендация для стенда.

Матрицы связываются так: `T_world_right = T_world_left × T_left_right`.
С текущими defaults правая база в world приблизительно
`[-0.0008297,-0.00059,-0.0001261]` m, пол z=−0.0001261 m.
При margins .5 получаются X `[-0.5008297,0.4991703]`, Y `[-1.038,0.49941]` m.
Это значения из исходной модели крепления, **не доказательство калибровки стенда**.

Нода `/iimt_workspace` принимает read-only:
`left_base_xyz`, `left_base_rpy`, `right_base_xyz`, `workspace_margin`, четыре
сторонних margin, числовой `floor_z`; публикует вычисляемый параметр `bounds_xyz`
в порядке `[xmin,ymin,xmax,ymax,zmin,zmax]` для box-областей.
`right_base_rpy` не нужен вычислению положения основания и не является параметром
workspace-ноды; он применяется URDF. Самостоятельные defaults workspace-ноды
(без launch) — левая база в нуле, правая `[0,1.2,0]`; штатный launch всегда передаёт
реальные настройки модели. Не путайте самостоятельные defaults с приведённой геометрией.

## 5. MoveIt-конфигурация

| Параметр | Single | Dual |
|---|---|---|
| `allow_trajectory_execution` | true | true |
| `moveit_manage_controllers` | false | false |
| `trajectory_execution.execution_duration_monitoring` | true | true |
| `allowed_execution_duration_scaling` | 1.1 | 1.1 |
| `allowed_goal_duration_margin` | 0.5 s + advance | 2 s + dispatch_lead_time_s + max(advance) |
| `allowed_start_tolerance` | 0.02 rad | 0.02 rad |
| `start_state_max_bounds_error` | 0.02 rad | 0.02 rad |
| KDL search resolution | 0.005 | 0.005 |
| KDL timeout | 0.005 s | 0.05 s на каждую arm-группу |

Single группа `iimt_group` содержит набор OMPL-планировщиков в
`config/ompl_planning.yaml`; dual явно использует RRTConnect, `range=0` означает
автонастройку OMPL. Параметры библиотечных узлов MoveIt/RViz/robot_state_publisher
сверх включённых YAML не перечисляются как собственные параметры проекта.

## 6. Legacy-параметры

Текущие launch **не читают** `iimt_ros2_control/config/hardware.yaml` и
`iimt_ros2_moveit_config/config/ros2_controllers.yaml`. Изменение `sdk_budget` там
не исправит direct driver. Для исторического пути сохранены исходные файлы и
комментарии; не переносите эти параметры в новый YAML.

Основные legacy startup-настройки `/iimt_hardware`: `robot_ip`, `port`,
`sdk_library_path`, `worker_priority`, `require_realtime`, `enable_motion`,
`lock_directory`; `servo.{a,v,period,lookahead_time,gain}`; `state_period`,
`sdk_lock_timeout`, `sdk_budget`, `command_timeout`, `state_timeout`,
`startup_timeout`, `max_lateness`, `command_jitter_tolerance`, `ros_clock_jump`,
`diagnostics_period`, `controller_cancel_service`, `controller_cancel_timeout`;
`limits.{min_position,max_position,max_velocity,max_acceleration,max_step,tracking_tolerance}`.
Их значения/проверки — в сохранённом YAML и исходнике SystemInterface. Это
совместимость, не рекомендуемый commissioning-путь и не API новых notebooks.

### Legacy hardware.yaml: значения сохранённого профиля

| Параметр | Default профиля | Назначение |
|---|---|---|
| `robot_ip`, `port` | `192.168.50.148`, 29999 | SDK адрес и порт |
| `sdk_library_path` | пусто | Launch real-режима подставляет установленную libiimtcl.so |
| `worker_priority`, `require_realtime` | 0, false | Приоритет worker; обязательность успешной RT-настройки |
| `enable_motion` | false | Startup-разрешение команд |
| `lock_directory` | `/tmp` | Каталог cooperative process lock, не блокировка legacy-клиентов |
| `servo.a`, `servo.v` | 7, 3 | SDK servo-параметры, не derivative каждой точки |
| `servo.period`, `state_period` | 1/30 s, 1/30 s | Командный и измерительный периоды |
| `servo.lookahead_time`, `servo.gain` | 0.012 s, 0 | SDK-поля |
| `sdk_lock_timeout` | 0.006 s | Ожидание SDK mutex, не прерывание вызова |
| `sdk_budget` | 0.025 s | Бюджет вызова, превышение обнаруживается после возврата |
| `command_timeout`, `state_timeout` | 0.1 s, 0.1 s | Возраст команды и feedback |
| `startup_timeout` | 10 s | Подключение/начальное состояние |
| `max_lateness` | 0.016 s | Допустимое опоздание dispatch |
| `command_jitter_tolerance` | 0.004 s | Отклонение интервала отправки |
| `ros_clock_jump` | 0.1 s | ROS/steady расхождение |
| `diagnostics_period` | 1 s | Диагностика |
| `controller_cancel_service` | `/iimt_group_controller/follow_joint_trajectory/_action/cancel_goal` | Сервис отмены JTC |
| `controller_cancel_timeout` | 1 s | Ожидание cancel |
| `limits.min_position`, `limits.max_position` | Как active CI-05 таблица | rad |
| `limits.max_velocity` | `[1,1,1,1,1,1]` | rad/s |
| `limits.max_acceleration` | `[1.875,2,2,2,2,2]` | rad/s² |
| `limits.max_step` | `[0.02,0.02,0.02,0.02,0.02,0.02]` | rad |
| `limits.tracking_tolerance` | `[0.2,0.2,0.2,0.2,0.2,0.2]` | rad |

Дополнительно legacy SystemInterface объявляет `joint_prefix` (обычно пусто),
`hardware_node_name` (`iimt_hardware`), `coordination_group` (обычно пусто).
Они выбираются через URDF HardwareInfo **до** создания ROS-ноды; одной записью
в YAML их идентичность менять нельзя. Все эти параметры read-only.
`controller_update_rate` — HardwareInfo-проверка согласованности с1/servo.period,
а не параметр современного direct driver.

### Legacy controller_manager / JTC

Источник: `iimt_ros2_moveit_config/config/ros2_controllers.yaml`.

| Параметр | Default | Значение |
|---|---|---|
| `controller_manager.update_rate` | 30 Hz | Должен совпадать с1/servo.period |
| `controller_manager.use_sim_time` | false | ROS clock |
| `iimt_group_controller.type` | `joint_trajectory_controller/JointTrajectoryController` | Loaded plugin |
| `joint_state_broadcaster.type` | `joint_state_broadcaster/JointStateBroadcaster` | State publisher plugin |
| `joints` | шесть непрефиксованных имён из active таблицы | Порядок JTC |
| `command_interfaces` | `[position]` | Командный интерфейс |
| `state_interfaces` | `[position,velocity]` | Измерительные интерфейсы |
| `open_loop_control` | false | Измеренное состояние вместо open-loop |
| `set_last_command_interface_value_as_state_on_activation` | false | Не подменять старт последней командой |
| `allow_partial_joints_goal` | false | Только полный набор |
| `allow_integration_in_goal_trajectories` | false | Не интегрировать неполное задание |
| `allow_nonzero_velocity_at_trajectory_end` | true | Допускает ненулевую конечную velocity |
| `interpolation_method` | splines | Интерполяция JTC |
| `state_publish_rate`, `action_monitor_rate` | 30 Hz, 50 Hz | Публикация состояния / action monitoring |
| `constraints.goal_time` | 0.25 s | Запас достижения цели |
| `constraints.stopped_velocity_tolerance` | 0.01 rad/s | Настройка JTC, не доказательство аппаратной остановки |
| `constraints.<каждый сустав>.trajectory` | 0.05 rad | Путевой допуск |
| `constraints.<каждый сустав>.goal` | 0.003 rad | Конечный допуск |

Это собственные значения legacy-конфига, а не исчерпывающая документация всех
параметров стороннего JTC. Их динамическая поддержка определяется установленной
версией ros2_controllers, не новым driver API.

### Legacy launch и мост

Старый `iimt_moveit.launch.py` принимает `robot_ip=192.168.50.148`, `robot_type=5`,
`enable_motion=false`, `use_mock_hardware=false`, `use_sim_time=false`,
`launch_rviz=true`, `db=false`, `sdk_library_path=""`,
`hardware_parameters_file` (hardware.yaml), `controllers_file` (ros2_controllers.yaml),
`moveit_parameters_file` (trajectory_execution.yaml). CI03/CI10 допустимы там только
в mock. `db=true` запускает optional warehouse Mongo на localhost:33829.

Нода старого моста `/iimt_ros2_trajectory_action` читает один custom parameter:
`joint_name.controller_joint_names`, массив шести строк; конфигурация сохранена
в `iimt_ros2_moveit_config/config/joint_names.yaml`. Он читается при создании,
не связан с динамическим обновлением. Action имеет то же имя FJT, что новый driver,
поэтому совместный запуск конфликтует. Публикации:
`/iimt_robot/moveit_controller` (JointTrajectoryPoint), `/iimt_robot/moveit_driver`
(String); подписки `/iimt_robot/fjt_feedback` (FJT Feedback),
`/iimt_robot/moveit_execution` (String). Новый driver эти legacy topics не обслуживает.
`iimt_ros2_msgs` не запускает узлы и не имеет runtime node parameters.
