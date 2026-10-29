# Архитектура и временная модель

## Два пути исполнения

**Текущий стек — прямой SDK-драйвер, не ros2_control.** Он сам реализует action
`control_msgs/action/FollowJointTrajectory`. Наличие слова controller в имени action
не означает, что запущен `joint_trajectory_controller`.

```text
Одна рука:
  CSV notebook ────────────────┐
  RViz → move_group → Execute ├→ /iimt_group_controller/follow_joint_trajectory
                              └→ iimt_ros2_driver → API_Servoj → робот

Две руки:
  CSV notebook / MoveIt
      → /both_arms_controller/follow_joint_trajectory
      → iimt_dual_coordinator
          ├→ /left/iimt_group_controller/follow_joint_trajectory → левый драйвер
          └→ /right/iimt_group_controller/follow_joint_trajectory → правый драйвер

Обратная связь:
  робот → TCP JointReader или API_GetRobotJointRad → поток чтения
        → mutex только для snapshot → ROS executor → joint_states
        → notebook / robot_state_publisher / MoveIt
```

CSV отправляется одной action-целью. **SDK ServoJ получает по одной текущей
шестисуставной команде**, а не весь CSV и не всю MoveIt-траекторию одним вызовом.
Новый драйвер не использует старую очередь topic-моста. Отправка SDK сама по себе
не подтверждает физическое достижение углов; измерения приходят отдельным каналом.

### Интерполяция и расписание

1. Драйвер проверяет состав суставов и переставляет входные данные по именам в SDK-порядок.
2. В каждой точке обязательны шесть `positions`; `velocities` и `accelerations`
   либо пусты, либо содержат шесть чисел. Ускорения без скоростей и `effort` не поддерживаются.
3. Время `time_from_start` должно возрастать. Если первый узел позже нуля, драйвер
   вставляет в t=0 измеренную начальную позу; если узел уже в t=0, проверяет её совпадение.
4. Между узлами: positions-only → линейная интерполяция; positions+velocities →
   кубическая; positions+velocities+accelerations → квинтическая.
5. Таймер вычисляет точку по прошедшему времени относительно общего начала,
   а не по счётчику отправленных узлов. Пропущенные такты не воспроизводятся пачкой;
   опоздание не должно превращать четырёхсекундный путь в восьмисекундный.
6. Проверяются ограничения положения, скорости, ускорения, шага, свежесть состояния
   и допуски слежения. Успех требует выполнения критерия конечной позиции.

Параметры `servo.a`, `servo.v`, `servo.period`, `servo.lookahead_time`, `servo.gain`
передаются SDK отдельно от вычисленных шести углов. Это не передача всех производных
каждой точки роботу: производные MoveIt используются для интерполяции на стороне драйвера.

**Best effort, не hard realtime.** Командный таймер, ОС, TCP и firmware имеют задержки.
Мягкие timing-overrun приводят к WARN и счётчикам, но ограничения состояния/лимитов
и ошибки SDK остаются причинами отказа. `max_lateness` не отключает остальные проверки.
Блокирующие SDK-вызовы ServoJ/Login/Stop не имеют гарантированного прерывания по timeout.

## Чтение и timestamps

`feedback_transport=tcp` использует прямой ограниченный дедлайном запрос dashboard;
`read_timeout` ограничивает всю транзакцию. `sdk` вызывает блокирующий
`API_GetRobotJointRad`; этот параметр **не прерывает** SDK-чтение.

Поток чтения не держит mutex во время сетевого ожидания. Он публикует короткий
snapshot; основной executor копирует его и обслуживает ROS. При timeout последнее
измерение сохраняется, но не превращается в новое: возраст состояния продолжает
расти. `state_timeout` — отдельная строгая проверка свежести.

`JointState.header.stamp` связан с завершением чтения на хосте, а не с аппаратным
временем выборки энкодера. Поэтому сравнение по абсолютным timestamps честно отражает
доступную обратную связь, но не доказывает отсутствие внутренней задержки измерения.
`read_rate=125` — желаемая частота попыток, не гарантия 125 независимых свежих ответов/с.

## Номинальное время и упреждение

- `trajectory.header.stamp` — абсолютное номинальное начало; `time_from_start` —
  относительное время каждого узла.
- Notebook назначает CSV `header.stamp = now + START_DELAY_S` после формирования
  сообщения. Значение 3 s отсчитывается от отправителя, **не от получения сервером**.
- `trajectory_advance_ms` сдвигает выдачу команд раньше номинального расписания.
  Номинальная шкала feedback/графиков не сдвигается ради улучшения совпадения.
- При будущем stamp упреждение требует достаточного запаса до старта. Для zero-stamp
  одиночный драйвер фиксирует номинальное начало при первой отправке с учётом advance.
- `~/trajectory_timing` хранит `goal_uuid`, `receipt_stamp_ns`,
  `nominal_start_stamp_ns`, `first_command_stamp_ns`, `nominal_start_final`,
  `trajectory_advance_ms`. QoS: reliable, transient-local, depth 1.

**330 ms — результат настройки конкретного опыта, не свойство каждого CI-05.**
Это значение нельзя вывести только из частоты CSV 30 Hz или периода ServoJ 8 ms.
На наблюдаемое отставание влияют firmware/ServoJ, transport и возраст измерения.
Стек по умолчанию имеет advance=0. Измеряйте отдельно обе руки на текущей сети,
частоте, прошивке и ServoJ-настройках; один постоянный сдвиг не исправляет случайный jitter.
Проект не заявляет, что происхождение каждой составляющей 330 ms доказано.

## Двурукий координатор

Координатор принимает все 12 суставов или все шесть суставов одной руки. Для
однорукой цели вторую руку удерживает в свежей неподвижной измеренной позе.
Произвольные неполные наборы отвергаются.

Для ненулевого stamp сохраняется **одно и то же** время обеих дочерних целей.
Для zero-stamp (обычный MoveIt-запрос) выбирается `now + dispatch_lead_time_s`.
Запас должен покрывать наибольшее упреждение и `dispatch_margin_s`.
Проверяются здоровье обоих драйверов и завершение приёма до раннего pre-roll.

При отказе ребёнка отменяется другой; отмена родителя отменяет обоих. Следующая
цель не принимается до завершения владения дочерними целями. Неподтверждённая
остановка/владение фиксирует fault; перезапуск допустим лишь после проверки
фактической остановки. Это не атомарный распределённый старт и не аппаратный E-stop.
Потеря процесса/сети может помешать доставке cancel.

## ROS-интерфейсы текущего стека

| Узел | Публикует | Принимает / вызывает |
|---|---|---|
| `/iimt_ros2_driver` | `/joint_states`, `/iimt_ros2_driver/diagnostics`, `/iimt_ros2_driver/trajectory_timing` | action `/iimt_group_controller/follow_joint_trajectory`; стандартные parameter services |
| `/left/iimt_ros2_driver`, `/right/iimt_ros2_driver` | `/{arm}/joint_states`, `/{arm}/iimt_ros2_driver/{diagnostics,trajectory_timing}` | action `/{arm}/iimt_group_controller/follow_joint_trajectory` |
| `/iimt_dual_coordinator` | глобальный `/joint_states` | action `/both_arms_controller/follow_joint_trajectory`; подписки на оба arm state/diagnostics; дочерние actions |
| `/move_group` | `/monitored_planning_scene` и MoveIt-интерфейсы | `/move_action`, `/execute_trajectory`; `/check_state_validity`, `/get_planning_scene`, `/apply_planning_scene` |
| `/robot_state_publisher` | TF дерева URDF | `/joint_states`, `robot_description` |
| `/iimt_workspace` | не поток управления | читает `/get_planning_scene`, применяет объекты через `/apply_planning_scene` |
| Jupyter ROS-клиент | action-запросы | state/timing, параметры драйверов; в dual также MoveIt services/actions |

Глобальный `/joint_states` содержит последовательность **двух шестисуставных
сообщений с исходными timestamps**, а не искусственно объединённое одновременное
12-суставное измерение. Координатор не синтезирует aggregate action feedback:
измерения пишутся с arm-топиков. В dual-режиме отправляйте цели только координатору;
ROS технически не запрещает другому клиенту обойти его и вызвать child action.

## Планирование и сцена

Dual использует OMPL `geometric::RRTConnect`; группа `both_arms` планирует 12 суставов
совместно. `left_arm` и `right_arm` имеют отдельные KDL IK; для составной группы
не назначается цепочечный IK. Межруковые пары не исключены из collision checking;
в SRDF сохранены отдельные однорукие исключения.

Скоростной предел планирования для каждого сустава:
`planning_limit_scale × min(limits.max_velocity, limits.max_step × command_rate)`.
Ускорение: `planning_limit_scale × limits.max_acceleration`. Драйверные пределы
при этом не ослабляются. Масштабы скорости/ускорения запроса notebook добавляют
дальнейшее замедление MoveIt-подвода, но не меняют CSV.

Границы workspace ориентированы по **world X/Y**, не по повёрнутым базам. Четыре
наружных сплошных box-области закрывают выход за прямоугольник баз + отступы.
Толщина 10 m и вертикальный диапазон далеко превосходят досягаемость текущей модели;
это не математически бесконечные полупространства. Потолок не добавляется.

Пол — бесконечная поверхность Plane `[0,0,1,-floor_z]`, не заполненный объём ниже
пола. По умолчанию он проходит через нижнюю базу. Разрешены только контакты пола
с двумя неподвижными base_link; движущиеся звенья и attached tools проверяются.
Узел сначала читает и объединяет ACM; не редактируйте её конкурентно при запуске.

`iimt_workspace` применяет сцену при запуске и не восстанавливает удалённые объекты
постоянно. Notebook требует четыре стены и пол и проверяет неизменность сцены перед
исполнением. Модель инструмента и препятствий должна соответствовать стенду.

**Прямой CSV не планируется заново MoveIt.** Dual notebook дискретно проверяет
его линейный путь через state validity; проверочные состояния не добавляются в
отправляемую траекторию. Между образцами и после изменения окружения возможны
неучтённые коллизии. Однорукий CSV notebook вообще не делает MoveIt collision preflight.

## Legacy, сохранённый для совместимости

`iimt_ros2_control` — старый SystemInterface с отдельной аппаратной нодой,
ros2_control/controller_manager и joint_trajectory_controller.
`iimt_ros2_trajectory_action` — старый мост FollowJointTrajectory в
`/iimt_robot/moveit_controller` (`JointTrajectoryPoint`) и `/iimt_robot/moveit_driver`.
`iimt_ros2_msgs` содержит legacy RobotMode/SafetyMode/ProgramState, SetMode action и
сервисы dashboard, а не новые управляющие ноды.

Старый `iimt_moveit.launch.py` и `config/ros2_controllers.yaml` относятся к первому
пути. Текущий single launch называется **`iimt_driver_moveit.launch.py`**.
Не запускайте legacy и direct драйверы к одному роботу, даже в разных ROS domains:
DDS-изоляция не изолирует SDK-соединения и физическое управление.

### Источники

- `ros2_ws/src/iimt_ros2_driver/src/iimt_ros2_driver.cpp`
- `ros2_ws/src/iimt_ros2_driver/include/{trajectory.hpp,joint_reader.hpp}`
- `ros2_ws/src/iimt_dual_control/iimt_dual_control/{coordinator.py,trajectory.py,bringup.py}`
- `ros2_ws/src/iimt_dual_moveit_config/{launch/dual_moveit.launch.py,scripts/workspace_scene.py}`
