# Диагностика без обхода защит

Сначала сохраняйте **первую** ошибку driver-ноды, diagnostics обеих рук и полный
текст action result. Последнее `MoveGroupInterface::execute() failed` обычно
сообщает следствие, а не причину. Не повышайте допуски и не отправляйте новые цели
при неизвестном состоянии остановки.

## Команды наблюдения

Команды ниже выполняются в ROS-shell контейнера (`make shell`). Они только читают
состояние; требуют запущенного соответствующего стека. Имена сверены с исходниками;
наличие живой аппаратуры не является условием проверки syntax/API.

```bash
ros2 node list
ros2 action list -t
ros2 topic list
ros2 param dump /iimt_ros2_driver
ros2 topic echo /iimt_ros2_driver/diagnostics --once
```

Для dual вместо последней пары:

```bash
ros2 param dump /left/iimt_ros2_driver
ros2 param dump /right/iimt_ros2_driver
ros2 topic echo /left/iimt_ros2_driver/diagnostics --once
ros2 topic echo /right/iimt_ros2_driver/diagnostics --once
```

Для времени запуска после недавней цели:

```bash
ros2 topic echo /left/iimt_ros2_driver/trajectory_timing --once --qos-durability transient_local --qos-reliability reliable
ros2 topic echo /right/iimt_ros2_driver/trajectory_timing --once --qos-durability transient_local --qos-reliability reliable
```

Если цели ещё не было, timing-топик не обязан содержать запись. `--once` может
ожидать сообщение бесконечно, пока его не опубликуют.

## Типовые случаи

| Симптом | Что означает | Что проверить |
|---|---|---|
| `No /execute_trajectory` / нет action server | MoveIt не запущен, не виден DDS или упал | `node list`, `action list -t`, launch logs, domain/RMW, одинаковое ROS-окружение |
| `/execute_trajectory` виден, но сервер не отвечает | Имя action мог объявить только клиент | Наличие `/move_group`; discovery имени само по себе не доказывает action server |
| `Didn't receive robot state ... recent timestamp` | Нет свежих joint_states для валидации | driver read errors, feedback_age, имена суставов и часы; не подставлять новый timestamp старым углам |
| `Joint read timed out` | TCP-транзакция не завершилась в read_timeout | Ethernet, нагрузка dashboard, другие клиенты, реальные read durations; timeout ≠ гарантированно обрыв сети |
| `API_GetRobotJointRad failed`, `Expected Joint radian` | Сбой SDK/ответа dashboard | Transport, совместимость firmware, доступность IP/порта, единственный владелец; последний sample не свежий |
| `Expected all six joints ...` | Старый build или неверный набор/имена | Текущий driver умеет reorder по именам; нужны полный набор6, без дублей; dual принимает6 одной руки или12 |
| `Six positions ... derivatives ...` | Неверная форма JointTrajectoryPoint | Positions6, derivatives пусты/6; acceleration только с velocity; effort пуст |
| `Start position mismatch` | Начало траектории не совпадает с измерением | Сделать отдельный проверенный подвод, не ослаблять start tolerance |
| `Missed endpoint deadline` | Командный цикл пропустил предусмотренную возможность корректно выдать конец | Timing counters, SDK blocking, ОС/сеть; просто увеличить path tolerance недостаточно |
| `PATH_TOLERANCE_VIOLATED` | Общий код отказа driver/координатора | Читать `error_string`, он различает tracking, stale feedback, Stop/Clear и другие причины |
| План не найден / collision at start | Начальная/целевая геометрия недопустима | Базы, workspace, пол, attachments, actual start; RRTConnect не выводит из коллизии автоматически |
| `parameters changed` / scene changed в notebook | Предпросмотр больше не соответствует исполнению | Повторно подготовить план/CSV, не удалять проверку |
| `read-only parameter` | Изменение on-the-fly запрещено | YAML/launch до запуска, затем штатный перезапуск после остановки |
| `ros2 control` отсутствует | CLI ros2_control не установлен или не нужен | Direct-стек им не управляется; используйте actions/driver diagnostics |
| `recognize_objects not available` RViz | Нет optional object recognition action | Само по себе не причина отказа joint trajectory |
| KDL предупреждает inertia у root link | Ограничение KDL-представления root inertia | Отдельно от SDK read/dispatch; не объясняет неподтверждённую остановку |

### `SDK Stop/Clear failed; physical stop is NOT confirmed`

Это **не утверждение, что ошибка положения превысила 3 rad**. `PATH_TOLERANCE_VIOLATED`
в этом driver может сопровождать Stop/Clear-fault. Одна из операций SDK остановки/
очистки не подтвердилась; программный код не может достоверно подтвердить остановку.

1. Убедитесь в фактической остановке независимо от ROS. При необходимости используйте
   предусмотренную аппаратную остановку стенда.
2. Не повторяйте цель и не перезапускайте драйвер как способ «сбросить сообщение»
   до проверки физического состояния и причины отказа.
3. Сохраните исходную ошибку до Stop/Clear, diagnostics обеих рук, timestamps и
   частичные графики notebook. Отказ остановки может быть вторичным.
4. После проверки восстановите соединение/состояние контроллера и перезапустите
   стек, если coordinator зафиксировал fault.

### Частота и latency

Из diagnostics полезны `read_calls`, `read_timeouts`, `last_read_duration_s`,
`max_read_duration_s`, `feedback_generation`, `feedback_age_s`, `sent`,
`last_servoj_duration_s`, `max_servoj_duration_s`, `last_send_interval_s`,
`missed_timer_slots`, `timer_dispatch_overruns`, `trajectory_start_overruns`,
`servoj_call_overruns`, `sdk_cycle_overruns`, `max_timing_overrun_s`.

Счётчики накопительные: WARN может оставаться после единичного прошлого события.
Смотрите прирост и свежесть, а не только level. Увеличение `read_timeout` даёт
запросу больше времени, но не ускоряет robot dashboard. Оно ограничено state_timeout;
увеличение последнего означает, что вы допускаете более старое состояние.

Read вынесен в поток, поэтому один медленный ответ не обязан растягивать командный
период. Однако сеть/firmware/SDK остаются общими ресурсами, а отдельный thread не
создаёт аппаратной гарантии realtime. Понижение read_rate может уменьшить нагрузку;
проверяйте результат измерением, не обещайте линейное ускорение.

### Jupyter не импортирует rclpy

ROS Humble в образе использует системный Python3.10. Выберите ядро **IIMT ROS2 Humble**
(`iimt-ros2`), а не случайный conda/kernel другого Python. Entrypoint должен source
ROS и overlay до старта Jupyter. Venv создаётся с `--system-site-packages`:
не устанавливайте одноимённый `rclpy` из PyPI и не копируйте ROS Python-каталоги вручную.

Ошибка `cannot import name import_c_library from rpyutils` обычно требует проверки
окружения/затенения модуля; переустановка notebook-графиков её не исправляет.
Смотрите `sys.executable`, `rclpy.__file__`, `rpyutils.__file__` в правильном kernel.

### Docker, DDS и RViz

- Поддерживаемый контейнер — Linux `amd64`, ROS Humble/Ubuntu22.04, Python3.10.
  SDK содержит x86-64 ELF; ARM-host требует эмуляции и не является проверенной
  платформой для управления по времени.
- Compose использует host network. Это не изолирует ROS traffic от хоста;
  настройте ROS_DOMAIN_ID/RMW одинаково у нужных участников.
- Jupyter слушает127.0.0.1 и требует token. При удалённой работе используйте
  защищённый туннель; не отключайте auth для удобства.
- RViz по умолчанию выключен. `compose.gui.yaml` требует действительного X11
  DISPLAY/XAUTHORITY. Не исправляйте GUI через `xhost +` или privileged-container.
- Mount всего проекта поверх `/opt/iimt` скроет установленный overlay. Штатный
  Compose монтирует только notebooks/data/results/config.
- Shared flock не позволяет двум штатным single/dual launch этого Compose-проекта
  работать одновременно, но не защищает от ROS launch снаружи, notebooks SDK или
  legacy-клиентов. Один процесс-владелец на каждый физический манипулятор.
- Файлы контейнера могут принадлежать root на хосте; это ожидаемо для текущего
  образа. Учитывайте права каталога результатов, не применяйте рекурсивный chmod777.

### Графики кажутся сдвинутыми

Сначала сравните `nominal_start_stamp_ns`, header stamp измерений и плановое время.
Не сдвигайте actual-кривую по кросс-корреляции перед вычислением основной ошибки.
Отдельная оценка lag полезна, но это другой показатель. Advance на одной руке
не должен автоматически копироваться на вторую. Проверьте, что CSV TIME_SCALE и
частоты драйвера соответствуют опыту, а ранний pre-roll помещается в start delay.

## Границы проверки

Offline tests, синтаксис notebook и mock launch подтверждают программные интерфейсы,
но не TCP latency физического контроллера, аварийную остановку, калибровку баз,
collision model или точность реального слежения. Для этих утверждений нужен отдельный
commissioning с независимым средством остановки и документированными измерениями.


## Необязательный legacy Mongo warehouse

`warehouse_ros_mongo` отсутствует в использованном Humble apt-репозитории. Эта старая
зависимость удалена из обязательного `package.xml`: текущие direct-driver single/dual
launch её не используют. Docker собирает все девять пакетов, но не включает Mongo backend.
Сохранённый legacy `iimt_moveit.launch.py db:=true` требует отдельной установки совместимого
`warehouse_ros_mongo` из исходников и здесь не проверяется. Оставляйте `db:=false`;
это не отключает планирование, сцену, контроллеры или экспорт измерений ноутбуков.
