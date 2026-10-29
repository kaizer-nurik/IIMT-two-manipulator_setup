# IIMT Manipulation · ROS 2 Humble

Отдельный проект для управления **одним или двумя IIMT-CI-05** через прямой SDK
ServoJ-драйвер, планирования MoveIt2 и измерения исполнения CSV-траекторий в Jupyter.
Включает исходники ROS-пакетов, две актуальные notebook-программы, пример CSV,
Docker, Docker Compose и Makefile.

**Штатный запуск: mock=true, движение выключено, RViz выключен, advance=0.**
Ни сборка, ни тесты не требуют подключённого манипулятора. Для real-режима нужны
явно проверенные IP, крепление баз, модель инструмента, свободный путь и независимая
аппаратная остановка. Этот стек не является safety PLC и не гарантирует hard realtime.

## Навигация

- [Быстрый старт](#быстрый-старт-в-docker)
- [Настройка .env и значения по умолчанию](#конфигурация-env)
- [RViz, X11 и xhost](#rviz-x11-и-xhost)
- [Состав пакетов](#пакеты-и-назначение)
- [Запуск без Compose](#прямой-ros2-launch)
- [Архитектура, ROS-интерфейсы и время](docs/architecture.md)
- [Полный справочник собственных параметров](docs/parameters.md)
- [Работа с notebook и графиками](docs/notebooks.md)
- [Диагностика и типовые ошибки](docs/troubleshooting.md)
- [Перенос из прежнего workspace](docs/migration.md)
- [Результаты проверки поставки](docs/verification.md)

## Что делает проект

- Принимает стандартную `FollowJointTrajectory` без старого FIFO topic-моста.
- Интерполирует задание по времени и вызывает ServoJ по одному набору шести углов.
- Читает состояния независимым потоком: bounded TCP reader или исходный SDK reader.
- Разделяет nominal timestamp, раннюю выдачу команд и timestamps измерений.
- Координирует две руки с общим абсолютным временем начала и отменой второй при
  отказе первой; это синхронизированное расписание, не аппаратный триггер.
- Планирует совместный RRTConnect-подвод с учётом другой руки, workspace и пола.
- Исполняет исходные CSV positions/time без добавленного разгона и выхода.
- Строит заданную/измеренную траекторию, ошибки и RMSE/max; при отказах сохраняет
  доступные частичные данные, не заменяя их фиктивными нулями.

**Что проект не обещает:** гарантированное время доставки TCP/SDK, атомарный
распределённый старт при сетевом сбое, аппаратную остановку через cancel,
непрерывную защиту от коллизий при прямом CSV или универсальную компенсацию330 ms.

## Структура

```text
iimt_manipulation/
├── README.md
├── Dockerfile
├── compose.yaml                 # single / dual / notebook / tools
├── compose.gui.yaml             # optional X11 для RViz
├── Makefile
├── .env.example                 # безопасный образец настройки
├── requirements-notebook.txt
├── docker/                      # ROS entrypoint, launch wrapper, Jupyter
├── scripts/                     # зависимости, notebook kernel, mock graph и выполнение
├── config/                      # пользовательские YAML (read-only mount)
├── docs/                        # архитектура, параметры, notebook, диагностика, перенос
├── ros2_ws/src/                 # 9 ROS-пакетов, см. таблицу ниже
├── notebooks/
│   ├── iimt_ci05_left_csv_ethernet.ipynb
│   └── iimt_ci05_dual_csv_ethernet.ipynb
├── data/trajectory_both_TimeGrid_30Hz.csv
└── results/                     # новые пользовательские измерения
```

Архивные результаты, все старые notebooks и PDF производителя не копируются в
runtime-проект. Достаточные для сборки SDK headers и бинарная библиотека включены
в driver-пакет. Миграция и compatibility-ссылки описаны отдельно; перенос не означает
разрешения распространять proprietary SDK.

## Требования

- Linux x86-64; Docker Engine и Compose plugin **≥2.24**; GNU Make.
- Доступ к ROS/Ubuntu/PyPI репозиториям при первой сборке.
- Для реальных роботов — сетевой маршрут Ethernet к обоим заданным IP.
- Для RViz — X11/Xwayland display и разрешение доступа (Xauthority или ограниченное
  правило `xhost`); headless-режим работает без GUI.

База образа — `osrf/ros:humble-desktop`, Ubuntu22.04 / Python3.10.
SDK `libiimtcl.so` — x86-64 ELF, поэтому образ намеренно `linux/amd64`.
ARM-эмуляция не проверена для управления по времени.
Официальная документация Humble указывает Ubuntu Jammy22.04 для deb-пакетов:
[исходник ROS installation guide](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst).
Требование Compose2.24 связано с optional `env_file.required`:
[Docker environment variables](https://docs.docker.com/compose/how-tos/environment-variables/set-environment-variables/).

## Быстрый старт в Docker

Команды выполняются **на Docker-хосте из каталога этого проекта**, не внутри старого ROS-контейнера.
Bind mounts разрешаются на стороне Docker daemon: при работе через чужой Docker socket
путь проекта должен существовать именно на хосте daemon. Создание `.env` — первый запуск;
не перезаписывайте уже настроенный файл без сравнения.

```bash
cp .env.example .env
make build
make test
make smoke
make up-single
make notebook
make logs SERVICE=notebook
```

Откройте `http://127.0.0.1:8888`; token показан в Jupyter logs. Выберите kernel
**IIMT ROS 2 Humble (Python 3.10)**. При пустом `JUPYTER_TOKEN` генерируется token,
а не отключается аутентификация. Notebook и результаты сохраняются на хосте.

С исходной `.env.example` команда `make up-single` стартует mock, **но не разрешает
движение**. Уже изменённый `.env` сохраняет ваши настройки, а не возвращает безопасные
значения при каждом запуске. Если вы намерены
пройти notebook в имитации, явно разрешите mock actions:

```bash
make down
make up-single LAUNCH_ARGS='use_mock_hardware:=true enable_motion:=true'
make notebook
```

Для dual вместо single:

```bash
make down
make up-dual LAUNCH_ARGS='use_mock_hardware:=true enable_motion:=true'
make notebook
```

У dual исходная нулевая mock-поза может быть недопустима по workspace. Наличие
живого ROS graph не означает допустимости initial pose. Notebook не обходит эту
проверку автоматически — см. [порядок работы](docs/notebooks.md).

### Makefile

| Команда | Действие |
|---|---|
| `make help` | Краткая справка |
| `make build` | Собрать общий образ: зависимости + все ROS-пакеты + Python/kernel |
| `make up-single` | Запустить текущий одиночный MoveIt/direct-driver стек |
| `make up-dual` | Запустить dual MoveIt, два driver и coordinator/workspace |
| `make notebook` | Запустить Jupyter, без автоматического запуска движений |
| `make shell` | ROS/Python shell в tools-контейнере |
| `make test` | Offline colcon tests и проверка файлов/notebook, без роботов |
| `make smoke` | Изолированный mock ROS graph smoke, без motion goals |
| `make deps` | rosdep/pip, ROS/Python импорты, SDK ldd и kernelspec |
| `make notebook-smoke` | CSV-ячейки обоих notebook и два inline-графика в реальном kernel |
| `make smoke-execution` | Выполнение single/dual mock и RRTConnect, без физического SDK |
| `make verify` | Все перечисленные проверки последовательно |
| `make show-args MODE=single` | Показать доступные launch arguments |
| `make show-args MODE=dual` | То же для двух рук |
| `make logs SERVICE=dual` | Логи выбранного сервиса |
| `make config` | Render Compose configuration |
| `make down` | Завершить контейнеры этого Compose-проекта |

`make down`/SIGINT не являются аварийной остановкой робота: сначала завершите
или штатно отмените движение и подтвердите остановку. Общий flock не разрешает
случайный одновременный запуск single и dual **этого Compose-проекта**, но не блокирует
чужой launch, vendor SDK notebook или другую управляющую программу.

## Конфигурация контейнеров

### Конфигурация .env

Файл `.env` лежит **рядом с `compose.yaml` и Makefile**. Первый раз скопируйте
`.env.example`; существующий файл сначала сравните с образцом, не перезаписывайте.
Ниже — полный исходный профиль из `.env.example`, **не текущие пользовательские
настройки**. Это рекомендуемая отправная точка для проверки установки, а не готовая
калибровка реального стенда:

```dotenv
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
USE_MOCK_HARDWARE=true
ENABLE_MOTION=false
LAUNCH_RVIZ=false

ROBOT_IP=192.168.50.95
LEFT_ROBOT_IP=192.168.50.92
RIGHT_ROBOT_IP=192.168.50.95

COMMAND_RATE=125.0
READ_RATE=125.0
READ_TIMEOUT=0.1
FEEDBACK_TRANSPORT=tcp
TRAJECTORY_ADVANCE_MS=0.0

LEFT_COMMAND_RATE=125.0
RIGHT_COMMAND_RATE=125.0
LEFT_READ_RATE=125.0
RIGHT_READ_RATE=125.0
LEFT_READ_TIMEOUT=0.1
RIGHT_READ_TIMEOUT=0.1
LEFT_FEEDBACK_TRANSPORT=tcp
RIGHT_FEEDBACK_TRANSPORT=tcp
LEFT_TRAJECTORY_ADVANCE_MS=0.0
RIGHT_TRAJECTORY_ADVANCE_MS=0.0

LEFT_BASE_XYZ="-0.0008297 -0.5380000 0.0818739"
LEFT_BASE_RPY="0 0 -0.6981317008"
RIGHT_BASE_XYZ="-0.345440489323 0.411679944176 -0.082000000000"
RIGHT_BASE_RPY="0 0 0"
WORKSPACE_MARGIN=0.5
FLOOR_Z=auto

LAUNCH_ARGS=
JUPYTER_PORT=8888
JUPYTER_TOKEN=
```

Пишите логические значения как `true` / `false`, дробные числа — через точку.
Единицы: частоты **Hz**, timeout **s**, упреждение **ms**, координаты **m**, углы
**rad**. Векторы — три числа через пробел в кавычках, не список с запятыми.
`.env` читает Compose: выполнять `source .env` или добавлять `export` к строкам не нужно.

#### Общие настройки ROS и режима

| Поле | Исходное значение | Что означает и почему начинать с него |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | DDS domain для driver, MoveIt, RViz и notebook. Одинаковое значение нужно всем участникам одного графа. `0` удобно для первого локального запуска; на общем стенде выберите согласованный свободный domain, чтобы не смешивать независимые ROS-графы. Это не механизм защиты доступа. |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | Реализация ROS middleware; CycloneDDS установлен в образе. Одинаковая настройка во всех контейнерах упрощает диагностику и воспроизводимость. Не меняет TCP-протокол связи с самим роботом. |
| `USE_MOCK_HARDWARE` | `true` | Имитация без подключения к физическому роботу. Оставляйте `true` для проверки установки, GUI и notebook. `false` включает реальные соединения: даже при `ENABLE_MOTION=false` драйвер уже может обращаться к оборудованию. |
| `ENABLE_MOTION` | `false` | Запрещает выполнение траекторий до явного разрешения. Для движения в mock поставьте `true`, сохранив `USE_MOCK_HARDWARE=true`. На железе включайте только после проверки стенда; этот параметр не включает питание и не снимает тормоза. |
| `LAUNCH_RVIZ` | `false` | Не запускает GUI в базовом headless-контейнере. Для окна нужны **одновременно** `true`, `compose.gui.yaml`, рабочий display и авторизация X11. Одно `true` не пробрасывает экран внутрь контейнера. |

#### IP-адреса

| Поле | Исходное значение | Режим и выбор значения |
|---|---|---|
| `ROBOT_IP` | `192.168.50.95` | Только `single`: адрес единственного робота. Для mock можно оставить пример; перед real запуском укажите фактический адрес Ethernet-интерфейса робота, не компьютера. |
| `LEFT_ROBOT_IP` | `192.168.50.92` | Только `dual`: адрес физической руки, которая должна соответствовать `left_` в модели и CSV. Это пример старого стенда, а не автоматически определённая левая рука. |
| `RIGHT_ROBOT_IP` | `192.168.50.95` | Только `dual`: адрес физической `right_` руки. На реальном стенде проверьте соответствие обеих рук; адреса двух роботов должны различаться. |

`single` не использует `LEFT_ROBOT_IP` / `RIGHT_ROBOT_IP`; `dual` не использует
`ROBOT_IP`. Доступность IP сама по себе не доказывает правильность назначения руки.

#### Частоты, чтение и упреждение: single

| Поле | Исходное значение | Назначение и обоснование |
|---|---|---|
| `COMMAND_RATE` | `125.0` Hz | Целевая частота выдачи ServoJ, допустимо `1…125`. Период при 125 Hz — 8 ms; это профиль для временной дискретизации заданной траектории, не обещание hard realtime. Для эксперимента на 30 Hz задайте `30.0`; длительность траектории по `time_from_start` от этого не должна умножаться. |
| `READ_RATE` | `125.0` Hz | Целевая частота независимого reader, допустимо `1…125`. Начальный профиль совпадает с командным, но фактические успешные измерения ограничены сетью/контроллером. Можно уменьшать отдельно от `COMMAND_RATE`; 125 Hz не означает 125 свежих измерений в секунду. |
| `READ_TIMEOUT` | `0.1` s | Deadline одной полной TCP-транзакции чтения, не командного цикла. 100 ms — исходный компромисс между редкими задержками ответа и ограничением зависания; не нужно автоматически делать его равным 8 ms. Должен быть положительным и не больше `state_timeout` из YAML (в штатном профиле 0.2 s). Timeout не превращает старый ответ в новое измерение. |
| `FEEDBACK_TRANSPORT` | `tcp` | Рекомендуемый reader с управляемым socket deadline. Альтернатива `sdk` использует блокирующий `API_GetRobotJointRad`: `READ_TIMEOUT` не умеет прерывать этот SDK-вызов. `sdk` оставлен для сравнения/диагностики, не для обещания bounded latency. |
| `TRAJECTORY_ADVANCE_MS` | `0.0` ms | Ранняя отправка команд относительно номинального времени, допустимо `0…1000`. Начинайте с нуля, чтобы сначала измерить задержку. Значения 320/330 ms — результаты конкретных опытов, не универсальный default; изменение прошивки, gain, сети или частоты требует повторной проверки. |

Важно: **125 Hz — значение именно `.env.example`**, не универсальное значение
пакета. Single YAML без переопределения имеет 30 Hz; dual-профиль — 125 Hz.
Не удаляйте поле и не оставляйте его пустым в расчёте на сохранение последнего
значения: wrapper пропускает пустые значения и срабатывают launch/YAML defaults.
Упреждение не сдвигает ось времени графиков и не заменяет 3-секундную задержку
старта из `header.stamp` сообщения notebook.

#### Частоты, чтение и упреждение: dual

Поля без `LEFT_` / `RIGHT_` выше **не управляют dual**. Здесь настройки каждой руки
независимы; одинаковые начальные значения упрощают сравнение и диагностику:

| Поле | Исходное значение | Назначение и почему |
|---|---|---|
| `LEFT_COMMAND_RATE` | `125.0` Hz | ServoJ левой руки, 8 ms; начальный общий профиль, диапазон `1…125`. |
| `RIGHT_COMMAND_RATE` | `125.0` Hz | ServoJ правой руки; начните с той же частоты, что у левой, если нет измеренной причины менять. |
| `LEFT_READ_RATE` | `125.0` Hz | Целевая частота reader левой руки; реальная частота зависит от длительности её ответов. |
| `RIGHT_READ_RATE` | `125.0` Hz | Reader правой руки; можно уменьшить отдельно, если её канал отвечает медленнее. |
| `LEFT_READ_TIMEOUT` | `0.1` s | TCP deadline левой руки; согласуйте с её `state_timeout` в YAML. Увеличение timeout не ускоряет чтение. |
| `RIGHT_READ_TIMEOUT` | `0.1` s | TCP deadline правой руки; исходно такой же для сопоставимых измерений. |
| `LEFT_FEEDBACK_TRANSPORT` | `tcp` | Bounded TCP reader левой руки; `sdk` только с пониманием блокирующего поведения. |
| `RIGHT_FEEDBACK_TRANSPORT` | `tcp` | То же для правой; исходно одинаковый транспорт исключает лишнее различие каналов. |
| `LEFT_TRAJECTORY_ADVANCE_MS` | `0.0` ms | Компенсация измеренной задержки левой руки, не поправка «по умолчанию». |
| `RIGHT_TRAJECTORY_ADVANCE_MS` | `0.0` ms | Компенсация правой руки; измеряйте отдельно, не копируйте значение левой автоматически. |

Общий timestamp старта не означает одновременного аппаратного измерения энкодеров.
Разное упреждение влияет на взаимное положение рук во времени; после его изменения
нужно заново проверить совместное движение, а не только уменьшение ошибки одной руки.

#### Геометрия, workspace и пол: только dual

| Поле | Исходное значение | Смысл и выбор |
|---|---|---|
| `LEFT_BASE_XYZ` | `"-0.0008297 -0.5380000 0.0818739"` | Положение левой базы в `world`, метры. Это перенесённая модель стенда; оставляйте для её mock-проверки, на железе замените результатом измерения монтажа. |
| `LEFT_BASE_RPY` | `"0 0 -0.6981317008"` | Ориентация левой базы относительно `world`: roll, pitch, yaw в радианах; текущий yaw равен примерно −40°. Не подставляйте градусы. |
| `RIGHT_BASE_XYZ` | `"-0.345440489323 0.411679944176 -0.082000000000"` | Положение правой базы **в системе левой**, не `world`. Для world-позы применяется композиция преобразований; просто складывать XYZ при повёрнутой левой базе нельзя. |
| `RIGHT_BASE_RPY` | `"0 0 0"` | Ориентация правой базы **относительно левой**. Нули означают одинаковую ориентацию баз, а не нулевой мировой yaw правой руки. |
| `WORKSPACE_MARGIN` | `0.5` m | По 50 cm наружу от охватывающего обе базы прямоугольника в осях `world` X/Y. Это исходная граница планирования, не гарантия безопасного физического объёма. Потолка нет; четыре стороны можно переопределить через `LAUNCH_ARGS`. |
| `FLOOR_Z` | `auto` | Высота горизонтальной плоскости пола в `world`; `auto` берёт меньшую мировую высоту двух баз. Подходит для исходной модели крепления. Если реальный пол другой, укажите его измеренную Z в метрах, например `0.0`. |

Эти поля wrapper не передаёт single-launch: их изменение не перемещает основание
однорукой модели и не добавляет ей dual-workspace. Настройка модели для планирования
не изменяет аппаратную калибровку робота. Ошибка баз делает collision checking
неверным, даже если в RViz всё выглядит правдоподобно.

#### Дополнительные аргументы и Jupyter

| Поле | Исходное значение | Назначение и почему |
|---|---|---|
| `LAUNCH_ARGS` | пусто | Дополнительные `name:=value` выбранного launch. Применяются **после** остальных env-полей и могут переопределить даже mock/motion. По умолчанию пусто, чтобы избежать скрытых конфликтов. Это не shell-скрипт: `&&`, команды и подстановки выполнять здесь нельзя. |
| `JUPYTER_PORT` | `8888` | Порт Jupyter на `127.0.0.1` Docker-хоста. Стандартный адрес — `http://127.0.0.1:8888`; если порт занят, задайте свободный, например `8889`. При конфликте процесс завершается, а не молча выбирает следующий порт. |
| `JUPYTER_TOKEN` | пусто | Wrapper удаляет пустую переменную, и Jupyter генерирует случайный token: пусто **не отключает авторизацию**. Это рекомендуемый default; смотрите token через `make logs SERVICE=notebook`. Для постоянного доступа задайте длинный случайный секрет; не коммитьте `.env` и не публикуйте логи/config с token. |

Jupyter слушает loopback даже при host networking. Для удалённой работы используйте
отдельно настроенный защищённый доступ, а не случайное открытие сервера без token.

#### Приоритет настроек и применение изменений

1. `compose.yaml` загружает `.env` в окружение контейнеров через `env_file`.
   Для `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `USE_MOCK_HARDWARE`, `ENABLE_MOTION`,
   `LAUNCH_RVIZ`, `LAUNCH_ARGS` есть ещё явный `environment`: при Compose-подстановке
   экспортированное значение shell имеет приоритет над `.env`.
2. Остальные поля, например `ROBOT_IP` и `COMMAND_RATE`, передаются через `env_file`
   без такого явного `environment`. Не рассчитывайте, что произвольный
   `export COMMAND_RATE=30` заменит значение из файла. Меняйте `.env` либо используйте
   поддерживаемое `make … LAUNCH_ARGS='command_rate:=30'`.
3. `docker/launch.py` выбирает single/dual-поля и затем накладывает `LAUNCH_ARGS`.
   Если аргумент повторён, последнее значение побеждает. `make … LAUNCH_ARGS='…'`
   заменяет **всю строку** `LAUNCH_ARGS` из `.env`, а не дописывает её.
4. Непереданные поля берутся из выбранного launch/YAML. Переменные вроде
   `SERVO_GAIN` или `ROBOT_TYPE` сами по себе не становятся ROS-параметрами: wrapper
   имеет явный список. Доступные аргументы смотрите через `make show-args MODE=single`
   или `MODE=dual`. Остальные параметры задавайте YAML в `config/` через
   `parameters_file`, `left_parameters_file`, `right_parameters_file`.

Это различие `env_file`, `.env`-подстановки и `environment` соответствует
[документации Docker Compose](https://docs.docker.com/compose/how-tos/environment-variables/set-environment-variables/)
и [правилам подстановки](https://docs.docker.com/compose/how-tos/environment-variables/variable-interpolation/).

Параметры нод read-only; `.env` не перечитывается работающей нодой автоматически.
После **штатного завершения/отмены движения** повторите `make up-single` или
`make up-dual` с тем же GUI-overlay, если он использовался. Compose применит
изменённое окружение при пересоздании сервиса. Для Jupyter повторите `make notebook`.
Обычный `docker compose restart` не подхватывает новые env-значения.
Пересборка образа ради одного `.env` не нужна. Не используйте смену конфигурации
или перезапуск контейнера вместо аварийной остановки.

Проверка перед запуском:

```bash
make config                 # содержит итоговое окружение; может раскрыть token
make show-args MODE=single   # объявления ROS launch, не полный dump runtime-параметров
```

Пример изменения границ без изменения robot code:

```bash
make up-dual LAUNCH_ARGS='workspace_margin:=0.5 workspace_x_max_margin:=0.6'
```

В контейнере root проекта `/opt/iimt`. Исходники и `ros2_ws/install` встроены в образ;
только `notebooks`, `data`, `results`, `config` монтируются с хоста (data/config read-only).
После изменения C++/Python-пакетов пересоберите образ. Не монтируйте весь checkout
поверх `/opt/iimt`: это скроет установленный workspace.

Compose использует Linux host network: ROS/DDS получает сетевую доступность хоста,
Jupyter всё равно слушает только loopback. Используйте одинаковый ROS_DOMAIN_ID и согласованную RMW/DDS-конфигурацию;
для воспроизводимости примеры используют один RMW. Синхронизация часов нужна
отдельно для временной корректности. Не запускайте
несколько независимых владельцев управления одним физическим роботом.

### RViz, X11 и xhost

Обычный `make up-single` / `make up-dual` **не подключает GUI-overlay**. При
`LAUNCH_RVIZ=true` без дисплея RViz может стартовать и сразу упасть с
`qt.qpa.xcb: could not connect to display`, пока MoveIt продолжает работать.
Это не ошибка mock и не повод переустанавливать Qt.

Команды ниже выполняйте **на Docker-хосте от пользователя графической сессии**,
не внутри ROS-контейнера и не из произвольной SSH-сессии. Сначала проверьте:

```bash
printf 'DISPLAY=%s\nXAUTHORITY=%s\n' "$DISPLAY" "${XAUTHORITY:-}"
xhost    # показать текущий контроль доступа и разрешения; ничего не меняет
```

| Дополнительное поле | Что задавать | Почему нет универсального default |
|---|---|---|
| `DISPLAY` | Значение графической сессии хоста, например `:0` или `:1`. Обычно уже экспортировано; можно указать в `.env`. | Не ставьте `:0` вслепую. `compose.gui.yaml` передаёт его и монтирует `/tmp/.X11-unix`. На headless-хосте реального окна без дисплейного сервера не будет. |
| `XAUTHORITY` | При cookie-авторизации — путь **на хосте** к существующему файлу авторизации сессии. Если переменная пуста, проверьте `xauth info`; `~/.Xauthority` — только возможный, не гарантированный путь. | Файл содержит секреты доступа. Его нужно явно смонтировать, а в контейнере указать путь монтирования. Одной переменной с путём хоста недостаточно. |

**В текущем `compose.gui.yaml` строки `XAUTHORITY` и mount cookie закомментированы.**
Поэтому в неизменённом overlay авторизацию нужно предоставить отдельно, например
ограниченным правилом `xhost` ниже. Для рекомендуемого cookie-варианта включите
обе строки в используемых сервисах `single` / `dual`, сохранив остальные настройки:

```yaml
environment:
  XAUTHORITY: /tmp/iimt.xauthority
volumes:
  - ${XAUTHORITY:?Set XAUTHORITY to an existing cookie file}:/tmp/iimt.xauthority:ro
```

Проверьте `test -f "$XAUTHORITY"`; используйте корректный cookie вашей X11-сессии,
не пустой файл. При рабочей cookie-авторизации ослаблять access control через
`xhost +` не требуется. Подробнее об авторизации:
[X.Org Xsecurity](https://www.x.org/releases/current/doc/man/man7/Xsecurity.7.xhtml).

#### Локальный запуск с ограниченным xhost

Для текущего root-контейнера и обычного локального X11 без UID remapping можно
временно разрешить доступ **локальному пользователю root**, не всем клиентам:

```bash
# Если доступ уже был открыт через xhost +, сначала верните проверку:
xhost -
xhost +si:localuser:root

make COMPOSE='docker compose -f compose.yaml -f compose.gui.yaml' \
  up-single \
  LAUNCH_ARGS='use_mock_hardware:=true enable_motion:=true launch_rviz:=true'
```

Для двух рук замените `up-single` на `up-dual`; не поднимайте оба стека одновременно.
Либо сохраните `LAUNCH_RVIZ=true` в `.env` и используйте тот же `COMPOSE` override.
Разрешение относится **ко всем подходящим локальным root-клиентам**, не только к
этому контейнеру. При rootless Docker / user namespace remapping UID может быть
другим; это не универсальная команда для таких конфигураций.

После завершения GUI уберите **только добавленное вами** разрешение, если оно не
было нужно другим приложениям до этой процедуры:

```bash
xhost -si:localuser:root
xhost
```

#### Что делает xhost + и почему это не default

```bash
xhost +    # отключает контроль доступа: НЕ рекомендуемый штатный способ
xhost -    # снова включает контроль доступа
```

`xhost +` разрешает подключаться **любому клиенту, способному достичь X-сервера**,
а не только Docker. Такой клиент потенциально получает доступ к экрану, вводу и
окнам. Команда не пробрасывает `DISPLAY`, не монтирует X11 socket и сама не включает
сетевое прослушивание сервера. Даже после неё GUI-overlay всё равно нужен.
Если вы временно использовали её для диагностики на изолированном стенде, закройте
тестовые GUI-клиенты и верните `xhost -`. Не включайте `xhost +` в `.env`, entrypoint,
Makefile или автозагрузку; авторизация cookie либо ограниченное правило предпочтительнее.

`xhost -` **не удаляет** ранее добавленные ACL-правила; для правила root нужна
отдельная `xhost -si:localuser:root`. Изменение ACL не разрывает уже установленные
соединения. Семантика команд: [официальная справка X.Org xhost](https://www.x.org/releases/current/doc/man/man1/xhost.1.xhtml).

В Wayland-сессии этот путь требует работающего **Xwayland** и его `DISPLAY`;
`xhost` не настраивает native Wayland. Удалённый SSH display нельзя автоматически
считать локальным `/tmp/.X11-unix`. `--privileged` и host IPC для этого рецепта не нужны.
Окно на вашем физическом дисплее проверяйте отдельно; документация не означает,
что текущая хостовая авторизация уже настроена.

## Пакеты и назначение

| Пакет | Состояние | Содержимое / роль |
|---|---|---|
| `iimt_ros2_driver` | **активный** | C++ direct FJT server, ServoJ schedule, независимый reader, diagnostics/timing, SDK headers и libiimtcl.so |
| `iimt_ros2_description` | **активный** | Однорукие URDF/meshes; для текущего стека используется CI-05 `iimt5.urdf` |
| `iimt_ros2_moveit_config` | **активный + legacy launch** | Single SRDF/KDL/OMPL/controllers; новый `iimt_driver_moveit.launch.py`, также сохранён старый `iimt_moveit.launch.py` |
| `iimt_dual_control` | **активный** | Два namespaced экземпляра single driver, arm YAML, общий FJT coordinator |
| `iimt_dual_description` | **активный** | Xacro двух CI-05, префиксы суставов, параметризованные крепления баз; общие meshes single description |
| `iimt_dual_moveit_config` | **активный** | `both_arms`/`left_arm`/`right_arm`, RRTConnect, scene node: четыре границы и пол |
| `iimt_ros2_control` | legacy-совместимость | Старый hardware SystemInterface, ros2_control; не запускается текущим direct launch |
| `iimt_ros2_msgs` | legacy-совместимость | RobotMode/SafetyMode/ProgramState, SetMode action, dashboard srv; сам не нода |
| `iimt_ros2_trajectory_action` | legacy-совместимость | Старый FJT → topic мост; не использовать совместно с direct driver |

Сохранение трёх legacy-пакетов позволяет собирать прежние зависимости и изучать
историю интерфейса, но не делает их рекомендуемым runtime. Необязательный legacy
`db:=true` требует отдельно установленного Mongo warehouse и не входит в образ;
текущим launch база данных не нужна. **Не путайте**
`iimt_driver_moveit.launch.py` с прежним `iimt_moveit.launch.py`.

### Главные ноды и endpoints

Single: `/iimt_ros2_driver`, `/robot_state_publisher`, `/move_group`, optional RViz;
управление `/iimt_group_controller/follow_joint_trajectory`, состояние `/joint_states`.

Dual: `/left/iimt_ros2_driver`, `/right/iimt_ros2_driver`, `/iimt_dual_coordinator`,
`/robot_state_publisher`, `/move_group`, `/iimt_workspace`, optional RViz;
управление `/both_arms_controller/follow_joint_trajectory`, измерения
`/left/joint_states` и `/right/joint_states`.

Каждый driver публикует `~/diagnostics` и `~/trajectory_timing`; MoveIt предоставляет
`/move_action`, `/execute_trajectory`, `/check_state_validity`, `/get_planning_scene`,
`/apply_planning_scene`. Полный контракт и QoS — [архитектура](docs/architecture.md).

## Прямой ros2 launch

Для shell в образе используйте `make shell`: ROS/overlay/venv уже sourced.
Ниже эквиваленты текущих launch с явным mock; не запускайте одновременно с уже
работающим Compose single/dual.

```bash
ros2 launch iimt_ros2_moveit_config iimt_driver_moveit.launch.py \
  use_mock_hardware:=true enable_motion:=false launch_rviz:=false \
  robot_type:=5 command_rate:=125 read_rate:=125 \
  feedback_transport:=tcp read_timeout:=0.1 trajectory_advance_ms:=0
```

```bash
ros2 launch iimt_dual_moveit_config dual_moveit.launch.py \
  use_mock_hardware:=true enable_motion:=false launch_rviz:=false \
  robot_type:=5 \
  left_command_rate:=125 right_command_rate:=125 \
  left_read_rate:=125 right_read_rate:=125 \
  left_feedback_transport:=tcp right_feedback_transport:=tcp \
  left_read_timeout:=0.1 right_read_timeout:=0.1 \
  left_trajectory_advance_ms:=0 right_trajectory_advance_ms:=0 \
  left_base_xyz:='-0.0008297 -0.5380000 0.0818739' \
  left_base_rpy:='0 0 -0.6981317008' \
  right_base_xyz:='-0.345440489323 0.411679944176 -0.082000000000' \
  right_base_rpy:='0 0 0' workspace_margin:=0.5 floor_z:=auto
```

**Правая база задана относительно левой**, левая — относительно world.
Преобразования и limits должны соответствовать фактическому монтажу до real launch.
Для real нужны `use_mock_hardware=false`, проверенные `robot_ip` либо два разных
`left_robot_ip`/`right_robot_ip`; отдельно `enable_motion=true` разрешает цели.
Это не включает питание и не снимает тормоза. Реальные движения при упаковке проекта
не запускались; готовой универсально безопасной real-команды здесь нет.

## Notebook и измерения

Оба notebook берут позиции/время из `data/trajectory_both_TimeGrid_30Hz.csv`, без
запуска SDK в Python. Single выбирает left-столбцы; dual — обе руки. При обновлении
2026-10-29 текущие пользовательские настройки и outputs сохранены: в двуруком
notebook флаги RUN_APPROACH/RUN_MOTION/RUN_RANDOM и CONFIRM_DUAL_PATH_SAFE включены.
Перед реальным запуском проверьте их; не используйте Run All на подключённом роботе.
Docker без пользовательского `.env` по-прежнему запускает mock с motion=false.
Поиск CSV переносимый, без зависимости от `/root/ws`. Предыдущая версия dual notebook
сохранена в `.migration_backup/20261029_docker_refresh/notebooks` исходного workspace.

Подвод выполняется отдельно от CSV. Single — медленный суставной подвод без MoveIt
collision preflight; dual — совместный RRTConnect-план и отдельная ячейка исполнения.
CSV start delay3 s задаётся **header.stamp сообщения из notebook**, не параметром driver.
Подробная последовательность, случайная цель и экспорт — [notebooks.md](docs/notebooks.md).

Графики используют глобальные timestamps; ошибка actual−desired на том же времени.
При ошибках RRTConnect-подвода/random показываются доступные частичные данные; при
неизвестном t0 ошибка не выдумывается. `NaN` означает отсутствие достаточных данных,
не «нулевое ускорение/ошибка». Хостовый timestamp read не равен аппаратному timestamp
энкодера — это ограничение измерительного канала.

## Workspace, коллизии и безопасность

Отступы по умолчанию50 cm от прямоугольника обеих баз в world X/Y; стороны можно
настроить отдельно. Сверху потолка нет. Пол по умолчанию проходит через нижнюю базу.
Стены — сплошные наружные box-области; пол — поверхность, не заполненный подпол.
Все moving-link/tool collision checks сохраняются; разрешён контакт пола только
с неподвижными креплениями. Перед планированием дождитесь сообщения о применении сцены.

RRTConnect проверяет взаимные коллизии обеих рук. Прямой dual CSV проходит только
дискретный preflight notebook, а direct action-клиент может обойти MoveIt вообще.
Модель не видит неизвестных препятствий. Нельзя считать успешный plan подтверждением
безопасности стенда без точных баз, инструментов и независимой остановки.

## Зависимости и сборка без Docker

Рекомендуемый воспроизводимый путь — Dockerfile: он ставит MoveIt, RViz, xacro,
robot_state_publisher, CycloneDDS, build/colcon/rosdep, ros2_control для legacy и
зависимости всех package.xml. Python requirements включают JupyterLab, ipykernel,
numpy, pandas, matplotlib, nbformat и openpyxl. CSV notebooks не требуют Excel;
openpyxl оставлен для работы с исходными табличными данными.

Для native Ubuntu22.04 сначала установите ROS Humble по официальной инструкции.
Затем, из корня проекта, в окружении системного Python3.10:

```bash
source /opt/ros/humble/setup.bash
rosdep update --rosdistro humble
rosdep install --from-paths ros2_ws/src --ignore-src --rosdistro humble -y
cd ros2_ws
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
cd ..
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements-notebook.txt
python -m ipykernel install --user --name iimt-ros2 --display-name 'IIMT ROS 2 Humble (Python 3.10)'
python -m jupyter lab notebooks
```

`rosdep init` выполняется один раз администратором, если rosdep ещё не инициализирован.
Смысл `--ignore-src` — не устанавливать уже присутствующие исходные пакеты;
[официальное руководство rosdep](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Intermediate/Rosdep.rst).
Этот native bootstrap целиком на чистой Ubuntu отдельно не прогонялся: сборка
пакетов проверена в Humble-контейнере, а установка окружения выполняется Dockerfile.
Native-сборка требует доступных build tools/rosdep/venv, уже перечисленных в Dockerfile.
Не смешивайте системный Humble с произвольным Python3.11/3.12 или сторонним PyPI rclpy.

## Проверка и ограничения воспроизводимости

`make verify` последовательно проверяет зависимости, все тесты, настоящее ядро
Jupyter с загрузкой CSV и графиками, mock-граф ROS и исполнение mock-траекторий
single/dual с планированием RRTConnect. `make smoke` отдельно проверяет граф без целей;
`make smoke-execution` явно разрешает **только mock**, проверяет его параметры перед
отправкой целей и использует loopback IP. Пользовательский `.env` не включает в этих
тестах реальное оборудование. Они не выполняют motion-ячейки notebook.

Domains по умолчанию: обычные unit218, SDK-fixture219, graph217, execution216;
`IIMT_TEST_DOMAIN_ID` переопределяет graph/execution/unit (кроме фиксированного219
SDK-fixture). Все проверки используют localhost-only. На занятом хосте выбирайте
свободный domain; эталонные проверки поставки дополнительно проводятся в контейнерах
с `network=none`, без подключения исходного workspace.

Результаты и сравнение с исходным workspace — [отчёт проверки](docs/verification.md).
Это подтверждает ROS API, зависимости и mock-поведение, а не реальные задержки SDK,
точность манипуляторов, физическую геометрию или E-stop. RViz установлен, но его
окно/X11 на физическом дисплее этой ревизией не проверяются.

Сообщения ошибок с рукой, сочленением, углами, timestamp и первичной причиной
Stop/Clear включены в Docker. [Описание полей](ros2_ws/src/iimt_ros2_driver/ERROR_DIAGNOSTICS_RU.md).

Версии Python ограничены диапазонами, а базовый образ/apt repositories не закреплены
digest/snapshot. Поэтому повторная сборка совместима по заявленному стеку, но не
является обещанием побитовой идентичности. Для квалифицированного стенда фиксируйте
собранный image digest, firmware, конфигурацию и результаты commissioning.

## Лицензирование

Сохраняйте notices и license-поля исходных ROS-пакетов. Их декларации **не означают**,
что бинарная `libiimtcl.so`, vendor headers, meshes или материалы производителя
автоматически имеют ту же лицензию. SDK включён как существующая зависимость проекта;
перед передачей Docker-образа/архива третьим лицам отдельно проверьте разрешение IIMT
на распространение. На весь собранный проект не назначается blanket open-source license.
