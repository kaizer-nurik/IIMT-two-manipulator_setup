# Проверка Docker-поставки — 2026-10-29

**Итог:** все необходимые зависимости текущих direct-driver single/dual workflow
и двух notebook установлены; обновлённые исходники присутствуют в образе;
полный CSV, MoveIt RRTConnect и графики работают в mock так же, как в исходном workspace.
Это проверка программного поведения, **не аппаратной точности/задержки**.

## Что исправлено при ревизии

- Последняя диагностика ошибок с рукой, суставом, углами, свежестью измерения,
  первичной причиной и отдельным результатом Stop/Clear включена и пересобрана.
- Добавлены прямые runtime-зависимости в package.xml:
  `iimt_ros2_description` — launch, launch_ros, ament_index_python, python3-yaml,
  xacro, robot_state_publisher, rviz2;
  `iimt_ros2_moveit_config` — launch/launch_ros;
  `iimt_ros2_driver` — launch. Ранее они были установлены транзитивно,
  но декларации пакетов были неполными.
- В dual notebook устранён повторно появившийся жёсткий `/root/ws`: CSV находится
  по структуре проекта в `data/`. Настройки экспериментов и outputs сохранены.
  Резервная копия до этой правки: `.migration_backup/20261029_docker_refresh/notebooks`
  исходного workspace. Пользовательский `.env` не изменён.
- `scripts/test.sh` теперь немедленно возвращает ненулевой exit code при провале
  colcon test; добавлены повторяемые проверки зависимостей, kernel и исполнения.

## Доказательства

Новый образ проверен в отдельных контейнерах **network=none**, без bind mount
старого workspace. Физическим роботам команды не отправлялись. Сравнение с
исходным вариантом запускало `/root/ws/ros2_ws/install` из read-only mount
в отдельном baseline-контейнере на предыдущем Docker runtime. Это сравнение
двух программных сборок на одном хосте, а не повтор реального опыта на манипуляторах.

| Проверка | Результат |
|---|---|
| Сборка Docker + rosdep | Все 9 пакетов собраны |
| Происхождение исходников | 180 файлов пакетов/notebook/data/scripts/docker совпадают по SHA256 |
| Установленный driver binary | Новые поля диагностики присутствуют; тестовые SDK-сбои проверены |
| `rosdep check` | Все системные зависимости удовлетворены |
| `python -m pip check` | Нет конфликтов требований |
| ROS/Python imports | 23 модуля успешно импортируются, включая rclpy и rpyutils.import_c_library |
| Python / kernel | Python3.10.12; iimt-ros2 использует /opt/iimt/venv/bin/python |
| Vendor SDK `ldd` | Нет отсутствующих shared libraries, Linux amd64 |
| Colcon tests | **29 tests, 0 errors, 0 failures, 0 skipped**; в одном CTest дополнительно7 action-сценариев драйвера |
| Ресурсы | Все 56 literal package:// ссылок разрешаются |
| Notebook AST / nbformat | Оба notebook валидны |
| Реальный Jupyter kernel | CSV loader обоих notebook:121×6/121×12; два inline PNG-графика |
| Single/dual graph smoke | actions, joint states и сцена доступны, mock=true/motion=false |
| Полный CSV playback | Все121 исходная точка, positions-only,125Hz, исходные времена, общий будущий старт+3s |
| Dual MoveIt | RRTConnect планирует; все возвращённые waypoints collision-valid; ExecuteTrajectory успешен |
| Пол / workspace | Четыре стены и floor присутствуют; контакт с floor отклоняется |
| Compose два контейнера | notebook видит обе руки и combined action через DDS |
| Compose launch-lock | Одновременный single запуск при активном dual отвергнут до запуска ROS |
| Jupyter HTTP | Без token API отклоняет запрос; с token kernelspec доступен |

## Сравнение с исходной сборкой

Одинаковый CSV длительностью3.999999989s,121 точка, только positions/time_from_start;
command/read125Hz, начало через3s. Подвод — отдельная инициализация mock, не часть CSV.

| Измерение | Исходный workspace | Новый Docker |
|---|---:|---:|
| Single: result относительно номинального t0 | 4.008037s | 4.007868s |
| Dual: result относительно номинального t0 | 4.012592s | 4.014611s |
| Ошибка конечной позиции single/dual | 0rad / 0rad | 0rad / 0rad |
| Dual RRTConnect + ExecuteTrajectory | PASS | PASS |
| Ошибка конца MoveIt mock | 0.0000899rad | 0.0000993rad |

Время в таблице — момент получения успешного action result относительно запрошенного
старта; оно включает доставку результата и jitter ОС. Это не аппаратный timestamp.
Ожидаемые3s ожидания дают суммарный roundtrip около7s, а не удвоение4-секундного пути.
Нулевая ошибка mock не является измерением точности робота. OMPL стохастический:
число waypoints может отличаться (в этих прогонах10/9), это не расхождение API.

## Повторение

На Docker-хосте из корня проекта:

```bash
make build
make verify
```

Либо отдельно: `make deps`, `make test`, `make notebook-smoke`, `make smoke`,
`make smoke-execution`. Все motion goals в тесте исполнения разрешены только после
проверки mock=true и loopback IP. Тест **не** запускает motion-ячейки notebook.

Для проверки Compose использовался отдельный временный проект с mock=true,
motion=false, loopback IP, выделенным domain215 и общей изолированной loopback-сетью
для dual/notebook. Рабочий `.env` и реальные IP не использовались.

## Границы и известные ограничения

- Физический SDK/Ethernet timing, точность, калибровка, тормоза и E-stop не проверялись.
  Сохранено best-effort поведение; realtime и универсальная компенсация330ms не обещаются.
- RViz/плагины установлены, но реальное GUI/X11 окно не проверялось. Для него нужен
  DISPLAY/Xauthority хоста и `compose.gui.yaml`.
- Остался известный Matplotlib warning `Unable to import Axes3D` из-за сочетания
  системной и pip-версии. Нужные notebook **2D inline-графики проверены**; 3D не входит
  в используемый функционал. Этот warning не является ошибкой чтения CSV или ROS.
- Необязательный старый `db:=true` требует отдельно установленного Mongo warehouse;
  текущие direct-driver launch его не используют.
- Hardware legacy-тесты чувствительны к параллельной CPU-нагрузке; текущий полный
  последовательный прогон прошёл без ослабления их ограничений.
- Пользовательские RUN_* флаги dual notebook сейчас включены. Проверяйте их перед
  работой с железом; обновление не сбрасывает настройки и не запускает ноутбук само.
- Apt/base image и диапазоны pip не закреплены snapshot/digest. Проверен конкретно
  собранный image, а не все будущие версии зависимостей.

Журналы вне распространяемого runtime (`/workspace/results/`):
`docker_update_build_20261029.log`, `docker_update_runtime_build_20261029.log`,
`docker_new_image_audit_20261029.log`, `docker_parity_original_20261029.log`,
`docker_source_provenance_20261029.log`, `docker_source_hashes_20261029.json`,
`docker_dependency_audit_20261029.md`, `docker_jupyter_http_20261029.log`,
`docker_compose_audit_20261029.log`.
