# Запуск IIMT

## Как запустить

Готовую `.env` **не перезаписывать** через `cp .env.example .env`. Подключить контроллеры и компьютер к LAN; Ethernet компьютера — `192.168.50.2/24`.

В `.env` уже стоят `USE_MOCK_HARDWARE=false`, `LAUNCH_RVIZ=true`. `ENABLE_MOTION=false` — просмотр; для движения после проверки физического пути установить `true`. Изменять режим только на неподвижных роботах, затем пересоздать выбранный сервис.

```bash
cd /home/vladislav/Documents/IIMT-SDK/IIMT-two-manipulator_setup
xhost +si:localuser:root
make build                         # первый запуск / после изменения исходников
docker compose stop single dual    # только когда активные движения завершены
make up-dual                       # для одной правой руки: make up-single
make notebook
docker compose logs --tail 60 notebook
```

Открыть ссылку с токеном из логов. Kernel: **IIMT ROS 2 Humble (Python 3.10)**.

| Режим | Notebook | Настройки для повторения теста |
|---|---|---|
| Single, правая `.163` | `notebooks/iimt_ci05_left_csv_ethernet.ipynb` | `TIME_SCALE=1.5`, подвод 12 с |
| Dual | `notebooks/iimt_ci05_dual_csv_ethernet.ipynb` | `TIME_SCALE=1.5`, `APPROACH_VELOCITY_SCALE=0.2`, `APPROACH_ACCELERATION_SCALE=0.2` |

**В dual-notebook эти значения нужно выставить вручную:** исходные пока `1.0` и `0.5`. Имя `left` в single-notebook означает столбцы CSV; руку выбирает `ROBOT_IP`.

Выполнять по ячейкам: загрузка → подключение → предпросмотр подвода → `RUN_APPROACH=True` → предпросмотр CSV → `RUN_MOTION=True`. Не использовать `Run All`. Для dual подтвердить проверку обоих путей через `CONFIRM_DUAL_PATH_SAFE=True`; `RUN_RANDOM` оставить `False`. После изменения режима или позы делать новый предпросмотр.

После завершения вернуть `RUN_*` и `ENABLE_MOTION` в `false`; остановить стек через `docker compose stop single dual`. Питание стенда выключается отдельно, штатно с пультов.

## Что изменено 30.10.2026 и зачем

| Изменение | Назначение |
|---|---|
| `.env`: левая `.95`, правая `.163`, `ROBOT_IP=.163` | Исправлена перепутанная привязка из исходной инструкции; single выбирает правую руку. |
| `.env`: измеренные базы, world в правой базе; смещение левой `[0.55/√2, −0.55/√2, 0.08]` м, поворот нулевой | Правильное расположение рук в модели и проверках столкновений. `RIGHT_BASE_XYZ` записан как обратное смещение — правая относительно левой. |
| `iimt5.urdf`, `ci05_arm.xacro`: ось J6 `0 0 1` | Направление вращения модели согласовано с FK контроллеров. |
| `.env`: упреждение single/left/right `320.0` мс | Отправка команд с упреждением по настройкам из инструкции. |
| `.env`: подключён `compose.gui.yaml`; ограниченный доступ X11 через `xhost` | RViz получает доступ к локальному экрану. |
| В notebooks выключены `RUN_*` и `CONFIRM_DUAL_PATH_SAFE`, очищены старые выводы | Ячейки движения требуют явного разрешения; прошлые результаты не смешиваются с новым запуском. |
