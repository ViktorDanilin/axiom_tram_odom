# Инструкция для жюри по проверке

## Подготовка окружения

Нужен ROS 2 Humble и рабочее пространство `colcon`. Пакет зависит от `tram_vehicle_msgs` (типы `/vehicle/*` и `/result/velocity`). Этот пакет в репозиторий модели не входит: он уже должен лежать в `src/` workspace.

Ожидаемая раскладка:

```
<ros2_ws>/
├── src/
│   ├── tram_vehicle_msgs/     # уже есть в workspace
│   └── tram_model/            # этот репозиторий
├── build/
├── install/
└── log/
```

В каждом новом терминале сначала подключите Humble:

```bash
source /opt/ros/humble/setup.bash
```

## Клонирование в workspace

Из каталога `src` уже существующего workspace (рядом с `tram_vehicle_msgs`):

```bash
cd <ros2_ws>/src
git clone https://github.com/ViktorDanilin/axiom_tram_odom.git tram_model
```

Каталог клона должен называться `tram_model`, чтобы пути ниже совпали с пакетом. `tram_vehicle_msgs` повторно клонировать не нужно.

ROS 2 bag в git не хранятся (`bags/` в `.gitignore`). Записи кладутся локально, например в `src/tram_model/bags/`.

## Сборка

Собираются оба пакета: сообщения, затем модель, которая от них зависит.

```bash
cd <ros2_ws>
colcon build --packages-select tram_vehicle_msgs tram_model
source install/setup.bash
```

`source install/setup.bash` нужен в каждом терминале, где запускаются нода или bag.

## Запуск модели

```bash
ros2 launch tram_model tram_model.launch.py position_source:=route velocity_source:=model
```

По умолчанию стартует узел `train_model_ros` и RViz. Скорость публикуется из физической модели (`velocity_source:=model`), положение — по маршруту (`position_source:=route`).

## Воспроизведение bag

В другом терминале, с тем же окружением (`source /opt/ros/humble/setup.bash` и `source <ros2_ws>/install/setup.bash`). Команда ниже — из каталога, где лежит запись:

```bash
cd <ros2_ws>/src/tram_model/bags
ros2 bag play 30618_88aea4d9_gt/
```

Сначала запускается launch, затем bag. Нода читает `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity` и `/vehicle/driver_position_cmd` и публикует `/result/velocity` и `/result/position`.

## Что должно появиться на выходе

Минимально обязательные топики:

    /result/velocity
    /result/position

Дополнительно доступны:

    /result/path
    /reference/path
    /route/position
    /route/path
    /route/map

Проверка наличия и типов:

    ros2 topic info /result/velocity
    ros2 topic info /result/position
    ros2 topic echo /result/velocity --once
    ros2 topic echo /result/position --once

Проверка фактической частоты публикации:

    ros2 topic hz /result/velocity
    ros2 topic hz /result/position

## Логи

Логи узла выводятся в консоль ROS 2. Для более подробной диагностики можно запустить узел с уровнем логирования `info` или `debug`. Параметр `log_period_s` задаёт период служебного логирования в реализации узла.

## Проверка точности скорости

Эталон — `/localization/kinematic_state`. Выход модели интерполируется по `header.stamp` к временным меткам эталона, без оптимизации временного сдвига. Для каждой точки
```{math}
e_i=\hat v_i-v_{\rm ref,i}.
```

Основные метрики:
```{math}
\mathrm{RMSE}=\sqrt{\frac{1}{N}\sum_{i=1}^{N}e_i^2},\qquad
\mathrm{bias}=\frac{1}{N}\sum_{i=1}^{N}e_i,
```
а также 95-й процентиль $|e_i|$. Отдельно оцениваются участки $v_{\rm ref}>1$ м/с, малая скорость $0{,}05<v_{\rm ref}\leq1$ м/с, тяга и торможение.

В репозитории для этого используется `scripts/evaluate_model.py`. Финальная версия репозитория должна содержать рабочую команду запуска этого скрипта и зафиксированный набор входных bag/масок выборки.

## Проверка положения

Для `/result/position` проверяются:

- корректность `header.stamp`, `frame_id`, позиции и продольной скорости;

- финальный дрейф относительно пройденной дистанции;

- along-track ошибка относительно референса;

- при наличии маршрутной карты — cross-track ошибка;

- визуальное совпадение оценённого пути с GNSS/reference path в RViz.

## Проверка задержки и производительности

На приложенном профиле DDS round-trip основная масса измерений находится ниже 100 мс; видны единичные выбросы выше 100 мс, но ниже допустимого пикового уровня 250 мс. Для формальной сдачи рекомендуется считать из исходного лога как минимум median, P95 и maximum latency, а также долю сообщений выше 100 мс.

```{figure} assets/latency_roundtrip.jpg
:width: 100%
:align: center

Пример профиля DDS round-trip в Native ROS 2 Humble / WSL при воспроизведении 1x. Красная линия — 100 мс, жёлтая — 250 мс.
```

Частота публикации проверяется через `ros2 topic hz`. CPU/RAM необходимо снимать для PID `train_model_ros` на той же машине и при той же скорости воспроизведения bag. В текущих предоставленных материалах точные численные CPU/RAM не зафиксированы, поэтому их нельзя достоверно указывать без дополнительного замера.

## Быстрая визуальная проверка

```{figure} assets/rviz_route.jpg
:width: 100%
:align: center

Пример визуализации оценённого пути и референсной траектории в RViz.
```
