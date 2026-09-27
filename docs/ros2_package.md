# Пакет ROS 2 Humble

## Ссылка на исходный код

[Исходный код tram_model](https://github.com/ViktorDanilin/axiom_tram_odom)

Решение состоит из ROS 2 пакета:

- `tram_model` — динамическая модель, оценивание скорости и положения, привязка к маршруту и публикация результата.

Основной исполняемый узел — `train_model_ros`. Физическая модель реализована в `tram_model/tram_model.py`, ROS-адаптер — в `tram_model/train_model_ros.py`. Динамический узел вычисляет продольную скорость по цепочке
```{math}
\text{команда водителя}\rightarrow\text{тяговый момент}\rightarrow\omega\rightarrow F_{\rm adh}\rightarrow a\rightarrow v\rightarrow s,
```
а измерения передней и задней тележек корректируют расчёт. EKF и маршрутная карта участвуют в оценке положения.

## Входные топики

| **Топик** | **Тип** | **Назначение** |
|:---|:---|:---|
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/DriverControllerCommand` | Ступень контроллера: положительная — тяга, отрицательная — торможение, ноль — выбег. |
| `/vehicle/front_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor` | Скорость передней тележки. В предоставленных записях фактические значения масштабируются из км/ч в м/с параметром `wheel_velocity_scale=1/3.6`. |
| `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor` | Скорость задней тележки; используется совместно с передней для коррекции модели. |
| `/sensing/gnss/master/fix`, `/sensing/gnss/rover/fix` | `sensor_msgs/NavSatFix` | Начальная координата, курс по базе антенн и привязка к карте; опционально — коррекция EKF, если включено GNSS-fusion. |

## Выходные топики

| **Топик** | **Тип** | **Содержимое** |
|:---|:---|:---|
| `/result/velocity` | `tram_vehicle_msgs/VelocitySensor` | Оценка продольной скорости, м/с. |
| `/result/position` | `nav_msgs/Odometry` | Оценка положения, ориентации и продольной скорости в системе `map`. |
| `/result/path` | `nav_msgs/Path` | Накопленная оценённая траектория. |
| `/reference/path` | `nav_msgs/Path` | Референсная GNSS-траектория для визуальной проверки. |
| `/route/position`, `/route/path`, `/route/map` | — | Маршрутная оценка и вспомогательные данные привязки к path graph. |

Дополнительно публикуется TF `map -> base_link` и статические TF антенн. До получения первой GNSS-точки преобразование в систему маршрутной карты недоступно; маршрутные выходы требуют успешной привязки к карте.

## Сборка

Из корня ROS 2 workspace:

    colcon build --packages-select tram_vehicle_msgs tram_model
    source install/setup.bash
