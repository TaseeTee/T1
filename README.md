# Unitree Go1 — T1, T2, T3 и T4

Решение команды 5-HT2A COLLECTIVE для заочного этапа. Контейнер поддерживает `ping`, `T2`, `T3` и `T4`. Узел T2 читает `/lowcmd_bare` и публикует проверенные команды в `/lowcmd`.

## Сборка и проверка

```bash
docker buildx build --platform linux/amd64 --load -t 5ht2a-go1:t4 .
python3 verify.py 5ht2a-go1:t4
```

В выводе ожидаются `offline ping OK`, `Tasks: T2, T3, T4` и UID 10001. Сеть нужна для сборки. Если Docker требует прав администратора, добавьте `sudo` к обеим командам.

Для проверки ROS 2 запустите из каталога официального симулятора:

```bash
docker pull --platform linux/amd64 registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0
python3 run_local.py --task T2 --headless --logs \
  --sim-image registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0 \
  --participant-image 5ht2a-go1:t4
```

Для просмотра движения используйте `--gui` вместо `--headless`. В публичном запуске поток тестовых целей нужно подать самостоятельно в `/lowcmd_bare`.

## Как устроен T2

- `tasks/t2_node.py` читает входную команду и время симуляции, публикует ответ.
- `tasks/t2_safety.py` ограничивает углы и размер шага. MuJoCo проверяет контакты на промежуточных позах по таблице из 655 пар.
- Если команда опасна, узел продолжает отправлять последнюю безопасную цель. Через 400 мс без входа скорость и добавочный момент обнуляются.

Модель и таблица взяты из [публичного симулятора](https://github.com/Urika000/go1-simulator-public/tree/3473f086fd8a111592b4b9d822500c79f20c811a/models/mujoco/unitree_go1). Версия пакета [unitree_go](https://github.com/unitreerobotics/unitree_ros2/tree/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/cyclonedds_ws/src/unitree/unitree_go) совпадает с версией симулятора. Лицензии лежат в `vendor/`.

## T4 — одометрия

`/entrypoint.sh T4` запускает оценку смещения по лидару и ориентации по IMU.
Выходы — `/odom` и преобразование `odom → trunk` в `/tf`, с одной позой и меткой времени.
Позиция оценивается в плоскости XY, высота фиксирована в нуле. Команды движения
подаёт симулятор или судья. Узел не использует траекторию команд и истинную позу.

Соберите образ и проверьте, что `verify.py` показывает `Tasks: T2, T3, T4`.
Из каталога публичного симулятора запустите:

```bash
python3 run_local.py --task T4 --headless --logs \
  --sim-image registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0 \
  --participant-image 5ht2a-go1:t4
```

В логе ожидайте `T4 ready`. Для проверки точности требуется прогон с движением
и эталонной траекторией в публичном симуляторе.

## T3 — ходьба Go1

`/entrypoint.sh T3` запускает политику ходьбы и штатный узел безопасности T2.
Политика читает `/cmd_vel` и IMU из `/lowstate`. Цели суставов идут в
`/lowcmd_bare`, а T2 публикует проверенные команды в `/lowcmd`.
Скорость для Judge — 0,22 м/с.

### Сборка и запуск

```bash
sudo docker buildx build --platform linux/amd64 --load -t 5ht2a-go1:t3 .
sudo python3 verify.py 5ht2a-go1:t3
```

В проверке ожидаются `offline ping OK` и `Tasks: T2, T3, T4`.
Из папки публичного симулятора:

```bash
sudo python3 run_local.py --task T3 --headless --logs \
  --sim-image registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0 \
  --participant-image 5ht2a-go1:t3
```

После `T3 ready` выполните из каталога проекта в другом Bash-терминале
(в Fish сначала запустите `bash`):

```bash
CONTAINER=$(sudo docker ps --filter ancestor=5ht2a-go1:t3 --format '{{.Names}}' | head -n1)
sudo docker exec -i "$CONTAINER" bash -c '
  source /opt/ros/humble/setup.bash
  export PYTHONPATH="/opt/participant/tasks:$PYTHONPATH"
  exec python3 -
' < training/check_ros_t3.py
```

Тест подаёт скорость и измеряет 3 м по публичному `/odom/ground_truth`.
Топик эталона используется только диагностикой. Для независимых повторов
остановите `run_local.py` через Ctrl+C и заново запустите оба контейнера.

### Обучение

Политика представляет собой четырёхтактную походку с обратной связью по IMU.
Десять весов управляют частотой, шагом, подъёмом стопы и коррекцией наклона и
курса. Веса обучены в MuJoCo методом кросс-энтропии (CEM): каждый кандидат
проходит эпизод, лучшие по суммарной награде формируют следующее поколение.
Награда учитывает движение вперёд, слежение за скоростью, устойчивость корпуса,
уход с курса и усилия приводов. При обучении действует ограничение шага, при
финальной оценке включён полный фильтр T2.

Веса лежат в `tasks/t3_weights.json`. Конфигурация, журнал наград и результаты
эпизодов лежат в `training/`. Повторить обучение и проверку на CPU можно
из корня проекта и отдельного Python-окружения:

```bash
python3 -m pip install mujoco==3.3.7 numpy==1.26.4
PYTHONPATH=tasks python3 training/train_t3.py
PYTHONPATH=tasks python3 training/train_t3.py --evaluate 10 --jobs 4
```

Локальная оценка использует модель Go1 и тот же закон PD и пределы момента,
что и публичный симулятор. ROS-обмен и задержки проверяются отдельным прогоном
из предыдущего раздела, итоговый балл выставляет Judge.
