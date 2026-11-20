# Unitree Go1 — T1 и T2

Решение команды 5-HT2A COLLECTIVE для заочного этапа. Контейнер поддерживает `ping` и `T2`. Узел T2 читает `/lowcmd_bare` и публикует проверенные команды в `/lowcmd`.

## Сборка и проверка

```bash
docker buildx build --platform linux/amd64 --load -t 5ht2a-go1:t2 .
python3 verify.py 5ht2a-go1:t2
```

В выводе ожидаются `offline ping OK`, `Tasks: T2` и UID 10001. Сеть нужна для сборки. Если Docker требует прав администратора, добавьте `sudo` к обеим командам.

Для проверки ROS 2 запустите из каталога официального симулятора:

```bash
docker pull --platform linux/amd64 registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0
python3 run_local.py --task T2 --headless --logs \
  --sim-image registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0 \
  --participant-image 5ht2a-go1:t2
```

Для просмотра движения используйте `--gui` вместо `--headless`. В публичном запуске поток тестовых целей нужно подать самостоятельно в `/lowcmd_bare`.

## Как устроен T2

- `tasks/t2_node.py` читает входную команду и время симуляции, публикует ответ.
- `tasks/t2_safety.py` ограничивает углы и размер шага. MuJoCo проверяет контакты на промежуточных позах по таблице из 655 пар.
- Если команда опасна, узел продолжает отправлять последнюю безопасную цель. Через 400 мс без входа скорость и добавочный момент обнуляются.

Модель и таблица взяты из [публичного симулятора](https://github.com/Urika000/go1-simulator-public/tree/3473f086fd8a111592b4b9d822500c79f20c811a/models/mujoco/unitree_go1). Версия пакета [unitree_go](https://github.com/unitreerobotics/unitree_ros2/tree/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/cyclonedds_ws/src/unitree/unitree_go) совпадает с версией симулятора. Лицензии лежат в `vendor/`.
