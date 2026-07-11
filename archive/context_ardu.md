# Isaac Sim + ArduPilot SITL — Контекст моста

Этот файл описывает всё про подключение ArduPilot SITL к Isaac Sim.
Цель: ArduPilot управляет дроном (ARM → takeoff 5), Isaac Sim симулирует физику.

---

## Архитектура

```
Isaac Sim (Windows)
  TCP 0.0.0.0:8228      ← WSL подключается сюда (bridge)
      ↕
wsl_bridge.py (WSL/Ubuntu)
  UDP 0.0.0.0:9002      ← ArduPilot шлёт servo-пакеты сюда
      ↕
ArduCopter SITL (WSL/Ubuntu)
  TCP 0.0.0.0:5760      ← НЕ подключаться! Вызывает блокировку JSON loop
  UDP → 127.0.0.1:14550 ← arm_takeoff.py слушает здесь (SERIAL1, UDP)
      ↕
arm_takeoff.py (WSL, терминал 3)
  udpin:0.0.0.0:14550
```

---

## Файлы проекта

| Файл | Назначение |
|------|-----------|
| `C:\VSCODE\ISAAC\notebooks\drone_control_ardupilot.ipynb` | Jupyter ноутбук: физика + TCP сервер 8228 |
| `C:\VSCODE\ISAAC\wsl_bridge.py` | Bridge WSL ↔ ArduPilot + Isaac Sim |
| `C:\VSCODE\ISAAC\arm_takeoff.py` | Минимальный MAVLink клиент: ARM + взлёт |
| `C:\VSCODE\ISAAC\sitl_defaults.parm` | Параметры ArduPilot по умолчанию (загружаются при старте) |

---

## Запуск (каждый раз) — ПОРЯДОК ВАЖЕН

### 1. Isaac Sim (Windows) — уже открыт

Расширение `isaacsim.code_editor.python_server` включено (TCP порт 8226).
Сцена `first_scene_isaac01.usd` загружена, дрон `/World/cf2x` присутствует.

### 2. Jupyter ноутбук — запустить ВСЕ ячейки по порядку

```
Kernel → Restart Kernel and Run All Cells
```

Ячейки в правильном порядке:
1. `cell-1` — функция `execute_in_isaac`
2. `23c3e6c0` — суставы пропеллеров (Drive снят)
3. `27b2c7e8` — `omni.timeline.play()` — запустить симуляцию
4. `d617f38f` — конфигурация дрона (масса, тяга, BODY_ID)
5. `e987e0e3` — функции `read_state`, `ROTOR_CONSTANTS`, `PROP_OFFSETS_XY`
6. **`45968662`** — bridge-ячейка: physics callback + TCP сервер на 0.0.0.0:8228

Ожидаемый вывод ячейки 45968662:
```
ok
OK: _pwm_to_vel MAX_VEL=1100.0
Physics callback: OK
TCP bridge server: 0.0.0.0:8228
Запусти wsl_bridge.py в WSL
```

### 3. Терминал WSL 1 — ArduPilot SITL

**ВАЖНО**: удалять eeprom.bin при каждом запуске.
**ВАЖНО**: НЕ подключаться к TCP 5760 — блокирует JSON loop.
**ВАЖНО**: `--serial0 tcp:5760` можно убрать — он не нужен, UDP через serial1 достаточно.

```bash
rm -f ~/ardupilot/eeprom.bin && cd ~/ardupilot && ./build/sitl/bin/arducopter --model JSON:127.0.0.1:9002 --speedup 1 -I0 --serial1 udpclient:127.0.0.1:14550 --defaults /mnt/c/VSCODE/ISAAC/sitl_defaults.parm
```

Ожидаемые строки в выводе:
```
JSON control interface set to 127.0.0.1:9002
UDP connection 127.0.0.1:14550
No JSON sensor message received, resending servos   ← нормально до старта bridge
JSON received: timestamp, imu: gyro, imu: accel_body, position, attitude, velocity
EKF3 IMU0 origin set
EKF3 IMU0 is using GPS
```

### 4. Терминал WSL 2 — Bridge

```bash
python3 /mnt/c/VSCODE/ISAAC/wsl_bridge.py
```

Ожидаемый вывод при успехе:
```
WSL Bridge запущен
  Isaac Sim TCP : 172.18.224.1:8228
Self-test loopback... OK
[100] 6000Hz  PWM:[1000, 1000, 1000, 1000]  pos(NED):[0.0, -0.0, -0.01]
```

### 5. Терминал WSL 3 — ARM + TAKEOFF

```bash
python3 /mnt/c/VSCODE/ISAAC/arm_takeoff.py
```

Ожидаемый вывод при успехе:
```
ArduPilot онлайн! sysid=1, mode=0
→ Ждём EKF origin set + GPS using (до 120 сек)...
  [MSG] EKF3 IMU0 origin set
  [MSG] EKF3 IMU0 is using GPS
  EKF + GPS готовы — ждём 3 сек для стабилизации...
→ Ставим режим GUIDED...
→ ARM попытка 1 (ждём 5 сек)...
   ARM принят!
→ SET_POSITION_TARGET z=-5.0м (GUIDED pos_control)...
  [SERVO!] PWM=[1450, 1450, 1450, 1450]  ← моторы спиннятся!
  [ALT] 0.5м
  [ALT] 1.2м ...
  Достигли 5.0м!
```

---

## Содержимое sitl_defaults.parm (актуально на 2026-06-29)

```
FRAME_CLASS 1
FRAME_TYPE 1
SCHED_LOOP_RATE 50
INS_FAST_SAMPLE 0
ARMING_CHECK 0
SIM_ACCEL_RND 0
SIM_GYR1_RND 0
SIM_GYR2_RND 0
FS_THR_ENABLE 0
FS_GCS_ENABLE 0
DISARM_DELAY 0
FS_EKF_ACTION 0
INS_USE2 0
INS_USE3 0
```

**Почему каждый параметр:**
- `FRAME_CLASS 1` + `FRAME_TYPE 1` — квадрокоптер X, обязательно для ARM
- `SCHED_LOOP_RATE 50` — снижает ожидаемый loop rate с 400 до 50 Гц. **Требует --defaults, нельзя менять на лету.**
- `INS_FAST_SAMPLE 0` — отключает fast IMU sampling (иначе ожидает 720 Гц гироскоп)
- `ARMING_CHECK 0` — отключает pre-arm проверки
- `SIM_ACCEL_RND 0`, `SIM_GYR1/2_RND 0` — отключает шум IMU
- `FS_THR_ENABLE 0` — отключает RC failsafe (иначе сразу "Disarming motors" после ARM)
- `FS_GCS_ENABLE 0` — отключает GCS failsafe
- `DISARM_DELAY 0` — отключает авто-разоружение по таймауту
- `FS_EKF_ACTION 0` — не дёргать в failsafe при плохом EKF (SITL без реального GPS)
- `INS_USE2 0`, `INS_USE3 0` — **ВАЖНО**: отключить IMU 1 и IMU 2. Bridge шлёт только один набор IMU-данных, второй IMU получает шум → "Accels inconsistent" блокирует даже FORCE ARM (это mandatory check внутри `arm()`, не обходится ARMING_CHECK=0)

---

## Протокол ArduPilot JSON SITL

### ArduPilot → Bridge (binary UDP)

**40-байтный формат** (текущий):
```
offset 0:  uint16 magic = 0x4553  ('SE')
offset 2:  uint16 frame_rate
offset 4:  uint32 frame_count
offset 8:  uint16[16] pwm  (1000-2000, 16 каналов)
```

Каналы 0-3 = моторы 1-4 (для квад-X). Значение 1000 = минимум (не крутятся). 1500+ = тяга.

**36-байтный формат** (старый, запасной):
```
offset 0:  uint16 magic = 0x4553
offset 2:  uint16 frame_rate
offset 4:  uint16[16] pwm
```

### Bridge → ArduPilot (JSON UDP)

```json
{"timestamp":1.234567,"imu":{"gyro":[0,0,0],"accel_body":[0,0,-9.81]},"position":[0,0,0],"velocity":[0,0,0],"attitude":[0,0,0]}
```

**КРИТИЧЕСКИ ВАЖНО:**
1. `timestamp` — в **СЕКУНДАХ** (не миллисекундах!)
2. `attitude` — **ОБЯЗАТЕЛЬНОЕ** поле [roll, pitch, yaw] в радианах (иначе ArduPilot отклоняет пакет)
3. JSON должен заканчиваться `\n`
4. Разделители без пробелов: `separators=(',', ':')`

Система координат: **NED** (North-East-Down). Z отрицательный = вверх.

**НЕ устанавливать GPS_TYPE=0 и EK3_GPS_TYPE=3!**
ArduPilot SITL JSON сам синтезирует GPS из данных position/velocity в JSON-ответе.
Если отключить GPS → EKF без позиции → нет тяги.

---

## Ключевые находки — история отладки

### ✅ РЕШЕНО: bridge падал с 400 Гц до 1 Гц

Три ошибки в bridge:
1. `timestamp` в миллисекундах вместо секунд
2. Нет поля `attitude` (обязательное)
3. Нет `\n` в конце JSON

### ✅ РЕШЕНО: TCP порт 5760 блокирует JSON loop

При подключении любого TCP-клиента к 5760 — bridge падает с 6000 Гц до 1 Гц.
Решение: `--serial1 udpclient:127.0.0.1:14550` и НЕ подключаться к 5760.

### ✅ РЕШЕНО: "Main loop slow (77Hz < 400Hz)"

Дефолтный SCHED_LOOP_RATE=400 — ArduPilot ждёт 400 Гц, наш ~77 Гц.
Решение: SCHED_LOOP_RATE=50 в sitl_defaults.parm + флаг `--defaults`.
SCHED_LOOP_RATE нельзя менять через PARAM_SET на лету — только при старте.

### ✅ РЕШЕНО: "Accels inconsistent" блокирует ARM

Два IMU с шумом дают расхождение акселерометров.
Решение: SIM_ACCEL_RND=0, SIM_GYR1_RND=0, SIM_GYR2_RND=0 в defaults.

### ✅ РЕШЕНО: "Disarming motors" сразу после ARM

RC failsafe срабатывал при отсутствии RC-сигнала.
Решение: FS_THR_ENABLE=0, FS_GCS_ENABLE=0, DISARM_DELAY=0 в defaults.

### ✅ РЕШЕНО: "Waiting for home" — ARM отклонялся

"EKF3 active" приходит раньше чем home установлен.
Решение: ждать "origin set" И "is using GPS" в arm_takeoff.py.

### ✅ РЕШЕНО: INS_ENABLE_MASK=1 ломал калибровку акселерометра

Устанавливали чтобы ограничить IMU — вызвало "3D Accel calibration needed" блокирующий даже FORCE ARM.
Решение: удалить INS_ENABLE_MASK из defaults.

### ✅ РЕШЕНО: MAV_CMD_PREFLIGHT_CALIBRATION перезапускал ArduPilot

Пытались вызвать чтобы исправить калибровку — ArduPilot уходил в интерактивную калибровку.
Решение: не использовать эту команду.

### ✅ РЕШЕНО: обычный ARM (без force) теперь проходит

После всех исправлений defaults — EKF полностью инициализируется, GPS здоров,
pre-arm проверки (которые обязательны даже при ARMING_CHECK=0) проходят.

### ❌ ГЛАВНАЯ НЕРЕШЁННАЯ ПРОБЛЕМА: PWM=1000 — дрон не взлетает

**Симптом**: ARM принят, "Arming motors" — но PWM в bridge всегда 1000.
Isaac Sim: Z=-0.0, дрон не двигается.

**Что пробовали (не помогло):**

1. **NAV_TAKEOFF** — result=0, но PWM=1000. Гипотеза: `ap.auto_armed=false` (без RC throttle).
2. **SET_POSITION_TARGET_LOCAL_NED** (z=-5.0) — тоже PWM=1000. Гипотеза оказалась неверной или неполной.
3. **test_motors.py v1** — STABILIZE + FORCE ARM + RC ch3=1200 override на 20 сек:
   - ARM прошёл: `Вооружён: True`
   - Режим переключился: `custom_mode=0 armed=True`
   - **Но SERVO_OUTPUT_RAW сообщений получено не было** (ни `[SERVO! РАБОТАЕТ]`, ни `[SERVO нет изм.]`)
   - Причина: `request_data_stream_send` (устаревший API) ArduPilot 4.x игнорирует
   - Вывод: **мы не знаем PWM** из-за молчащего стрима, и **не знаем видит ли ArduPilot RC ch3=1200**

**✅ ROOT CAUSE НАЙДЕНА и ИСПРАВЛЕНА**: В SITL JSON-режиме ArduPilot берёт RC-каналы **из JSON-ответа bridge** (поле `"rc"`), а НЕ из MAVLink `RC_CHANNELS_OVERRIDE`.

**ВАЖНО — правильный формат поля RC** (из SIM_JSON.h keytable):
```python
# ПРАВИЛЬНО:
"rc": {"rc_1": 1500, "rc_2": 1500, "rc_3": 1500, ..., "rc_8": 1500}

# НЕПРАВИЛЬНО (ArduPilot игнорирует!):
"rcin": [1500, 1500, 1500, ...]
```
Поле `"rcin"` в виде массива ArduPilot ТИХО ИГНОРИРУЕТ. Это стоило нескольких часов отладки.

---

### ✅ РЕШЕНО: SERVO_OUTPUT_RAW стрим не приходил

`request_data_stream_send` устарел и молча игнорируется в ArduPilot 4.x.
Исправление: `MAV_CMD_SET_MESSAGE_INTERVAL` (команда 511):
```python
conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    36, 200000, 0, 0, 0, 0, 0)  # msg_id=36 (SERVO_OUTPUT_RAW), интервал 200мс
```

---

### ✅ РЕШЕНО: PWM=1000 → landing detector блокировал тягу

**Проблема**: После ARM + GUIDED, моторы застревали на PWM=1100 (GROUND_IDLE / MOT_SPIN_ARM).
**Причина**: `ap.land_complete=True` → `set_desired_spool_state(THROTTLE_UNLIMITED)` внутри ArduPilot принудительно сбрасывается до `GROUND_IDLE`. SET_POSITION_TARGET не помогает — оно вызывает `pos_control_run()` который тоже блокируется landing detector.
**Решение**: `NAV_TAKEOFF` → `auto_takeoff_run()` → принудительно сбрасывает `land_complete=False` → THROTTLE_UNLIMITED.
**Требование**: для NAV_TAKEOFF нужен `ap.auto_armed=True` → нужен rc3=1500 в bridge JSON.

**🎉 ПЕРВЫЙ ВЗЛЁТ ПОДТВЕРЖДЁН**: После NAV_TAKEOFF моторы дошли до PWM=[1948, 1948, 1946, 1950]. Isaac Sim не принял (Jupyter TCP сервер не был запущен). ArduPilot корректно командовал взлёт.

---

### ❌ ТЕКУЩИЙ БЛОКЕР: FORCE ARM result=-4 / "Accels inconsistent"

**Симптомы** (сессия 2026-06-29 поздно):
- Normal ARM: ОТКЛОНЁН (result=-4) — rc3=1500 / throttle not neutral (хотя ARMING_CHECK=0)
- FORCE ARM: ОТКЛОНЁН (result=-4) + MSG "Arm: Accels inconsistent" + "Arm: AHRS: waiting for home"

**Причина**: `INS_USE2` не был выставлен → два IMU → второй IMU получает другие данные → `ins.healthy()` = false → это MANDATORY check внутри `arm()`, не обходится даже FORCE ARM.

**Дополнительный признак**: В T1 терминале "Loaded defaults from sitl_defaults.parm" повторялся десятки раз — был добавлен неверный параметр `EK3_CHECK_SCALE 200` (не существует в этой версии ArduPilot), вызывал цикл валидации.

**Исправления (применены, ещё не протестированы)**:
- `sitl_defaults.parm`: добавлены `INS_USE2 0`, `INS_USE3 0`; удалён `EK3_CHECK_SCALE 200`
- `arm_takeoff.py`: ARM теперь проверяет HEARTBEAT (надёжнее чем COMMAND_ACK result)

**Следующий тест**: чистый перезапуск всего → ожидается что ARM пройдёт → NAV_TAKEOFF → PWM >1400 → Isaac Sim получает сервосы → дрон взлетает.

---

### ✅ ДОКАЗАНО: физика Isaac Sim работает

Ручное управление через ячейку `manual-throttle-patch` + `manual-throttle-code` в ноутбуке:
- PWM=1500 (ХОВЕР) → дрон слегка подпрыгивает
- PWM=1800-2000 (+200 несколько раз) → дрон поднимается на 4 корпуса
- Каждый клик = импульс тяги, потом bridge сбрасывает на 1000 (от ArduPilot)

**Вывод**: Isaac физика ✅, bridge ✅, PWM→сила ✅. Проблема **только в ArduPilot** — он шлёт PWM=1000 даже после ARM.

### ❌ HOLD loop v1 завесил Jupyter kernel

Реализация через 20 TCP-соединений/сек из Python → перегрузила kernel, требовался перезапуск VS Code.

**Исправление (v2)**: HOLD через внутренний флаг в Isaac Sim:
- Патч-ячейка `manual-throttle-patch` переопределяет `_wsl_handle` — добавляет глобалы `_manual_mode` и `_manual_pwm_val`
- Когда `_manual_mode=True`, bridge-запросы НЕ перезаписывают `_wsl_servos`
- HOLD ON/OFF = один TCP-вызов, без спама
- Панель: `manual-throttle-code` — кнопки +200/+50/-50/-200/СТОП/ХОВЕР + toggle HOLD

---

## Текущее состояние arm_takeoff.py (актуальный алгоритм, 2026-06-29)

1. Запуск heartbeat thread (GCS heartbeat раз в секунду)
2. Ждём heartbeat от ArduPilot (до 60 сек)
3. Запрашиваем SERVO_OUTPUT_RAW stream через `MAV_CMD_SET_MESSAGE_INTERVAL` (msg_id=36, 200мс)
4. Устанавливаем параметры SITL через PARAM_SET
5. Ждём EKF готовности (до 30 сек): STATUSTEXT "origin set"/"is using GPS" ИЛИ HEARTBEAT status>=3
6. Ставим режим GUIDED (mode=4)
7. Пробуем обычный ARM, потом FORCE ARM; проверяем оба COMMAND_ACK (result==0) И HEARTBEAT (armed flag)
8. **Переустанавливаем GUIDED** после ARM (FORCE ARM может сбросить режим)
9. Ждём 2 сек для `auto_armed=True` (rc3=1500 в bridge обеспечивает это)
10. **NAV_TAKEOFF** altitude=5.0м → `auto_takeoff_run()` → сбрасывает land_complete → THROTTLE_UNLIMITED
11. Мониторинг 60 сек: печатаем [SERVO! ВЗЛЁТ] если PWM>1100, [ALT] при изменении высоты

**wsl_bridge.py**: rc3=1500 в поле `"rc": {"rc_1":1500,...,"rc_8":1500}` — обязательно для auto_armed

---

## Диагностика

**Проверить что physics callback работает** (ячейка Jupyter):
```python
result = await execute_in_isaac('print(_wsl_active, round(_wsl_t_ms, 1), _wsl_servos[:4])')
print(result.get("output", ""))
# Ожидается: True 123456.7 [1000, 1000, 1000, 1000]  (или >1000 после ARM+CMD)
```

**Проверить что нет зомби-процессов**:
```bash
pkill -f wsl_bridge.py; pkill -f arm_takeoff.py
```

**Проверить что Isaac Sim TCP доступен из WSL**:
```bash
WIN_IP=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')
python3 -c "
import socket, json
s = socket.socket(); s.settimeout(2)
s.connect(('$WIN_IP', 8228))
s.sendall(json.dumps({'servos':[1000]*16}).encode())
s.shutdown(1)
print(s.makefile().read())
"
```

---

## Конфигурация дрона (ячейка d617f38f / e987e0e3)

```python
DRONE_ROOT    = "/World/cf2x"
BODY_PATH     = "/World/cf2x/body"
THRUST_HEADROOM = 6.0
MAX_ROTOR_VELOCITY = 1100.0  # рад/с

omega = (pwm - 1000) / 1000.0 * MAX_ROTOR_VELOCITY   # 0..1100 рад/с
F = ROTOR_CONSTANT * omega**2                          # Ньютон
```

Масса дрона (~0.0282 кг) берётся из симуляции автоматически.

---

## Структура WSL

```
~/ardupilot/              # ArduPilot (склонирован, собран)
/mnt/c/VSCODE/ISAAC/      # Проект
  wsl_bridge.py           # Bridge скрипт
  arm_takeoff.py          # ARM + взлёт через MAVLink UDP
  sitl_defaults.parm      # Параметры ArduPilot при старте
  context_ardu.md         # Этот файл
  notebooks/
    drone_control_ardupilot.ipynb
```

Windows IP из WSL: `cat /etc/resolv.conf | grep nameserver | awk '{print $2}'`
(обычно `172.18.224.1`)

---

## Хронология сессий

| Дата | Что сделано |
|------|-------------|
| ~2026-06-25 | Исправлены 3 ошибки bridge (timestamp, attitude, \n), ARM+bridge работают |
| ~2026-06-27 | Добавлен sitl_defaults.parm, SCHED_LOOP_RATE=50, RC failsafe выключен |
| ~2026-06-26 | NAV_TAKEOFF и SET_POSITION_TARGET_LOCAL_NED — PWM=1000. Root cause: SITL JSON игнорирует RC_CHANNELS_OVERRIDE, берёт RC только из JSON. Неправильный формат `"rcin":[...]` (правильный: `"rc":{"rc_1":...}`). |
| ~2026-06-27 | Физика Isaac доказана: ручной тест PWM=1450→взлёт, 1400→стоит. HOLD v1 завесил kernel, исправлен на v2. |
| 2026-06-29 | **ПРОРЫВ**: исправлен формат RC поля (`"rc":{"rc_1":...}`), исправлен stream request (MAV_CMD_SET_MESSAGE_INTERVAL). Моторы вышли из PWM=1000 → 1100 (GROUND_IDLE). Обнаружена проблема landing detector. |
| 2026-06-29 | **ПЕРВЫЙ ВЗЛЁТ**: NAV_TAKEOFF → ArduPilot командовал PWM=1950! Isaac Sim TCP не был запущен (Jupyter), дрон не полетел физически. Доказано что вся цепочка ArduPilot работает. |
| 2026-06-29 | **Текущий блокер**: FORCE ARM result=-4, "Accels inconsistent" — из-за второго IMU. Исправлено: INS_USE2=0, INS_USE3=0 в defaults. EK3_CHECK_SCALE удалён (невалидный параметр). ARM код улучшен (HEARTBEAT проверка). Не протестировано — продолжить в следующей сессии. |
