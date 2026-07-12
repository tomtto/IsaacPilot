---
name: isaac-ardupilot-bridge
description: ArduPilot SITL + Isaac Sim bridge project — ✅ ЦЕЛЬ ДОСТИГНУТА 2026-07-08: NAV_TAKEOFF 5м + зависание; новый флоу «2 notebooks, ноль терминалов»
metadata: 
  node_type: memory
  type: project
  originSessionId: 34b1f7f8-5ea0-4611-923a-166032127249
---

## ✅ МИССИЯ ВЫПОЛНЕНА (2026-07-08)

**NAV_TAKEOFF 5м + стабильное зависание** — первый успешный полёт в рамках проекта.

```
t=103.29s alt=-4.99 x=-0.21 y=+0.31 thr=14.759N tilt=0.1deg gyro=[-0.2, 0.2, 0.3]
t=103.39s alt=-4.99 x=-0.22 y=+0.32 thr=14.734N tilt=0.2deg gyro=[ 0.4, 0.2, 0.3]
```
Высота стабильна 4.99м (цель 5.0), наклон 0.1°, угловые скорости — доли °/с, ни одной ошибки EKF за весь полёт. EKF3 in-flight yaw alignment complete.

## Рабочая конфигурация — НЕ ЛОМАТЬ

**Сцена**: `C:\VSCODE\ISAAC\scenes\first_scene_isaac02.usd`

**Физика Isaac Sim**:
- 240Hz (PhysxSceneAPI.TimeStepsPerSecond)
- Iris-репспек: масса 1.5кг, инерция (0.029125, 0.029125, 0.055225) кг·м²
- ROTOR_CONSTANT = 8.54858e-6, ROLLING_MOMENT_COEFF = 1e-6
- Плечи роторов: X=±0.13м, Y=±0.21м

**Фрейм мира Isaac Sim**: X=North, Y=**West** (НЕ East!), Z=Up
- Найдено тестом знаков — критическая находка

**Конверсия FLU→FRD (Фикс-12)**:
```python
SERVO_REMAP = [0, 3, 1, 2]   # rotor i <- AP servo[SERVO_REMAP[i]]
ROT_DIR = [-1, +1, -1, +1]   # FR=CCW, RR=CW, RL=CCW, FL=CW
# Y-компонента меняет знак у ВСЕХ величин:
gyro:       [wx, -wy, -wz]
accel_body: [fx, -fy, -fz]   # accel Пегаса (реальное ускорение - гравитация)
position:   [bp[0], -bp[1], -bp[2]]   # bp = xf.ExtractTranslation() (мировые координаты!)
velocity:   [vx, -vy, -vz]
quaternion: [qw, qx, -qy, -qz]        # scipy [x,y,z,w] → AP [w,x,y,z]
```

**Транспорт**: lockstep UDP :9002 (не TCP), `_SERVO_MAGIC = 18458 (0x47FA)`

**Запуск SITL**:
```bash
WIN_IP=$(grep nameserver /etc/resolv.conf | awk '{print $2}')
$HOME/ardupilot/build/sitl/bin/arducopter \
  --model JSON:$WIN_IP \
  --defaults $HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm,/mnt/c/VSCODE/ISAAC/sitl_defaults.parm \
  -I0 --serial0 udpclient:$WIN_IP:14550
```

## Новый флоу «2 notebooks, ноль терминалов»

**Статус: ✅ ПРОВЕРЕН ПОЛНЫМ ПРОГОНОМ 2026-07-09** — автономный взлёт на 1.7м и зависание ВНУТРИ склада (новая сцена), точность удержания north=0.02/east=-0.02м. Багфиксы в ходе проверки:
1. **Кавычки не переживают Windows→wsl.exe→bash**: инлайн-команда с `awk '{print $2}'` давала ПУСТОЙ $WIN_IP (SITL слал в никуда). Фикс: `launch_sitl()` пишет `C:\VSCODE\ISAAC\run_sitl.sh` (newline='\n'!) и запускает `wsl.exe bash /mnt/c/.../run_sitl.sh` — экранирование не пересекает границу систем.
2. **SITL запускается через sim_vehicle** (по требованию пользователя — открывает знакомое окно ArduCopter для диагностики), не через голый arducopter. Скрипт активирует `~/.profile` + `venv-ardupilot`, MavProxy получает `--out udp:$WIN_IP:14550` (ручной `output add` больше не нужен).
3. **wait_ekf() перед ARM**: без него ARM падал с "Need Position Estimate"/"waiting for home" (EKF ставит origin через ~10-20с после старта SITL; в старом arm_takeoff.py ожидание было, при переносе в notebook потерялось). Ждём появления LOCAL_POSITION_NED.
4. **arm() с 3 циклами ретраев**: ARM проходит не с первой попытки (чеки "Gyro 0 rate 240Hz < loop rate*1.8 360Hz" и "Main loop slow 160<200" — следствие физики 240Hz при SCHED_LOOP_RATE 200), обычно добивает 2-3 цикл. Опциональная полировка на будущее: SCHED_LOOP_RATE ~130 удовлетворит оба чека (130*1.8=234<240) и ARM станет чистым с первой попытки — НЕ делали, не проверено.
5. Зависание нового kernel полётных заданий: если ячейка «Подключение» перезапущена пока старая ждёт heartbeat — сокет 14550 занят; лечится Restart kernel.

Откат на крайний случай: `notebooks/drone_control_ardupilot_WORKING_BACKUP.ipynb` (дословная копия notebook победного полёта 2026-07-08).

**✅ Задание 2 тоже пройдено (2026-07-09)**: полная миссия внутри склада — взлёт 1.7м → квадрат 2×2м по 4 точкам `goto()` → `land()` с disarm. Углы срезаются на ~0.6м из-за `tolerance=0.7` в goto (точка засчитывается при входе в радиус) — для точных углов передавать `tolerance=0.3`. **Весь стек миссий работает: Шаг 2 плана закрыт полностью, фундамент Шага 3 (оркестратор) проверен.**

## GitHub: перенос проекта — ЗАВЕРШЁН (2026-07-11)

Приватный репозиторий **github.com/tomtto/IsaacPilot** — **push сделан успешно**, актуальное состояние на GitHub. История: пользователь сначала дал задачу Cline/GLM-5.2 — тот создал приличные README.md/.gitignore/archive/, но его `git add -A` НЕ исключал `scenes/` (4.5 ГБ) и раздул `.git` до 3.3 ГБ при нуле коммитов. Тот раздутый `.git` снесён.

**Каноническая рабочая папка для git теперь — `C:\VSCODE\ISAACPILOT`** (НЕ `C:\VSCODE\ISAAC`!). Создана заново 2026-07-11 как чистая копия 16 файлов, отслеживаемых git (скопированы вручную по списку `git ls-files` из старого `ISAAC/`, без scenes/PegasusSimulator/isaac_env). Собственный `git init`, коммит `a85f56f` (16 файлов, 708К, крупнейший `assets/cf2x/cf2x.usd` 216КБ). git есть только в WSL (2.53.0), на Windows не установлен — все команды через `wsl bash -c "cd /mnt/c/VSCODE/ISAACPILOT && git ..."`.

**Push выполнен (2026-07-11)**: `git remote add origin https://github.com/tomtto/IsaacPilot.git && git push -u origin main --force` — force понадобился, т.к. на GitHub уже была авто-заглушка (коммит `96202f0`, только .gitignore+README, не связанная история). Push прошёл, `main` отслеживает `origin/main`. Токен создан как classic PAT без срока действия (Note: `IsaacPilot-push`, scope `repo`), пользователь входит через Google-аккаунт (без обычного пароля GitHub) — при следующих push тоже нужно будет использовать PAT как пароль, не аккаунт-пароль.

**Важно на будущее**: `C:\VSCODE\ISAAC\` (старая папка, коммит `f925c67`, remote не настроен) и `C:\VSCODE\ISAACPILOT\` (новая, запушенная) сейчас существуют ОБЕ и разошлись. Рабочий код (isaac_bridge.py, notebooks) физически остаётся в `ISAAC/` — там же лежит `isaac_env/`, `scenes/`, `PegasusSimulator/` (последнюю пользователь планирует убрать позже, ещё не сделано). `ISAACPILOT/` — только git-зеркало для GitHub. При будущих изменениях кода в `ISAAC/` нужно будет вручную копировать изменённые файлы в `ISAACPILOT/` перед коммитом/push (единого источника правды пока нет — не автоматизировано).

**Мост сцено-независим (2026-07-09)**: пользователь создал новую сцену-склад `scenes/save_scene/scene_02_sklad.usd`, где дрон по пути `/Root/cf2x` (не `/World/cf2x`) и стоит не в нуле (-2.72, -8.56, 0). Первый прогон падал на «не видит дрона» — исправлено в `isaac_bridge.py`: шаг 1 ищет прим по ИМЕНИ `cf2x` через `stage.Traverse()` (любой путь), drive/config ищут `m*_joint`/`m*_prop`/`body` по именам под найденным DRONE_ROOT, PhysicsScene ищется/создаётся по типу. Ненулевой спавн — не проблема (home ArduPilot = точка включения, `goto()` считается от неё). Если в сцене два прима `cf2x` — возьмётся первый найденный.

Весь код мост вынесен в `C:\VSCODE\ISAAC\isaac_bridge.py`. Терминалы WSL не нужны.

**Notebook 1**: `C:\VSCODE\ISAAC\notebooks\drone_control_ardupilot.ipynb`
```python
from isaac_bridge import *
await bridge_up()    # 9 шагов: Drive off → Config → Geom → 240Hz → Play → Infra → Fix-12 → Iris
launch_sitl()        # запускает arducopter в отдельном окне через wsl.exe
await diag()         # пакеты от ArduPilot должны расти, dt≈4.17мс
# После краша: Stop→Play в Isaac → await reset_motors() → stop_sitl() → launch_sitl()
```

**Notebook 2**: `C:\VSCODE\ISAAC\notebooks\flight_missions.ipynb`
```python
# MAVLink → UDP 14550 → Windows напрямую (pymavlink в isaac_env)
arm_and_takeoff(5)           # ARM + взлёт на 5м
goto(10, 0, 5)               # лететь 10м на север
land()                       # посадка + disarm
where()                      # текущие координаты
monitor(30)                  # телеметрия 30 сек
```

**Firewall (один раз от Admin)**:
```powershell
New-NetFirewallRule -DisplayName "ArduPilot JSON UDP 9002" -Direction Inbound -Protocol UDP -LocalPort 9002 -Action Allow
New-NetFirewallRule -DisplayName "ArduPilot MAVLink UDP 14550" -Direction Inbound -Protocol UDP -LocalPort 14550 -Action Allow
```

## Установка Isaac Sim (рабочий рецепт)

1. Windows LongPaths: `Set-ItemProperty -Path 'HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem' -Name 'LongPathsEnabled' -Value 1`
2. `pip install isaacsim[all,extscache]==6.0.1.0` в venv `C:\VSCODE\ISAAC\isaac_env` (Python 3.12.7)
3. Extension `isaacsim.code_editor.python_server` (порт 8226) — включить в Isaac Sim
4. `isaacsim.code_editor.jupyter` — держать ВЫКЛЮЧЕННЫМ (сломано в 6.0.1.0)

## Хронология проблем и фиксов

| Проблема | Фикс |
|----------|------|
| Magic number 0x4553 вместо 0x47FA → всегда PWM=1000 | Фикс magic |
| TCP-транспорт (latency, таймаут) → ArduPilot не ждёт | Lockstep UDP :9002 |
| FPE / зомби-полёты | Броня + watchdog |
| EKF слепой (altitude 0 навсегда) | accel Пегаса (реальное ускорение) |
| 50Hz контур → слишком медленно | 240Hz физика |
| Y=East (неверно) → волчок, EKF variance | Тест знаков → Y=West, Фикс-12 |
| PID под 28г → перегейн → раскачка | Iris-репспек 1.5кг |

## Следующий шаг — MAVLink-оркестратор

`flight_missions.ipynb` уже написан и готов. Цель: команды GUIDED «лети в точку», посадка, полные миссии. Этот же код без изменений поедет на реальный Matek H743.

**Why:** Не терять конфиг между сессиями.
**How to apply:** Начинать с `bridge_up()` → `launch_sitl()` → flight_missions.
