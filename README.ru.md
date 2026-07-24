# IsaacPilot — мост Lockstep между ArduPilot SITL и Isaac Sim 6

Мост между **ArduPilot SITL** и **NVIDIA Isaac Sim 6.0** для симуляции квадрокоптера. Использует lockstep-транспорт по UDP для детерминированной физики на 240Hz, с собственной конверсией фреймов FLU→FRD и репспеком под дрон Iris (1.5кг).

## ✅ Статус

- **NAV_TAKEOFF 5м + стабильное зависание** — первый успешный полёт (2026-07-08)
- **Полная складская миссия** — взлёт 1.7м → квадрат 2×2м → посадка+disarm (2026-07-09)
- **Независимость от сцены** — работает с любой USD-сценой (находит `cf2x` по имени, поддерживает ненулевой спавн)

## 🏗️ Архитектура

```
Notebook 1 (drone_control_ardupilot.ipynb)
  bridge_up()  → Isaac Sim (TCP :8226)
    9 steps: find drone → drive off → config → geometry → 240Hz physics →
             play → lockstep infra → Fix-12 callback (UDP :9002) → Iris respec
  launch_sitl() → WSL (sim_vehicle.py)
    SITL: JSON physics → Windows:9002, MAVLink → Windows:14550

Notebook 2 (flight_missions.ipynb)
  MAVLink ← UDP :14550 (direct from SITL, no MavProxy)
  arm_and_takeoff(5) → goto(10,0,5) → land()
```

### Ключевые технические детали

| Компонент | Значение |
|---|---|
| Транспорт | Lockstep UDP :9002 (не TCP) |
| Физика | 240Hz (PhysxSceneAPI.TimeStepsPerSecond) |
| Дрон | Iris-репспек: 1.5кг, инерция (0.029125, 0.029125, 0.055225) кг·м² |
| Константа ротора | 8.54858e-6, коэффициент rolling moment 1e-6 |
| Плечи моторов | X=±0.13м, Y=±0.21м |
| Servo remap | `[0, 3, 1, 2]` |
| Направления вращения | `[-1, +1, -1, +1]` (FR=CCW, RR=CW, RL=CCW, FL=CW) |
| Magic number | 18458 (0x47FA) |
| Мировой фрейм Isaac | X=North, Y=**West** (не East!), Z=Up |

### Конверсия FLU→FRD (Fix-12)

Смена знака Y-компоненты у ВСЕХ величин:
```python
gyro:       [wx, -wy, -wz]
accel_body: [fx, -fy, -fz]    # Pegasus accel (true acceleration - gravity)
position:   [bp[0], -bp[1], -bp[2]]
velocity:   [vx, -vy, -vz]
quaternion: [qw, qx, -qy, -qz]  # scipy [x,y,z,w] → AP [w,x,y,z]
```

## 🚀 Быстрый старт

### Требования

1. **Windows 10/11** с WSL2 (Ubuntu)
2. **Isaac Sim 6.0**, установленный через pip в venv (`isaac_env`), плюс `pip install ipykernel pymavlink` в этот venv (`pymavlink` обеспечивает команды полёта в `flight_missions.ipynb`)
3. **ArduPilot SITL**, собранный в WSL (`~/ardupilot/build/sitl/bin/arducopter`)
4. **Правила файрвола** (PowerShell от администратора, один раз):
   ```powershell
   New-NetFirewallRule -DisplayName "ArduPilot JSON UDP 9002" -Direction Inbound -Protocol UDP -LocalPort 9002 -Action Allow
   New-NetFirewallRule -DisplayName "ArduPilot MAVLink UDP 14550" -Direction Inbound -Protocol UDP -LocalPort 14550 -Action Allow
   ```

### Запуск

1. Открой Isaac Sim со сценой, содержащей дрон `cf2x` — модель дрона лежит в `assets/cf2x/cf2x.usd`; добавь её в любой стейдж (**File → Import** или перетаскиванием) и назови прим `cf2x`. Пошагово — [INSTALL.ru.md](INSTALL.ru.md) §6.2.
2. Включи экстеншен: `isaacsim.code_editor.python_server` (порт 8226)
3. Открой `notebooks/drone_control_ardupilot.ipynb` (kernel: `isaac_env`):
   ```python
   await bridge_up()    # 9 steps: setup bridge, physics, Iris respec
   launch_sitl()        # starts ArduPilot SITL in a separate window
   await diag()         # packet counters should grow, dt≈4.17ms
   ```
4. Открой `notebooks/flight_missions.ipynb` (kernel: `isaac_env`):
   ```python
   # Выполни ячейку "Connect", затем "Commands"
   arm_and_takeoff(5)   # ARM + takeoff to 5m
   goto(10, 0, 5)       # fly 10m north
   land()               # land + disarm
   ```

### После краша

```
Stop → Play in Isaac → await reset_motors() → stop_sitl() → launch_sitl()
```

## 📁 Структура проекта

```
IsaacPilot/
├── isaac_bridge.py                     # Основной модуль моста (весь код на стороне Isaac)
├── sitl_defaults.parm                  # Параметры ArduPilot SITL
├── README.md          / README.ru.md
├── INSTALL.md         / INSTALL.ru.md
├── TROUBLESHOOTING.md / TROUBLESHOOTING.ru.md
├── .gitignore
├── notebooks/
│   ├── drone_control_ardupilot.ipynb   # Настройка моста + запуск SITL
│   └── flight_missions.ipynb           # MAVLink-миссии
└── assets/
    └── cf2x/                           # Модель дрона (USD)
```

## 🔧 Параметры SITL (`sitl_defaults.parm`)

| Параметр | Значение | Зачем |
|---|---|---|
| `FRAME_CLASS` | 1 | Квадрокоптер X |
| `FRAME_TYPE` | 1 | Рама X |
| `SCHED_LOOP_RATE` | 200 | Под 240Hz физику (ARM может потребовать 2-3 повтора) |
| `ARMING_CHECK` | 0 | Отключить pre-arm проверки |
| `INS_USE2` | 0 | Отключить 2-й IMU (мост шлёт один набор IMU) |
| `INS_USE3` | 0 | Отключить 3-й IMU |
| `MOT_THST_HOVER` | 0.35 | Оценка тяги зависания |
| `FS_THR_ENABLE` | 0 | Отключить RC failsafe |
| `FS_GCS_ENABLE` | 0 | Отключить GCS failsafe |
| `DISARM_DELAY` | 0 | Отключить авто-disarm |

## 📜 Лицензия

MIT — см. [LICENSE](LICENSE). Lockstep-мост в этом репозитории свободен в использовании.

Слой реактивной перцепции и роевой координации, показанный в демо-видео, — отдельное **проприетарное** ядро, в этот репозиторий **не входит** (доступно по запросу).
