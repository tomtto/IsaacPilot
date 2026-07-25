# IsaacPilot — ArduPilot SITL + Isaac Sim 6 Lockstep Bridge

Bridge between **ArduPilot SITL** and **NVIDIA Isaac Sim 6.0** for quadcopter simulation. Uses lockstep UDP transport for deterministic physics at 240Hz, with a custom FLU→FRD frame conversion and Iris drone respec (1.5kg).

## 🎥 Launch walkthrough

[![IsaacPilot — Isaac Sim + ArduPilot SITL bridge: launch & flight](https://img.youtube.com/vi/CJ-g0ms-7XA/maxresdefault.jpg)](https://youtu.be/CJ-g0ms-7XA)

*Launching the bridge and a first flight — Isaac Sim ↔ ArduPilot SITL, straight from Jupyter notebooks.*

## ✅ Status

- **NAV_TAKEOFF 5m + stable hover** — first successful flight (2026-07-08)
- **Full warehouse mission** — takeoff 1.7m → 2×2m square → land+disarm (2026-07-09)
- **Scene-independent** — works with any USD scene (finds `cf2x` by name, supports non-zero spawn)

## 🏗️ Architecture

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

### Key Technical Details

| Component | Value |
|---|---|
| Transport | Lockstep UDP :9002 (not TCP) |
| Physics | 240Hz (PhysxSceneAPI.TimeStepsPerSecond) |
| Drone | Iris respec: 1.5kg, inertia (0.029125, 0.029125, 0.055225) kg·m² |
| Rotor constant | 8.54858e-6, rolling moment coeff 1e-6 |
| Motor arms | X=±0.13m, Y=±0.21m |
| Servo remap | `[0, 3, 1, 2]` |
| Rotation dirs | `[-1, +1, -1, +1]` (FR=CCW, RR=CW, RL=CCW, FL=CW) |
| Magic number | 18458 (0x47FA) |
| Isaac world frame | X=North, Y=**West** (not East!), Z=Up |

### FLU→FRD Conversion (Fix-12)

Y-component sign flip on ALL quantities:
```python
gyro:       [wx, -wy, -wz]
accel_body: [fx, -fy, -fz]    # Pegasus accel (true acceleration - gravity)
position:   [bp[0], -bp[1], -bp[2]]
velocity:   [vx, -vy, -vz]
quaternion: [qw, qx, -qy, -qz]  # scipy [x,y,z,w] → AP [w,x,y,z]
```

## 🚀 Quick Start

### Prerequisites

1. **Windows 10/11** with WSL2 (Ubuntu)
2. **Isaac Sim 6.0** installed via pip in a venv (`isaac_env`), plus `pip install ipykernel pymavlink` in that venv (`pymavlink` powers the flight commands in `flight_missions.ipynb`)
3. **ArduPilot SITL** built in WSL (`~/ardupilot/build/sitl/bin/arducopter`)
4. **Firewall rules** (PowerShell as Admin, once):
   ```powershell
   New-NetFirewallRule -DisplayName "ArduPilot JSON UDP 9002" -Direction Inbound -Protocol UDP -LocalPort 9002 -Action Allow
   New-NetFirewallRule -DisplayName "ArduPilot MAVLink UDP 14550" -Direction Inbound -Protocol UDP -LocalPort 14550 -Action Allow
   ```

### Running

1. Open Isaac Sim with a scene containing a `cf2x` drone — the drone model ships in `assets/cf2x/cf2x.usd`; add it to any stage (**File → Import** or drag it in) and name the prim `cf2x`. Step-by-step in [INSTALL.md](INSTALL.md) §6.2.
2. Enable extension: `isaacsim.code_editor.python_server` (port 8226)
3. Open `notebooks/drone_control_ardupilot.ipynb` (kernel: `isaac_env`):
   ```python
   await bridge_up()    # 9 steps: setup bridge, physics, Iris respec
   launch_sitl()        # starts ArduPilot SITL in a separate window
   await diag()         # packet counters should grow, dt≈4.17ms
   ```
4. Open `notebooks/flight_missions.ipynb` (kernel: `isaac_env`):
   ```python
   # Run "Connect" cell, then "Commands" cell
   arm_and_takeoff(5)   # ARM + takeoff to 5m
   goto(10, 0, 5)       # fly 10m north
   land()               # land + disarm
   ```

### After a crash

```
Stop → Play in Isaac → await reset_motors() → stop_sitl() → launch_sitl()
```

## 📁 Project Structure

```
IsaacPilot/
├── isaac_bridge.py                     # Main bridge module (all Isaac-side code)
├── sitl_defaults.parm                  # ArduPilot SITL parameters
├── README.md          / README.ru.md
├── INSTALL.md         / INSTALL.ru.md
├── TROUBLESHOOTING.md / TROUBLESHOOTING.ru.md
├── .gitignore
├── notebooks/
│   ├── drone_control_ardupilot.ipynb   # Bridge setup + SITL launch
│   └── flight_missions.ipynb           # MAVLink missions
└── assets/
    └── cf2x/                           # Drone model (USD)
```

## 🔧 SITL Parameters (`sitl_defaults.parm`)

| Parameter | Value | Why |
|---|---|---|
| `FRAME_CLASS` | 1 | Quadcopter X |
| `FRAME_TYPE` | 1 | X frame |
| `SCHED_LOOP_RATE` | 200 | Match 240Hz physics (ARM may need 2-3 retries) |
| `ARMING_CHECK` | 0 | Disable pre-arm checks |
| `INS_USE2` | 0 | Disable 2nd IMU (bridge sends one IMU set) |
| `INS_USE3` | 0 | Disable 3rd IMU |
| `MOT_THST_HOVER` | 0.35 | Hover thrust estimate |
| `FS_THR_ENABLE` | 0 | Disable RC failsafe |
| `FS_GCS_ENABLE` | 0 | Disable GCS failsafe |
| `DISARM_DELAY` | 0 | Disable auto-disarm |

## 📜 License

MIT — see [LICENSE](LICENSE). The lockstep bridge in this repository is free to use.

The reactive-perception and swarm-coordination layer shown in the demo video is a separate **proprietary** core and is **not** part of this repository (available on request).