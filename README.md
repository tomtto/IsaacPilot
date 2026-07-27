# IsaacPilot — ArduPilot SITL + Isaac Sim 6 Lockstep Bridge

Bridge between **ArduPilot SITL** and **NVIDIA Isaac Sim 6.0** for quadcopter simulation. Uses lockstep UDP transport for deterministic physics at 240Hz, with a custom FLU→FRD frame conversion and Iris drone respec (1.5kg).

## 🐝 Built on this bridge

On top of this open lockstep bridge we built an **autonomous 2-drone security swarm** — a patrol drone spots an intruder with onboard YOLO and hands the coordinates to a guard drone, fully autonomously:

[![Autonomous 2-drone security swarm](https://img.youtube.com/vi/xz-7rIDq75M/maxresdefault.jpg)](https://youtu.be/xz-7rIDq75M)

*3 consecutive live runs, unedited (only subtitles added).* The swarm logic shown here — reactive perception, a maneuver library, a mission dispatcher and swarm coordination — is a separate proprietary core, built on top of this open bridge and written from scratch instead of ROS2. It's under active development and continually being improved. **This repository is the open bridge it runs on.**

## 🧠 Synthetic perception (next artifact)

The worker/intruder detection in the demo runs on a YOLO model trained on **fully synthetic data** — generated in Isaac Sim through a custom Replicator (GUI) pipeline, tuned for quality: domain-randomized scenes → KITTI → YOLO, with a full engineering log of the fixes that took the hit-rate from ~8% to ~82%. This dataset pipeline is a **separate open artifact, released as the next step after deeper testing** — so you'll be able to generate your own dataset with your own classes.

## 👤 About this project

IsaacPilot is a hobby project, built end-to-end by one person, in the evenings. That's a framing, not a disclaimer — the whole stack here (physics bridge, SITL protocol, frame math, perception, coordination) was designed and debugged by a single developer, so what you get is small, readable and hackable rather than a heavy framework.

It also means this is **not an enterprise product**: there's no support SLA, issues may sit for a while, and the API can change. Treat it as a working reference and a starting point, not a turnkey dependency.

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

## 💡 Three things that weren't in any docs

- Isaac's world frame is **Y=West**, not East — found by a deliberate sign-test.
- ArduPilot's EKF needs **specific force** (accel − gravity), not raw body acceleration.
- The JSON SITL protocol is **lockstep** — any TCP middleman silently kills it.

## ⚙️ Engineering decisions & trade-offs

- **A custom lockstep bridge, not an existing framework.** The JSON SITL protocol is lockstep — one physics reply per step. A thin UDP callback inside Isaac's physics loop keeps that guarantee with no middleman; a heavier layer would only add latency that breaks it.
- **ArduPilot SITL instead of a hand-written controller.** After 60+ iterations of a custom stabilizer, the project switched to ArduPilot — the same code that flies on real hardware (Matek H743). Reusing a proven autopilot beats re-deriving flight control in a sim.
- **Written from scratch instead of ROS2.** ROS2 is the right tool for complex multi-robot systems; for this focused sim-to-real bridge it would be overkill — more moving parts, less transparency. The lightweight core keeps the whole pipeline in a few readable files.

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

## 🎥 Launch walkthrough

[![IsaacPilot — Isaac Sim + ArduPilot SITL bridge: launch & flight](https://img.youtube.com/vi/CJ-g0ms-7XA/maxresdefault.jpg)](https://youtu.be/CJ-g0ms-7XA)

*Launching the bridge and a first flight — Isaac Sim ↔ ArduPilot SITL, straight from Jupyter notebooks.*

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

The reactive-perception and swarm-coordination layer is a separate **proprietary** core, built on top of this bridge and under active development. It is not part of this repository.

## 📬 Contact

- Email: swarmbotlab@gmail.com
- YouTube: [@swarmbotlab](https://www.youtube.com/@swarmbotlab)

For collaboration or a pilot project — feel free to get in touch.
