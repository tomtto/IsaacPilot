# IsaacPilot — Troubleshooting & development history

This file is a collection of all the real problems the project ran into on the way from the first spinning propeller to a fully autonomous takeoff/mission/landing. If you are reproducing this bridge on your own machine and hit a similar error, there's a good chance it is already covered here.

It is a filtered digest — every experiment (including dead ends) distilled down to what will actually help another person.

---

## Solved problems (ArduPilot SITL ↔ Isaac Sim bridge)

### 1. TCP transport broke lockstep synchronization
**Symptom:** the physics behaved strangely (unstable oscillations, loss of thrust) on the first attempts to connect ArduPilot SITL over TCP.
**Cause:** the ArduPilot JSON SITL protocol is **lockstep**: ArduPilot sends PWM, stops its internal clock, and waits for ONE JSON physics reply per one of its own steps. TCP introduced latency/buffering, so replies did not arrive strictly one-per-step — ArduPilot's internal clock drifted apart from Isaac Sim's actual physics steps.
**Fix:** switch to lockstep **UDP :9002** with a callback right inside Isaac Sim's physics loop (`subscribe_physics_step_events`) — exactly one reply per physics step, with no buffering and no intermediary process.

### 2. Wrong magic number in the servo packet → PWM always 1000
**Symptom:** ArduPilot accepts the connection, but in every incoming packet PWM = 1000 (the minimum); the motors never spin up.
**Cause:** the code used the magic number `0x4553` (from an earlier version of the bridge), while the actual ArduPilot SITL JSON protocol (`SIM_JSON.h`) expects `18458` (`0x47FA`).
**Fix:** `_SERVO_MAGIC = 18458` in the packet parser (`isaac_bridge.py`), checking both possible packet formats (40 and 36 bytes) against this magic number.

### 3. PWM = 1000 even after a successful ARM
**Symptom:** ARM succeeds, GUIDED mode is set, but the servo channels still don't move.
**Cause:** in SITL JSON mode ArduPilot takes the RC channels **from the bridge's own JSON reply** (the `"rc"` field), not from the MAVLink `RC_CHANNELS_OVERRIDE`. We were sending `"rcin": [1500, ...]` (an array) — ArduPilot **silently ignores** this format.
**Fix:** the correct format is an object with named fields:
```python
"rc": {"rc_1": 1500, "rc_2": 1500, "rc_3": 1500, ..., "rc_8": 1500}
```
This cost several hours of debugging — the error gives no signal at all, nothing simply happens.

### 4. SERVO_OUTPUT_RAW never arrives (can't diagnose PWM)
**Symptom:** the `SERVO_OUTPUT_RAW` stream (msg_id 36) is requested, but the messages don't arrive at all.
**Cause:** the deprecated `request_data_stream_send` was used — ArduPilot 4.x silently ignores it.
**Fix:** `MAV_CMD_SET_MESSAGE_INTERVAL` (command 511):
```python
conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    36, 200000, 0, 0, 0, 0, 0)  # msg_id=36 (SERVO_OUTPUT_RAW), interval 200ms
```

### 5. Motors stuck at PWM≈1100 (GROUND_IDLE), no thrust
**Symptom:** after ARM the PWM is just above the minimum (~1100) but doesn't rise, even on position commands.
**Cause:** `ap.land_complete=True` inside ArduPilot forces the motor state to `GROUND_IDLE` — a plain `SET_POSITION_TARGET` can't override it, because it too is blocked by the same landing detector.
**Fix:** the **NAV_TAKEOFF** command — it explicitly invokes `auto_takeoff_run()`, which clears `land_complete=False` and moves the motors to `THROTTLE_UNLIMITED`. It requires `ap.auto_armed=True`, which needs `rc3=1500` in the JSON `"rc"` field.

### 6. FORCE ARM rejected: "Accels inconsistent"
**Symptom:** both a normal ARM and a FORCE ARM are rejected with this message, despite `ARMING_CHECK=0`.
**Cause:** ArduPilot emulates **multiple IMUs** by default. The bridge sends data for only one IMU set — the second one is left without updates / with noise, and `ins.healthy()` returns false. This is a **mandatory check** inside `arm()` that `ARMING_CHECK=0` does not bypass.
**Fix:** in `sitl_defaults.parm` — `INS_USE2 0`, `INS_USE3 0` (disable the second/third IMU).

### 7. SITL loops on "Loaded defaults from sitl_defaults.parm"
**Symptom:** at SITL startup the same defaults-loading line repeats dozens of times in the terminal.
**Cause:** the parameter file contained a parameter invalid for this ArduPilot version (`EK3_CHECK_SCALE`) — the parameter validator can't apply it and restarts the validation loop.
**Fix:** remove non-existent/invalid parameters from `sitl_defaults.parm`, keeping only the ones actually supported.

### 8. "Disarming motors" right after a successful ARM
**Symptom:** ARM is confirmed, but a second or two later the autopilot disarms itself.
**Cause:** RC failsafe triggers when there is no continuous RC signal (in SITL, with no real transmitter, there won't be one).
**Fix:** in the defaults — `FS_THR_ENABLE 0`, `FS_GCS_ENABLE 0`, `DISARM_DELAY 0`.

### 9. ARM rejected: "waiting for home" / "Need Position Estimate"
**Symptom:** ARM fails with these messages even though telemetry seems to be flowing.
**Cause:** the EKF doesn't set its origin (the position reference point) immediately after SITL startup, but only ~10-20 seconds later. If you try to ARM sooner, the EKF isn't ready yet.
**Fix:** explicitly wait, before ARM, for the messages `EKF3 IMU0 origin set` and `EKF3 IMU0 is using GPS` (or the corresponding `LOCAL_POSITION_NED` fields) — the `wait_ekf()` function in the project.

### 10. `INS_ENABLE_MASK` breaks accelerometer calibration
**Symptom:** after setting this parameter (in an attempt to limit the set of active IMUs) a blocking "3D Accel calibration needed" error appears, not bypassable even with FORCE ARM.
**Fix:** don't use `INS_ENABLE_MASK` for this purpose — the correct path is `INS_USE2`/`INS_USE3` (see problem 6).

### 11. `MAV_CMD_PREFLIGHT_CALIBRATION` drives ArduPilot into interactive calibration
**Symptom:** after sending this command (in an attempt to fix calibration issues) ArduPilot enters a state that waits for manual calibration and stops responding normally.
**Fix:** don't call this command in an automated flow at all.

### 12. Coordinate system: spinning top and growing EKF variance
**Symptom:** the drone spins up in yaw and gradually drifts away by tens of meters; the EKF shows growing variance.
**Cause:** it was initially assumed that the Isaac Sim world uses a standard ENU/NED-like orientation with **Y=East**. A sign test (deliberately apply a known deviation and see where the drone actually goes) revealed that in the scene used, **Y=West**, not East.
**Fix:** "Fix-12" — a full FLU→FRD conversion with an explicit sign flip of the Y component for ALL quantities (gyro, accelerometer, position, velocity, quaternion), plus `SERVO_REMAP=[0,3,1,2]` and `ROT_DIR=[-1,+1,-1,+1]` for motor mapping. Details in `README.md`, the "FLU→FRD Conversion" section.

### 13. EKF is "blind" — altitude always reads 0
**Symptom:** telemetry flows, the EKF initializes, but the computed altitude doesn't move at all.
**Cause:** the bridge's JSON reply sent "raw" acceleration from the physics engine, rather than what an accelerometer actually measures (specific force = real acceleration minus gravity). Mathematically, the EKF can't tell free fall from rest if it receives incorrect accel data.
**Fix:** compute `accel_body` correctly as the true accelerometer measurement (subtract gravity from the physical body's acceleration), rather than taking the raw center-of-mass acceleration.

### 14. A 50 Hz control loop — too slow for stability
**Symptom:** the drone is unstable even with generally correct settings.
**Cause:** Isaac Sim physics was computed at a low rate, insufficient for stable stabilization of a multicopter with this dynamics.
**Fix:** physics raised to **240 Hz** (`PhysxSceneAPI.TimeStepsPerSecond`).

### 15. PID gains tuned for a 28-gram Crazyflie tear a heavier drone apart
**Symptom:** oscillation/over-gain when using parameters originally tuned for a light drone.
**Cause:** ArduPilot's controller gains and the physical body's mass/inertia didn't match each other — with a light body, the same gains become overly aggressive.
**Fix:** the "Iris respec" — the physical body's mass and inertia in Isaac Sim are brought to those of a real Iris-class drone (mass 1.5 kg, inertia `(0.029125, 0.029125, 0.055225)` kg·m²), which ArduPilot's default parameters are calibrated for out of the box.

### 16. Quotes don't survive the Windows → wsl.exe → bash transition
**Symptom:** an inline command like `wsl bash -c "... $(cat /etc/resolv.conf | awk '{print $2}') ..."` run from Windows (PowerShell / Python `subprocess`) produced an **empty** `$WIN_IP` — SITL sent packets into the void.
**Cause:** escaping of quotes and substitutions doesn't survive the double transition between shells (Windows → `wsl.exe` → bash inside) — some special characters are interpreted by the wrong layer.
**Fix:** build a full `.sh` script on disk (necessarily with `newline='\n'`, not Windows `\r\n`!) and run it as a file (`wsl.exe bash /mnt/c/.../run_sitl.sh`) rather than as an inline string — escaping within a single file of a single system behaves predictably.

### 17. Connecting to TCP 5760 completely stalls the JSON loop
**Symptom:** as soon as any TCP client (e.g. `telnet`/MAVProxy) connected to port 5760, the lockstep packet rate dropped from thousands of Hz to ~1 Hz.
**Fix:** don't connect to 5760 at all — all physics and telemetry exchange goes over UDP (`:9002` physics, `:14550` MAVLink); this TCP port is not needed for the project.

### 18. ARM doesn't pass on the first try (usually cycle 2-3)
**Symptom:** the first ARM attempt is rejected by checks like "Gyro 0 rate 240Hz < loop rate*1.8" or "Main loop slow".
**Cause:** Isaac Sim physics runs at 240 Hz, while ArduPilot's `SCHED_LOOP_RATE` is set to 200 — right on the boundary between two internal scheduler checks, so the first few ARM cycles formally fail validation even though the system is working fine.
**Fix (in use):** simply retry ARM 2-3 times — this is normal behavior with the current configuration, not a bug. **A possible cleaner improvement (unverified):** lower `SCHED_LOOP_RATE` to ~130, so that `130×1.8=234<240` satisfies both checks and ARM passes on the first try.

### 19. The notebook hangs when restarting the missions kernel
**Symptom:** after `Restart Kernel` in the notebook with the flight commands, the new session hangs on connecting.
**Cause:** the old kernel still holds the UDP socket `14550` open, waiting for a heartbeat; the port is busy.
**Fix:** wait for the old kernel to fully stop before starting a new one (or explicitly restart both notebooks together when in doubt).

### 20. `goto()` cuts the corners of the route
**Symptom:** when flying a rectangular/square route, the turns are cut by about half a meter rather than passing exactly through the target point.
**Cause:** a point is considered reached when entering the `tolerance` radius (default `0.7`) — this is the expected "fly-by along an arc" behavior, not a positioning bug.
**Fix:** for precise corners, pass a smaller `tolerance` (e.g. `0.3`) to the `goto()` call.

### 21. The bridge can't find the drone in a new scene
**Symptom:** on a new USD scene (a different warehouse layout) the script fails with "can't see the drone" even though the drone is visually present in the scene.
**Cause:** the drone path was hardcoded (`/World/cf2x`) — in the new scene the drone's root is named differently (`/Root/cf2x`), and the drone is not at the world origin.
**Fix:** find the prim **by name** `cf2x` via `stage.Traverse()` (not by a fixed path); likewise find the joints/propellers by a name pattern (`m*_joint`, `m*_prop`, `body`) relative to the found root. A non-zero spawn is not a problem — ArduPilot takes home from the point of first arming, not from the world origin.

### 22. A scaled drone prim takes off fine, then flips — scale corrupts the reported attitude
**Symptom:** the drone prim is scaled up in the scene (`Scale 5` on the root) to make a palm-sized quad visible on video. Takeoff completes normally ("altitude 4.70 m"), then within a couple of seconds the drone drifts sideways and tumbles. The flight log shows motors fighting each other: `pwm=[1825, 1470, 1950, 1150]` — some at the upper rail, some near idle.

**Ruled out first:** mass and inertia are intact (PhysX reports exactly the authored 1.5 kg and the Iris inertia — the prim scale does not touch explicit mass properties); the motor arms used for torque are constants, independent of scene geometry; the collider grows, but that doesn't affect control.

**Cause:** `read_state()` took the orientation from the **world matrix** via `ExtractRotationQuat()`. That function is only valid for a matrix **without scale/shear**. With a scale in the matrix the resulting quaternion is not proportional to the true rotation, so after normalization it is a *different* rotation:

| real tilt | reported to ArduPilot (Scale 5) |
|---|---|
| 2° | 5.0° |
| 10° | 24.6° |
| 30° | 66.3° |

The error is exactly zero at level attitude — which is why takeoff looks perfect. Then: the drone tips 2° → the autopilot sees 5° → it corrects twice as hard as needed → the real tilt grows → the reported tilt grows faster → divergence and a flip within seconds. A larger scale makes it worse (at Scale 6, 10° is reported as 27°).

The same call appears a second time inside the physics callback, where it is worse still: that matrix rotates the body-frame torque into world axes **and** converts the angular velocity into body axes (the gyro sent to ArduPilot). So attitude, torque and gyro were all wrong at once.

**Fix:** strip the scale before extracting the rotation, in both places:
```python
quat = transform.RemoveScaleShear().ExtractRotationQuat()
```
Verified numerically: at `Scale 1` the result is unchanged (no regression), at `Scale 5/6` the angles come back exact. After the fix a 5× scaled drone hovers with a tilt of 0.06° and 0.012 m/s residual velocity.

**Not affected, and why:** the thrust direction (`xf.TransformDir(...).GetNormalized()` — normalized, so the direction survives), the position (a uniform scale doesn't distort the translation), and the velocities (read from `RigidBodyAPI`, not from the matrix).

**Rule of thumb:** any matrix from `ComputeLocalToWorldTransform()` may carry the prim's scale. If you extract a **rotation** from it, call `RemoveScaleShear()` first. For directions, `TransformDir()` + `GetNormalized()` is safe.

---

## Historical note: the abandoned approach (Step 1 of the plan)

Before switching to ArduPilot SITL, the project went through **60+ iterations** of trying to write its own geometric stabilization controller (inspired by Pegasus Simulator / Lee-Mellinger) directly on Isaac Sim's `omni.physx` API — with the goal of simply confirming that the `apply_force_at_pos` / `read_state()` combination works reliably at all, before trusting an external autopilot with it.

The controller was never brought to a stable hover — the true cause of the instability (after ruling out gains, gyroscopics, collisions, force-application physics, damping coordinate systems and a dozen other hypotheses) remained not fully confirmed at the point of moving to Step 2. The project deliberately **gave up on its own stabilization math** in favor of the proven ArduPilot autopilot — the very same code will fly on real hardware (Matek H743), unlike a one-off simulation controller.

---

## Where else to look if your problem isn't described here

- `README.md` — the `sitl_defaults.parm` parameter table and a brief "Problem and fix timeline" table
- `INSTALL.md` — if the problem is at the environment-setup stage (WSL2, Isaac Sim, Jupyter kernel) rather than the bridge itself
