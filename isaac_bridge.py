"""
isaac_bridge.py — весь запуск моста ArduPilot <-> Isaac Sim одной функцией.

Использование в notebook (kernel isaac_env):
    import sys; sys.path.append(r"C:\\VSCODE\\ISAAC")
    from isaac_bridge import *
    await bridge_up()      # поднять всё в Isaac (мост, физика 240Hz, Iris, колбэк Фикс-12)
    launch_sitl()          # запустить ArduPilot SITL в отдельном окне (WSL)
    await diag()           # диагностика + полётный лог
    await reset_motors()   # после Stop->Play
    stop_sitl()            # убить SITL (перед рестартом)

Рабочая конфигурация 2026-07-08 (первый успешный NAV_TAKEOFF 5м + зависание):
lockstep UDP :9002, физика 240Hz, конверсии FLU->FRD (мир Isaac: X=North, Y=West, Z=Up),
SERVO_REMAP [0,3,1,2], ROT_DIR [-1,+1,-1,+1], accel Пегаса, Iris-репспек 1.5кг.
"""

import asyncio
import json as _json
import subprocess

ISAAC_HOST = "127.0.0.1"
ISAAC_PORT = 8226  # extension isaacsim.code_editor.python_server


async def execute_in_isaac(source: str, host: str = ISAAC_HOST, port: int = ISAAC_PORT) -> dict:
    """Отправить код в открытый Isaac Sim и получить результат."""
    reader, writer = await asyncio.open_connection(host, port)
    writer.write(source.encode())
    writer.write_eof()
    data = await reader.read()
    writer.close()
    return _json.loads(data.decode())


# ============================================================================
# Шаги настройки Isaac (исполняются по порядку внутри Isaac Sim)
# ============================================================================

_SRC_SANITY = '''
import omni.usd
stage = omni.usd.get_context().get_stage()
print("Связь с Isaac: OK")

# Ищем дрон по ИМЕНИ прима — путь может быть любым (/World/cf2x, /Root/cf2x, ...)
DRONE_ROOT = None
for _p in stage.Traverse():
    if _p.GetName() == "cf2x":
        DRONE_ROOT = str(_p.GetPath())
        break
if DRONE_ROOT is None:
    raise RuntimeError("Прим 'cf2x' не найден в сцене — добавь дрона и запусти снова")
print("Дрон найден:", DRONE_ROOT)
'''

_SRC_DRIVE = '''
import omni.usd
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()
_joint_names = ["m1_joint", "m2_joint", "m3_joint", "m4_joint"]
_found = 0
for _p in stage.Traverse():
    if _p.GetName() in _joint_names and str(_p.GetPath()).startswith(DRONE_ROOT):
        drive = UsdPhysics.DriveAPI.Apply(_p, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateTargetVelocityAttr(0.0)
        drive.CreateDampingAttr(0.0)
        drive.CreateStiffnessAttr(0.0)
        _found += 1
if _found != 4:
    raise RuntimeError("Найдено суставов пропеллеров: %d из 4 (под %s)" % (_found, DRONE_ROOT))
print("Drive снят — суставы пропеллеров свободны (4/4)")
'''

_SRC_CONFIG = '''
# DRONE_ROOT уже найден на шаге 1. Ищем body и пропеллеры по именам под ним.
import omni.physx
import omni.usd
from pxr import PhysicsSchemaTools, Sdf, UsdGeom, Usd, UsdPhysics
import carb

stage = omni.usd.get_context().get_stage()

_prop_names = ["m1_prop", "m2_prop", "m3_prop", "m4_prop"]
_props = {}
BODY_PATH = None
for _p in stage.Traverse():
    _path = str(_p.GetPath())
    if not _path.startswith(DRONE_ROOT):
        continue
    if _p.GetName() in _prop_names:
        _props[_p.GetName()] = _path
    elif _p.GetName() == "body" and BODY_PATH is None:
        BODY_PATH = _path
if BODY_PATH is None or len(_props) != 4:
    raise RuntimeError("Не найдены body/пропеллеры под %s (props: %s, body: %s)"
                       % (DRONE_ROOT, sorted(_props), BODY_PATH))
PROP_PATHS = [_props[n] for n in _prop_names]
print("body:", BODY_PATH)

MASS_OVERRIDE_KG = None
GRAVITY = 9.81
THRUST_HEADROOM = 6.0

stage_id = omni.usd.get_context().get_stage_id()
sim_iface = omni.physx.get_physx_simulation_interface()
physx_iface = omni.physx.get_physx_interface()
BODY_ID = PhysicsSchemaTools.sdfPathToInt(Sdf.Path(BODY_PATH))

def _get_total_mass():
    if MASS_OVERRIDE_KG is not None:
        return MASS_OVERRIDE_KG
    total = 0.0
    for p in [BODY_PATH] + PROP_PATHS:
        prim = stage.GetPrimAtPath(p)
        m = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
        total += m if m else 0.0
    return total

DRONE_MASS_KG = _get_total_mass()
HOVER_FORCE_TOTAL_N = DRONE_MASS_KG * GRAVITY
HOVER_FORCE_PER_MOTOR_N = HOVER_FORCE_TOTAL_N / 4
MAX_FORCE_PER_MOTOR_N = HOVER_FORCE_PER_MOTOR_N * THRUST_HEADROOM

print(f"Масса (до Iris-репспека): {DRONE_MASS_KG:.4f} кг")
'''

_SRC_GEOM = '''
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, UsdPhysics

MAX_ROTOR_VELOCITY = 1100.0

PROP_WORLD_POS = [
    UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    for path in PROP_PATHS
]
PROP_CENTROID_XY = (
    sum(p[0] for p in PROP_WORLD_POS) / 4,
    sum(p[1] for p in PROP_WORLD_POS) / 4,
)
PROP_OFFSETS_XY = [(p[0] - PROP_CENTROID_XY[0], p[1] - PROP_CENTROID_XY[1]) for p in PROP_WORLD_POS]
ROT_DIR = [+1, -1, +1, -1]
ROTOR_CONSTANT = MAX_FORCE_PER_MOTOR_N / (MAX_ROTOR_VELOCITY ** 2)
ROLLING_MOMENT_COEFF = ROTOR_CONSTANT * 0.117
ROTOR_CONSTANTS = [ROTOR_CONSTANT] * 4
ROLLING_MOMENT_COEFFS = [ROLLING_MOMENT_COEFF] * 4

def read_state():
    """Позиция, ориентация, скорость и угловая скорость тела дрона."""
    body_prim = stage.GetPrimAtPath(BODY_PATH)
    transform = UsdGeom.Xformable(body_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = transform.ExtractTranslation()
    quat = transform.ExtractRotationQuat()
    imag = quat.GetImaginary()
    R = Rotation.from_quat([imag[0], imag[1], imag[2], quat.GetReal()])
    rb = UsdPhysics.RigidBodyAPI(body_prim)
    v = rb.GetVelocityAttr().Get()
    w_deg = rb.GetAngularVelocityAttr().Get()
    w = np.radians([w_deg[0], w_deg[1], w_deg[2]])
    return np.array([pos[0], pos[1], pos[2]]), R, np.array([v[0], v[1], v[2]]), w

print("Геометрия и read_state() готовы")
'''

_SRC_PHYS240 = '''
from pxr import UsdPhysics, PhysxSchema, Sdf

_scene_prim = None
for _p in stage.Traverse():
    if _p.IsA(UsdPhysics.Scene):
        _scene_prim = _p
        break

if _scene_prim is None:
    _scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
    _scene_prim = _scene.GetPrim()
    print("PhysicsScene не было — создан", _scene_prim.GetPath())

_ps_api = PhysxSchema.PhysxSceneAPI.Apply(_scene_prim)
_ps_api.CreateTimeStepsPerSecondAttr().Set(240)
print("Физика:", _ps_api.GetTimeStepsPerSecondAttr().Get(), "Hz")
'''

_SRC_PLAY = '''
import omni.timeline
omni.timeline.get_timeline_interface().play()
print("Симуляция запущена (Play)")
'''

_SRC_INFRA = '''
import threading
import numpy as np

_MAX_VEL = globals().get("MAX_ROTOR_VELOCITY", 1100.0)
def _pwm_to_vel(pwm):
    return max(0.0, (max(1000, min(2000, pwm)) - 1000) / 1000.0 * _MAX_VEL)

_missing = [g for g in ["ROTOR_CONSTANTS","PROP_OFFSETS_XY","ROT_DIR","ROLLING_MOMENT_COEFFS",
                         "read_state","stage","sim_iface","physx_iface","BODY_PATH","BODY_ID"]
            if g not in globals()]
if _missing:
    raise RuntimeError("Не определены: " + str(_missing))

_wsl_servos = [1000] * 16
_wsl_state  = {
    "imu":        {"gyro": [0, 0, 0], "accel_body": [0, 0, -9.81]},
    "position":   [0.0, 0.0, 0.0],
    "velocity":   [0.0, 0.0, 0.0],
    "quaternion": [1.0, 0.0, 0.0, 0.0],
}
_wsl_t_ms   = 0.0
_wsl_lock   = threading.Lock()
_wsl_active = True

try:
    _timeline_sub.unsubscribe()
except Exception:
    pass

print("Инфраструктура OK, _wsl_active =", _wsl_active)
'''

# Фикс-12: lockstep UDP + конверсии FLU->FRD + accel Пегаса + броня + watchdog
_SRC_CALLBACK = '''
import socket, struct, json, math
import numpy as np
from scipy.spatial.transform import Rotation as _R

# === Фикс-12: исправленный motor-map (лево-право было зеркально) ===
SERVO_REMAP = [0, 3, 1, 2]   # rotor i <- AP servo[SERVO_REMAP[i]]; роторы: FR, RR, RL, FL
ROT_DIR = [-1, +1, -1, +1]   # FR=CCW, RR=CW, RL=CCW, FL=CW (переопределяет геометрию)
AP_JSON_PORT = 9002
_SERVO_MAGIC = 18458  # 0x47FA
_WATCHDOG_MS = 1000.0
_GRAVITY_WORLD = np.array([0.0, 0.0, -9.81])

def _clamp(v, lo, hi):
    return max(lo, min(hi, float(v)))

try:
    _ap_udp_sock.close()
except Exception:
    pass
_ap_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_ap_udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
_ap_udp_sock.bind(("0.0.0.0", AP_JSON_PORT))
_ap_udp_sock.setblocking(False)
try:
    _ap_udp_sock.ioctl(-1744830452, False)  # SIO_UDP_CONNRESET (Windows-квирк)
except Exception:
    pass

_ap_addr = None
_ap_pkt_count = 0
_ap_reply_count = 0
_ap_last_pkt_t_ms = -1e9
_ap_watchdog_fired = False
_prev_v_world = np.array([0.0, 0.0, 0.0])
_cb_error_count = 0
_cb_last_error = ""
_last_good_state = {
    "imu":        {"gyro": [0.0, 0.0, 0.0], "accel_body": [0.0, 0.0, -9.81]},
    "position":   [0.0, 0.0, 0.0],
    "velocity":   [0.0, 0.0, 0.0],
    "quaternion": [1.0, 0.0, 0.0, 0.0],
}
_wsl_log = []
_wsl_log_last_t = -1e9
_LOG_INTERVAL_MS = 50.0
_LOG_MAX_ENTRIES = 6000

def _parse_servos(data):
    if len(data) >= 40 and struct.unpack_from("<H", data, 0)[0] == _SERVO_MAGIC:
        return list(struct.unpack_from("<16H", data, 8))
    if len(data) >= 36 and struct.unpack_from("<H", data, 0)[0] == _SERVO_MAGIC:
        return list(struct.unpack_from("<16H", data, 4))
    return None

def _on_physics_step_wsl(dt):
    global _wsl_state, _wsl_t_ms, _ap_addr, _ap_pkt_count, _ap_reply_count
    global _ap_last_pkt_t_ms, _ap_watchdog_fired, _wsl_log_last_t
    global _prev_v_world, _cb_error_count, _cb_last_error, _last_good_state
    if dt <= 0 or not _wsl_active: return
    _wsl_t_ms += dt * 1000.0

    got_packet = False
    for _i in range(64):
        try:
            data, addr = _ap_udp_sock.recvfrom(4096)
        except (BlockingIOError, OSError):
            break
        s = _parse_servos(data)
        if s is not None:
            _wsl_servos[:] = s
            _ap_addr = addr
            got_packet = True
            _ap_pkt_count += 1
    if got_packet:
        _ap_last_pkt_t_ms = _wsl_t_ms
        _ap_watchdog_fired = False

    if (_ap_pkt_count > 0 and not _ap_watchdog_fired
            and _wsl_t_ms - _ap_last_pkt_t_ms > _WATCHDOG_MS):
        _wsl_servos[:] = [1000] * 16
        _ap_watchdog_fired = True
        print("[WATCHDOG] SITL молчит > " + str(int(_WATCHDOG_MS)) + "мс — моторы выключены")

    servos = list(_wsl_servos)

    try:
        mv = [_pwm_to_vel(servos[SERVO_REMAP[i]]) for i in range(4)]
        fn = [ROTOR_CONSTANTS[i] * mv[i]**2 for i in range(4)]
        total_thrust = sum(fn)
        tau_roll  = sum( PROP_OFFSETS_XY[i][1] * fn[i] for i in range(4))
        tau_pitch = sum(-PROP_OFFSETS_XY[i][0] * fn[i] for i in range(4))
        tau_yaw   = sum(ROLLING_MOMENT_COEFFS[i] * mv[i]**2 * ROT_DIR[i] for i in range(4))
        body_prim = stage.GetPrimAtPath(BODY_PATH)
        xf = UsdGeom.Xformable(body_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        up = xf.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
        bp = xf.ExtractTranslation()
        sim_iface.apply_force_at_pos(
            stage_id, BODY_ID,
            carb.Float3(float(up[0]*total_thrust), float(up[1]*total_thrust), float(up[2]*total_thrust)),
            carb.Float3(float(bp[0]), float(bp[1]), float(bp[2])), "Force")
        q_usd = xf.ExtractRotationQuat()
        im = q_usd.GetImaginary()
        q_sc = _R.from_quat([im[0], im[1], im[2], q_usd.GetReal()])
        Rm = q_sc.as_matrix()
        tw = Rm @ np.array([tau_roll, tau_pitch, tau_yaw])
        sim_iface.apply_torque(stage_id, BODY_ID, carb.Float3(float(tw[0]), float(tw[1]), float(tw[2])))

        _, R, v, w_world = read_state()
        w = Rm.T @ np.array([w_world[0], w_world[1], w_world[2]])

        a_true_world = (v - _prev_v_world) / dt
        _prev_v_world = v.copy()
        f_world = a_true_world - _GRAVITY_WORLD
        f_body = Rm.T @ f_world

        # === Фикс-12: конверсия FLU->FRD (мир Isaac: X=North, Y=West, Z=Up) ===
        qn = q_sc.as_quat()
        new_state = {
            "imu": {
                "gyro":       [_clamp(w[0],-50,50),        _clamp(-w[1],-50,50),       _clamp(-w[2],-50,50)],
                "accel_body": [_clamp(f_body[0],-50,50),   _clamp(-f_body[1],-50,50),  _clamp(-f_body[2],-50,50)],
            },
            "position":   [_clamp(bp[0],-1e4,1e4),  _clamp(-bp[1],-1e4,1e4),  _clamp(-bp[2],-1e4,1e4)],
            "velocity":   [_clamp(v[0],-200,200),    _clamp(-v[1],-200,200),    _clamp(-v[2],-200,200)],
            "quaternion": [float(qn[3]), float(qn[0]), float(-qn[1]), float(-qn[2])],
        }
        _vals = (new_state["position"] + new_state["velocity"] + new_state["quaternion"]
                 + new_state["imu"]["gyro"] + new_state["imu"]["accel_body"])
        if not all(math.isfinite(x) for x in _vals):
            raise ValueError("non-finite value in state")
        _last_good_state = new_state
    except Exception as e:
        _cb_error_count += 1
        _cb_last_error = repr(e)
        new_state = _last_good_state
        total_thrust = 0.0
        up = None

    with _wsl_lock:
        _wsl_state = new_state

    if got_packet and _ap_addr is not None:
        _rc3 = 1500 if any(p > 1000 for p in servos[:4]) else 1100
        resp = {
            "timestamp":  _wsl_t_ms / 1000.0,
            "imu":        new_state["imu"],
            "position":   new_state["position"],
            "velocity":   new_state["velocity"],
            "quaternion": new_state["quaternion"],
            "rc": {"rc_1":1500,"rc_2":1500,"rc_3":_rc3,"rc_4":1500,
                   "rc_5":1500,"rc_6":1500,"rc_7":1500,"rc_8":1500},
        }
        try:
            _ap_udp_sock.sendto((json.dumps(resp, separators=(",", ":")) + chr(10)).encode(), _ap_addr)
            _ap_reply_count += 1
        except Exception:
            pass

    if (up is not None and _wsl_t_ms - _wsl_log_last_t >= _LOG_INTERVAL_MS
            and len(_wsl_log) < _LOG_MAX_ENTRIES):
        _wsl_log_last_t = _wsl_t_ms
        tilt_deg = float(np.degrees(np.arccos(max(-1.0, min(1.0, up[2])))))
        _wsl_log.append({
            "t":        round(_wsl_t_ms / 1000.0, 2),
            "dt_ms":    round(dt * 1000.0, 2),
            "alt":      round(-bp[2], 3),
            "x":        round(bp[0], 3),
            "y":        round(bp[1], 3),
            "thrust":   round(total_thrust, 3),
            "accel_z":  round(float(new_state["imu"]["accel_body"][2]), 2),
            "tilt_deg": round(tilt_deg, 1),
            "gyro_deg": [round(float(np.degrees(x)), 1) for x in w],
            "pwm":      servos[:4],
        })

try: _ap_subscription.unsubscribe()
except: pass
_ap_subscription = physx_iface.subscribe_physics_step_events(_on_physics_step_wsl)
print("Колбэк Фикс-12 активен — lockstep UDP :" + str(AP_JSON_PORT))
'''

# Фикс-11: репспек в Iris (проверенная пара из Pegasus)
_SRC_IRIS = '''
import numpy as np
from pxr import UsdPhysics, Gf

IRIS_MASS_KG        = 1.5
IRIS_INERTIA        = (0.029125, 0.029125, 0.055225)  # кг*м^2 (Ixx, Iyy, Izz)
IRIS_ARM_X          = 0.13
IRIS_ARM_Y          = 0.21
IRIS_ROTOR_CONSTANT = 8.54858e-6
IRIS_ROLLING_COEFF  = 1e-6

body_prim = stage.GetPrimAtPath(BODY_PATH)
_mass_api = UsdPhysics.MassAPI.Apply(body_prim)
_mass_api.CreateMassAttr().Set(IRIS_MASS_KG)
_mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*IRIS_INERTIA))
_mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

DRONE_MASS_KG = IRIS_MASS_KG
MASS_OVERRIDE_KG = IRIS_MASS_KG
HOVER_FORCE_TOTAL_N = DRONE_MASS_KG * GRAVITY
MAX_FORCE_PER_MOTOR_N = IRIS_ROTOR_CONSTANT * (MAX_ROTOR_VELOCITY ** 2)
ROTOR_CONSTANT = IRIS_ROTOR_CONSTANT
ROLLING_MOMENT_COEFF = IRIS_ROLLING_COEFF
ROTOR_CONSTANTS = [ROTOR_CONSTANT] * 4
ROLLING_MOMENT_COEFFS = [ROLLING_MOMENT_COEFF] * 4
PROP_OFFSETS_XY = [( IRIS_ARM_X, -IRIS_ARM_Y),
                   (-IRIS_ARM_X, -IRIS_ARM_Y),
                   (-IRIS_ARM_X,  IRIS_ARM_Y),
                   ( IRIS_ARM_X,  IRIS_ARM_Y)]

print("Iris-репспек: масса {} кг, запас тяги {:.2f}x".format(
    _mass_api.GetMassAttr().Get(), 4*MAX_FORCE_PER_MOTOR_N/HOVER_FORCE_TOTAL_N))
'''

_STEPS = [
    ("1/9 Связь и дрон",          _SRC_SANITY),
    ("2/9 Drive off",             _SRC_DRIVE),
    ("3/9 Конфигурация",          _SRC_CONFIG),
    ("4/9 Геометрия",             _SRC_GEOM),
    ("5/9 Физика 240Hz",          _SRC_PHYS240),
    ("6/9 Play",                  _SRC_PLAY),
    ("7/9 Инфраструктура",        _SRC_INFRA),
    ("8/9 Колбэк Фикс-12",        _SRC_CALLBACK),
    ("9/9 Iris-репспек",          _SRC_IRIS),
]


async def bridge_up():
    """Поднять весь мост в Isaac Sim одной командой (безопасно перезапускать)."""
    for name, src in _STEPS:
        r = await execute_in_isaac(src)
        out = (r.get("output") or "").strip()
        tb = r.get("traceback") or ""
        status = r.get("status", "?")
        print(f"[{name}] {status}: {out}")
        if status != "ok" or tb:
            print(tb)
            print(">>> ОСТАНОВЛЕНО — исправь ошибку и запусти bridge_up() снова")
            return False
    print()
    print("=== МОСТ ГОТОВ. Дальше: launch_sitl(), потом полётные задания ===")
    return True


async def diag():
    """Диагностика: счётчики lockstep, watchdog, ошибки, полётный лог."""
    r = await execute_in_isaac('''
print("_wsl_active =", _wsl_active)
print("Пакетов от ArduPilot:", _ap_pkt_count, "  Ответов Isaac:", _ap_reply_count)
print("Адрес SITL:", _ap_addr)
_last = globals().get("_ap_last_pkt_t_ms", None)
if _last is not None and _ap_pkt_count > 0:
    print("Возраст последнего пакета: {:.2f} сек сим-времени".format((_wsl_t_ms - _last) / 1000.0))
    print("Watchdog сработал:", globals().get("_ap_watchdog_fired", "-"))
print("Ошибок колбэка:", globals().get("_cb_error_count", "-"), " Последняя:", globals().get("_cb_last_error", ""))
print("Всего записей лога:", len(_wsl_log))
flight = [e for e in _wsl_log if any(p > 1000 for p in e["pwm"])]
print("Полётных записей (PWM>1000):", len(flight))
lines = []
for e in flight:
    lines.append(
        "t={:7.2f}s dt={:5.2f}ms alt={:+6.2f} x={:+5.2f} y={:+5.2f} thr={:.3f}N az={:+6.2f} tilt={:5.1f}deg gyro={} pwm={}".format(
            e["t"], e["dt_ms"], e["alt"], e["x"], e["y"], e["thrust"], e.get("accel_z", 0.0),
            e["tilt_deg"], e["gyro_deg"], e["pwm"]
        )
    )
print(chr(10).join(lines))
''')
    print(r.get("output", ""))
    print(r.get("traceback", ""))


async def reset_motors():
    """После Stop->Play в Isaac: серво в ноль, мост включён, лог очищен."""
    r = await execute_in_isaac(
        '_wsl_servos[:] = [1000] * 16\n'
        '_wsl_active = True\n'
        '_wsl_log[:] = []\n'
        'print("servos=1000, _wsl_active=True, лог очищен")'
    )
    print(r.get("output", ""))


# ============================================================================
# SITL (запуск WSL-процесса из Windows, отдельное консольное окно)
# ============================================================================

# Команду передаём через скрипт-файл: кавычки/$-подстановки НЕ переживают
# границу Windows->wsl.exe->bash (проверено: WIN_IP приходил пустым)
_SITL_SCRIPT_WIN = r"C:\VSCODE\ISAAC\run_sitl.sh"
_SITL_SCRIPT_WSL = "/mnt/c/VSCODE/ISAAC/run_sitl.sh"

_SITL_SCRIPT = """#!/bin/bash
# Окружение как в интерактивном терминале (venv-ardupilot из install-prereqs)
source "$HOME/.profile" 2>/dev/null || true
if [ -f "$HOME/venv-ardupilot/bin/activate" ]; then
    source "$HOME/venv-ardupilot/bin/activate"
fi

WIN_IP=$(grep nameserver /etc/resolv.conf | awk '{print $2}')
if [ -z "$WIN_IP" ]; then
    echo "ОШИБКА: не смог определить IP Windows из /etc/resolv.conf"
    read -r -p "Enter чтобы закрыть..."
    exit 1
fi
echo "=== SITL (sim_vehicle): физика JSON -> $WIN_IP:9002, MAVLink -> $WIN_IP:14550 ==="

cd "$HOME/ardupilot" || exit 1
exec python3 Tools/autotest/sim_vehicle.py -v ArduCopter \\
    --model "JSON:$WIN_IP" \\
    --add-param-file="$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm" \\
    --add-param-file=/mnt/c/VSCODE/ISAAC/sitl_defaults.parm \\
    --no-rebuild \\
    --out "udp:$WIN_IP:14550"
"""


def launch_sitl():
    """Запустить ArduPilot SITL в отдельном окне. Isaac-мост должен быть уже поднят.

    MAVLink летит на Windows UDP:14550 (нужно одноразовое правило firewall).
    """
    with open(_SITL_SCRIPT_WIN, "w", newline="\n", encoding="utf-8") as f:
        f.write(_SITL_SCRIPT)
    p = subprocess.Popen(
        ["wsl.exe", "bash", _SITL_SCRIPT_WSL],
        creationflags=subprocess.CREATE_NEW_CONSOLE,
    )
    print("SITL запускается в отдельном окне (PID wsl:", p.pid, ")")
    print("В окне должно быть: 'физика JSON -> 172.x.x.x:9002' (IP НЕ пустой!)")
    print("Через ~5 сек проверь: await diag() — счётчик пакетов должен расти.")
    return p


def stop_sitl():
    """Убить процесс arducopter в WSL (перед рестартом после краша)."""
    subprocess.run(["wsl.exe", "pkill", "-f", "arducopter"])
    print("SITL остановлен (pkill arducopter)")
