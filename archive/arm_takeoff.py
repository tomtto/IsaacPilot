#!/usr/bin/env python3
from pymavlink import mavutil
import time, threading

ARDUPILOT_UDP = 'udpin:0.0.0.0:14550'
TAKEOFF_ALT   = 5.0  # метры (NED: z = -TAKEOFF_ALT)

conn = mavutil.mavlink_connection(ARDUPILOT_UDP, source_system=255)
print(f"Слушаем UDP {ARDUPILOT_UDP}")

def _hb_loop():
    while True:
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)
threading.Thread(target=_hb_loop, daemon=True).start()

print("Ждём heartbeat от ArduPilot...")
msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=60)
if not msg:
    print("ОШИБКА: нет heartbeat за 60 сек."); raise SystemExit(1)

sysid = conn.target_system
print(f"ArduPilot онлайн! sysid={sysid}, mode={msg.custom_mode}")

# Запрашиваем SERVO_OUTPUT_RAW (SET_MESSAGE_INTERVAL, не устаревший request_data_stream)
conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    36, 200000, 0, 0, 0, 0, 0)  # msg_id=36 (SERVO_OUTPUT_RAW), интервал 200мс

print("→ Устанавливаем параметры SITL...")
params = [
    (b'FRAME_CLASS',     1),
    (b'FRAME_TYPE',      1),
    (b'ARMING_CHECK',    0),
    (b'SCHED_LOOP_RATE', 200),
    (b'INS_FAST_SAMPLE', 0),
]
for name, val in params:
    conn.mav.param_set_send(sysid, 1, name, val, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.3)
time.sleep(1)

print("→ Ждём EKF origin set + GPS using (до 60 сек)...")
t0 = time.time()
origin_set = False
gps_using  = False
while time.time() - t0 < 60:
    m = conn.recv_match(blocking=True, timeout=1)
    if not m: continue
    if m.get_type() == 'STATUSTEXT':
        print(f"  [MSG] {m.text}")
        if 'origin set' in m.text: origin_set = True
        if 'is using GPS' in m.text: gps_using = True
    if origin_set and gps_using:
        print("  EKF + GPS готовы — ждём 3 сек для стабилизации...")
        time.sleep(3)
        break

if not (origin_set and gps_using):
    print(f"  Timeout (origin={origin_set} gps={gps_using}) — пробуем ARM всё равно")

time.sleep(1)

print("→ Ставим режим GUIDED...")
conn.mav.command_long_send(sysid, 1,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4, 0, 0, 0, 0, 0)
time.sleep(2)

armed = False
for attempt, force in [(1, False), (2, True)]:
    label = "FORCE ARM" if force else "ARM"
    print(f"→ {label} попытка {attempt} (ждём 5 сек)...")
    conn.mav.command_long_send(sysid, 1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, 21196.0 if force else 0.0,
        0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < 5:
        m = conn.recv_match(blocking=True, timeout=1)
        if not m: continue
        t = m.get_type()
        if t == 'STATUSTEXT':
            print(f"  [MSG] {m.text}")
        elif t == 'HEARTBEAT':
            if m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print(f"   {label} принят! (HEARTBEAT подтвердил)")
                armed = True
                break
        elif t == 'COMMAND_ACK' and m.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            print(f"   {label} ACK result={m.result} ({'OK' if m.result == 0 else 'FAIL'})")
            if m.result == 0:
                armed = True
            break
    if armed: break
    time.sleep(1)

if not armed:
    print("ARM не прошёл — выходим")
    raise SystemExit(1)

# После ARM явно переустанавливаем GUIDED — после FORCE ARM режим мог сброситься
time.sleep(0.5)
print("→ Переустанавливаем режим GUIDED после ARM...")
conn.mav.command_long_send(sysid, 1,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4, 0, 0, 0, 0, 0)
time.sleep(1)
# Проверяем режим
m = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
if m:
    print(f"  Режим после ARM: custom_mode={m.custom_mode} (ожидаем 4=GUIDED)")

# ── NAV_TAKEOFF: единственная команда, которая сбрасывает land_complete ──
# SET_POSITION_TARGET не работает при land_complete=True (landing detector держит GROUND_IDLE)
# NAV_TAKEOFF → auto_takeoff_run() → принудительно THROTTLE_UNLIMITED + сброс land_complete
# Требует: auto_armed=True (rc3=1500 в bridge это обеспечивает)

# Ждём чтобы auto_armed успел обновиться (несколько циклов ArduPilot)
print("→ Ждём auto_armed (2 сек)...")
time.sleep(2)

print(f"→ NAV_TAKEOFF на {TAKEOFF_ALT}м...")
conn.mav.command_long_send(
    sysid, 1,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0,       # param1-4 (минимальный угол наклона, не используется для коптера)
    0, 0,             # lat, lon (0 = текущее)
    TAKEOFF_ALT       # altitude (метры относительно home)
)

print("Слушаем 60 сек...")
t0 = time.time()
last_alt       = None
last_servo_log = 0

while time.time() - t0 < 60:
    m = conn.recv_match(blocking=True, timeout=0.1)
    if not m: continue
    t = m.get_type()

    if t == 'STATUSTEXT':
        print(f"  [MSG] {m.text}")
    elif t == 'HEARTBEAT':
        armed_now = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"  [MODE] custom_mode={m.custom_mode} armed={armed_now}  (4=GUIDED)")
    elif t == 'COMMAND_ACK':
        print(f"  [ACK] cmd={m.command} result={m.result}")
    elif t == 'SERVO_OUTPUT_RAW':
        pwm = [m.servo1_raw, m.servo2_raw, m.servo3_raw, m.servo4_raw]
        elapsed = time.time() - last_servo_log
        if elapsed > 0.3:
            if any(p > 1100 for p in pwm):
                print(f"  [SERVO! ВЗЛЁТ] PWM={pwm}")
            elif any(p != 1000 for p in pwm):
                print(f"  [SERVO idle] PWM={pwm}")
            last_servo_log = time.time()
    elif t == 'GLOBAL_POSITION_INT':
        alt = m.relative_alt / 1000.0
        if last_alt is None or abs(alt - last_alt) > 0.05:
            print(f"  [ALT] {alt:.2f}м")
            last_alt = alt
        if alt >= TAKEOFF_ALT - 0.5:
            print(f"  Достигли {alt:.1f}м!"); break

print("Готово.")
