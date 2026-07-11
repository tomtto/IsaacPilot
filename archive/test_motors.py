#!/usr/bin/env python3
"""
Минимальный тест вывода моторов.
STABILIZE mode + RC override ch3=1200 — в этом режиме газ напрямую идёт на моторы.
Если PWM поднимется выше 1000 — ArduPilot вывод работает.
Если нет — фундаментальная проблема в ArduPilot или конфигурации.
"""
from pymavlink import mavutil
import time, threading

conn = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=255)

def _hb():
    while True:
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)
threading.Thread(target=_hb, daemon=True).start()

print("Ждём heartbeat...")
msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=30)
if not msg: raise SystemExit("Нет heartbeat за 30 сек")
sysid = conn.target_system
print(f"Подключились sysid={sysid}  mode={msg.custom_mode}")

# Запрашиваем потоки через SET_MESSAGE_INTERVAL (request_data_stream устарел, ArduPilot 4.x его игнорирует)
# SERVO_OUTPUT_RAW = message ID 36, 10 Hz
conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    36, 100000, 0, 0, 0, 0, 0)
# RC_CHANNELS = message ID 65, 5 Hz  — чтобы видеть дошёл ли RC override до ArduPilot
conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    65, 200000, 0, 0, 0, 0, 0)
time.sleep(0.3)

# STABILIZE mode (0) — газ RC напрямую на моторы, без EKF/GPS
print("→ STABILIZE (mode=0)...")
conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, 0, 0, 0, 0, 0)
time.sleep(1)

# Проверяем что режим переключился
m = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
if m:
    print(f"  Текущий режим: {m.custom_mode} (ожидаем 0=STABILIZE)")

# FORCE ARM
print("→ FORCE ARM...")
conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
    1, 21196, 0, 0, 0, 0, 0)
t0 = time.time()
armed = False
while time.time() - t0 < 5:
    m = conn.recv_match(blocking=True, timeout=1)
    if not m: continue
    if m.get_type() == 'STATUSTEXT': print(f"  [MSG] {m.text}")
    if m.get_type() == 'COMMAND_ACK' and m.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        print(f"  ARM ACK result={m.result} ({'OK' if m.result == 0 else 'FAIL'})")
        armed = m.result == 0
        break
    if m.get_type() == 'HEARTBEAT' and (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True
        break

print(f"  Вооружён: {armed}")

# RC override: ch3=1200 (20% газ — моторы крутятся но дрон не взлетит)
# Если хочешь взлёт — поставь 1550
THROTTLE_TEST = 1200

print(f"\n→ RC override ch3={THROTTLE_TEST} на 20 сек...")
print("  Смотри в bridge-терминале: должно появиться PWM > 1000")
print("  [SERVO! РАБОТАЕТ] = вывод ArduPilot ОК")
print("  Если только [1000,1000,1000,1000] — проблема в ArduPilot")
print()

t0 = time.time()
last_servo_print = 0
last_hb_mode = None

while time.time() - t0 < 20:
    # Шлём RC override каждые 100мс
    conn.mav.rc_channels_override_send(
        sysid, 1,
        65535, 65535, THROTTLE_TEST, 65535,
        65535, 65535, 65535, 65535)

    m = conn.recv_match(blocking=True, timeout=0.1)
    if not m: continue
    t = m.get_type()

    if t == 'STATUSTEXT':
        print(f"  [MSG] {m.text}")
    elif t == 'HEARTBEAT':
        mode = m.custom_mode
        armed_now = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if mode != last_hb_mode:
            print(f"  [MODE] custom_mode={mode} armed={armed_now}")
            last_hb_mode = mode
    elif t == 'RC_CHANNELS':
        now = time.time()
        if now - last_servo_print > 1.0:
            print(f"  [RC] ch1={m.chan1_raw} ch2={m.chan2_raw} ch3={m.chan3_raw} ch4={m.chan4_raw}  ← ch3 должен быть {THROTTLE_TEST}")
    elif t == 'SERVO_OUTPUT_RAW':
        pwm = [m.servo1_raw, m.servo2_raw, m.servo3_raw, m.servo4_raw]
        now = time.time()
        if any(p != 1000 for p in pwm):
            if now - last_servo_print > 0.3:
                elapsed = now - t0
                print(f"  [SERVO! РАБОТАЕТ ✓] t={elapsed:.1f}s PWM={pwm}")
                last_servo_print = now
        else:
            if now - last_servo_print > 2.0:
                elapsed = now - t0
                print(f"  [SERVO нет изм.] t={elapsed:.1f}s PWM={pwm}")
                last_servo_print = now

# Сбрасываем RC override (ch3=1000)
conn.mav.rc_channels_override_send(sysid, 1, 65535, 65535, 1000, 65535, 65535, 65535, 65535, 65535)
print("\nТест завершён. RC override сброшен.")
