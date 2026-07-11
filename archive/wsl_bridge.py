#!/usr/bin/env python3
"""
WSL ArduPilot ↔ Isaac Sim bridge.

Ключевой принцип: отвечаем ArduPilot НЕМЕДЛЕННО (с кешированным стейтом),
потом обновляем Isaac Sim. ArduPilot не ждёт TCP-вызов.

Запуск:
    python3 /mnt/c/VSCODE/ISAAC/wsl_bridge.py
"""

import socket
import json
import struct
import time
import sys
import threading

# ── Windows IP ──
try:
    with open("/etc/resolv.conf") as f:
        WIN_IP = [l.split()[1] for l in f if l.startswith("nameserver")][0].strip()
except Exception:
    WIN_IP = "172.18.224.1"

ISAAC_PORT = 8228
AP_PORT    = 9002

print("WSL Bridge запущен")
print(f"  Isaac Sim TCP : {WIN_IP}:{ISAAC_PORT}")
print(f"  ArduPilot UDP : 0.0.0.0:{AP_PORT}")
print()

# ── UDP сокет ──
ap_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ap_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
ap_sock.bind(("0.0.0.0", AP_PORT))
ap_sock.settimeout(2.0)

# ── Self-test ──
print("Self-test loopback... ", end="", flush=True)
_ts = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_ts.sendto(b"SELFTEST", ("127.0.0.1", AP_PORT))
_ts.close()
try:
    _d, _ = ap_sock.recvfrom(4096)
    print("OK" if _d == b"SELFTEST" else f"странные данные ({len(_d)} б)")
except socket.timeout:
    print("FAILED — loopback UDP не работает!")
    sys.exit(1)

# ── Парсинг servo-пакета ArduPilot ──
SERVO_PACKET_MAGIC = 18458  # 0x47FA — ArduPilot SITL binary servo packet

def parse_servos(data: bytes) -> list:
    # 40-byte format: magic(2)+frame_rate(2)+frame_count(4)+pwm[16](32)
    if len(data) >= 40 and struct.unpack_from("<H", data, 0)[0] == SERVO_PACKET_MAGIC:
        return list(struct.unpack_from("<16H", data, 8))
    # 36-byte format (old): magic(2)+frame_rate(2)+pwm[16](32)
    if len(data) >= 36 and struct.unpack_from("<H", data, 0)[0] == SERVO_PACKET_MAGIC:
        return list(struct.unpack_from("<16H", data, 4))
    try:
        return json.loads(data.decode("utf-8", errors="replace")).get("pwm", [1000]*16)
    except Exception:
        return [1000] * 16

# ── Общий кеш стейта (обновляется из фонового потока) ──
_state_lock = threading.Lock()
_cached_state = {
    "imu":        {"gyro": [0,0,0], "accel_body": [0,0,-9.81]},
    "position":   [0.0, 0.0, 0.0],
    "velocity":   [0.0, 0.0, 0.0],
    "quaternion": [1.0, 0.0, 0.0, 0.0],  # [W, X, Y, Z] NED body frame
}
_pending_servos = [1000] * 16
_isaac_busy     = False

def _isaac_worker():
    """Фоновый поток: шлёт сервосы в Isaac Sim, обновляет кеш стейта."""
    global _cached_state, _isaac_busy
    while True:
        if not _isaac_busy:
            time.sleep(0.001)
            continue
        servos = list(_pending_servos)
        try:
            tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp.settimeout(2.0)
            tcp.connect((WIN_IP, ISAAC_PORT))
            tcp.sendall(json.dumps({"servos": servos}).encode())
            tcp.shutdown(socket.SHUT_WR)
            chunks = []
            while True:
                chunk = tcp.recv(8192)
                if not chunk:
                    break
                chunks.append(chunk)
            tcp.close()
            resp = json.loads(b"".join(chunks).decode())
            with _state_lock:
                _cached_state = {
                    "imu":        resp.get("imu",        _cached_state["imu"]),
                    "position":   resp.get("position",   _cached_state["position"]),
                    "velocity":   resp.get("velocity",   _cached_state["velocity"]),
                    "quaternion": resp.get("quaternion", _cached_state["quaternion"]),
                }
        except Exception as e:
            pass  # Держим старый кеш
        _isaac_busy = False

_worker = threading.Thread(target=_isaac_worker, daemon=True)
_worker.start()

# ── Главный цикл ──
ap_addr   = None
pkt_count = 0
_t0       = time.time()
_LOG_PKTS = 10  # первые N пакетов печатаем полностью
_first_high_pwm = False  # печатаем raw hex при первом PWM > 1000

print(f"Ожидаем пакеты от ArduPilot…")

while True:
    # 1. Принимаем servo-пакет
    try:
        data, addr = ap_sock.recvfrom(4096)
        ap_addr = addr
    except socket.timeout:
        print(".", end="", flush=True)
        continue
    except Exception as e:
        print(f"\n[recv error] {e}")
        continue

    # 2. Парсим сервосы
    try:
        servos = parse_servos(data)
    except Exception:
        servos = [1000] * 16

    # 3. НЕМЕДЛЕННО отвечаем ArduPilot с кешированным стейтом
    t_s = time.time() - _t0  # ArduPilot ожидает секунды, не миллисекунды
    with _state_lock:
        snap = dict(_cached_state)
    # rc3: 1100 пока disarmed → нормальный ARM без "throttle not neutral"
    #      1500 после ARM (PWM > 1000) → auto_armed=True → NAV_TAKEOFF работает
    _rc3 = 1500 if any(p > 1000 for p in servos[:4]) else 1100
    sensor = {
        "timestamp":  round(t_s, 6),
        "imu":        snap["imu"],
        "position":   snap["position"],
        "velocity":   snap["velocity"],
        "quaternion": snap["quaternion"],
        "rc": {"rc_1":1500,"rc_2":1500,"rc_3":_rc3,"rc_4":1500,
               "rc_5":1500,"rc_6":1500,"rc_7":1500,"rc_8":1500},
    }
    out_bytes = (json.dumps(sensor, separators=(',', ':')) + "\n").encode()
    try:
        # \n в конце обязателен — ArduPilot парсит по нулевым байтам из \n
        ap_sock.sendto(out_bytes, ap_addr)
    except Exception as e:
        print(f"\n[send error] {e}")

    # Лог первых 10 пакетов туда и обратно
    if pkt_count < _LOG_PKTS:
        print(f"\n─── PKT #{pkt_count+1} ───")
        print(f"  ← ArduPilot  {len(data)}б  PWM:{servos[:4]}")
        q = snap['quaternion']; print(f"  → Isaac/AP   t={t_s:.3f}s  pos={[round(x,3) for x in snap['position']]}  quat=[{q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f}]  rc_ch3={sensor['rc']['rc_3']}")

    # 4. Запускаем обновление Isaac Sim (фоновый поток)
    if not _isaac_busy:
        _pending_servos[:] = servos
        _isaac_busy = True

    pkt_count += 1
    # Первый пакет с PWM > 1000 — печатаем raw hex для диагностики
    if not _first_high_pwm and any(x > 1000 for x in servos[:4]):
        _first_high_pwm = True
        print(f"\n[!!!ПЕРВЫЙ ВЫСОКИЙ PWM!!!] raw({len(data)}б)={data[:40].hex()}  parsed={servos[:4]}")
    if pkt_count % 100 == 0:
        s = servos[:4]
        p = snap["position"]
        rate = pkt_count / (time.time() - _t0)
        motors_on = any(x > 1000 for x in s)
        tag = " *** МОТОРЫ ***" if motors_on else ""
        print(f"\n[{pkt_count}] {rate:.0f}Hz  PWM:{s}  pos(NED):{[round(x,2) for x in p]}{tag}")
