import asyncio
import json
import math
import re
import struct
import threading
import time
import http.server
import os
import datetime
import serial
import serial.tools.list_ports
import websockets

# ─── Global State ────────────────────────────────────────────
clients = set()
usb_port = None          # BU04 USB口 (数据)
usb_thread = None
at_port = None           # TTL口 (AT指令)
at_thread = None
running_usb = False
running_at = False
loop = None

# Calibration
angle_offset = 0.0       # 角度偏移校正
dist_coeff_a = 1.0       # 距离校正系数 a (y = a*x + b)
dist_coeff_b = 0.0       # 距离校正系数 b

# Logging
log_file = None
logging_enabled = False

# Tags state
tags = {}                 # addr -> tag data


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [{"port": p.device, "desc": p.description} for p in ports]


# ─── JSON Protocol Parser ────────────────────────────────────
# Format: JS006C{"TWR":{"a16":"4096","R":115,"T":0,"D":76,"P":-123,"Xcm":-57,"Ycm":50,...}}

def parse_uwb_line(line):
    results = []

    # NewTag
    m = re.search(r'JS[0-9A-Fa-f]{4}(\{"NewTag"[^}]+\})', line)
    if m:
        try:
            d = json.loads(m.group(1))
            results.append({"type": "new_tag", "id": d["NewTag"]})
        except Exception as e:
            print(f"[WARN] NewTag parse error: {e}")

    # TWR position data (full JS wrapper)
    m = re.search(r'JS[0-9A-Fa-f]{4}(\{"TWR":\{.+?\}\})', line)
    if m:
        try:
            d = json.loads(m.group(1))["TWR"]
            results.append(build_position(d))
        except Exception as e:
            print(f"[WARN] TWR(JS) parse error: {e} | {m.group(1)[:80]}")

    # Fallback: standalone TWR inner JSON {"a16":...}
    if not any(r.get("type") == "position" for r in results):
        m = re.search(r'(\{"a16":.+?\})', line)
        if m:
            try:
                d = json.loads(m.group(1))
                if "D" in d:
                    results.append(build_position(d))
            except Exception as e:
                print(f"[WARN] TWR(standalone) parse error: {e} | {m.group(1)[:80]}")

    # Fallback: {"TWR":{...}} without JS prefix
    if not any(r.get("type") == "position" for r in results):
        m = re.search(r'(\{"TWR":\{.+?\}\})', line)
        if m:
            try:
                d = json.loads(m.group(1))["TWR"]
                results.append(build_position(d))
            except Exception as e:
                print(f"[WARN] TWR(noJS) parse error: {e}")

    return results


def build_position(twr):
    global angle_offset, dist_coeff_a, dist_coeff_b
    raw_d = twr.get("D", 0)
    xcm = twr.get("Xcm", 0)
    ycm = twr.get("Ycm", 0)
    pdoa = twr.get("P", 0)
    addr = str(twr.get("a16", ""))
    seq = twr.get("R", 0)
    ts = twr.get("T", 0)
    clk = twr.get("O", 0)
    v_raw = twr.get("V", 0)
    acc_x = twr.get("X", 0)
    acc_y = twr.get("Y", 0)
    acc_z = twr.get("Z", 0)

    # Distance correction
    corrected_d = dist_coeff_a * raw_d + dist_coeff_b

    # Angle calculation
    if ycm != 0:
        angle = math.atan(xcm / ycm) * 180 / math.pi
    else:
        angle = 90.0 if xcm > 0 else (-90.0 if xcm < 0 else 0.0)
    angle += angle_offset

    # Battery voltage from V field (mV)
    battery_mv = v_raw

    # is_lowbattery / is_alarm from usercmd (not in JSON, ignore)
    tag_data = {
        "type": "position",
        "addr": addr,
        "seq": seq,
        "timestamp": ts,
        "raw_distance": raw_d,
        "distance": round(corrected_d, 1),
        "angle": round(angle, 1),
        "pdoa": pdoa,
        "xcm": xcm,
        "ycm": ycm,
        "xm": round(xcm / 100.0, 3),
        "ym": round(ycm / 100.0, 3),
        "clock_offset": clk,
        "battery": battery_mv,
        "acc_x": acc_x,
        "acc_y": acc_y,
        "acc_z": acc_z,
    }

    # Update global tags
    tags[addr] = tag_data

    # Log if enabled
    if logging_enabled and log_file:
        try:
            log_file.write(json.dumps(tag_data) + "\n")
            log_file.flush()
        except:
            pass

    return tag_data


# ─── HEX Protocol Parser ─────────────────────────────────────
# head(0x2A) len sn addr(2B) angle(4B) distance(4B) usercmd(2B)
# F_Path(4B) RX_Level(4B) Acc_X(2B) Acc_Y(2B) Acc_Z(2B) Check Foot(0x23)

def parse_hex_frame(data):
    """Parse binary HEX protocol frames from byte buffer, returns (results, remaining_bytes)"""
    results = []
    i = 0
    while i < len(data):
        # Find head byte 0x2A
        if data[i] != 0x2A:
            i += 1
            continue

        if i + 2 > len(data):
            break

        payload_len = data[i + 1]
        frame_len = 1 + 1 + payload_len + 1 + 1  # head + len + payload + check + foot

        if i + frame_len > len(data):
            break  # incomplete frame

        if data[i + frame_len - 1] != 0x23:
            i += 1
            continue

        # Parse payload (little-endian)
        try:
            offset = i + 2  # skip head, len
            sn = data[offset]
            addr = struct.unpack_from('<H', data, offset + 1)[0]
            angle_raw = struct.unpack_from('<i', data, offset + 3)[0]  # signed 32-bit
            dist_raw = struct.unpack_from('<I', data, offset + 7)[0]  # unsigned 32-bit, cm
            usercmd = struct.unpack_from('<H', data, offset + 11)[0]
            fpath = struct.unpack_from('<I', data, offset + 13)[0]
            rxlevel = struct.unpack_from('<I', data, offset + 17)[0]
            acc_x_raw = struct.unpack_from('<h', data, offset + 21)[0]
            acc_y_raw = struct.unpack_from('<h', data, offset + 23)[0]
            acc_z_raw = struct.unpack_from('<h', data, offset + 25)[0]

            is_lowbat = usercmd & 0x01
            is_alarm = (usercmd >> 1) & 0x01

            # Build TWR-like dict
            twr = {
                "a16": str(addr),
                "R": sn,
                "T": 0,
                "D": dist_raw,
                "P": angle_raw,
                "Xcm": 0,
                "Ycm": 0,
                "O": 0,
                "V": rxlevel,
                "X": acc_x_raw,
                "Y": acc_y_raw,
                "Z": acc_z_raw,
            }
            # For HEX, angle is directly provided, approximate Xcm/Ycm
            if dist_raw > 0:
                angle_deg = angle_raw  # already in degrees (integer)
                rad = angle_deg * math.pi / 180.0
                twr["Xcm"] = int(dist_raw * math.sin(rad))
                twr["Ycm"] = int(dist_raw * math.cos(rad))

            pos = build_position(twr)
            pos["is_lowbat"] = is_lowbat
            pos["is_alarm"] = is_alarm
            pos["fpath"] = fpath
            results.append(pos)
        except:
            pass

        i += frame_len

    return results, data[i:]


# ─── Serial Reader Threads ────────────────────────────────────

def usb_reader():
    """读取USB口数据 (JSON + HEX)"""
    global usb_port, running_usb
    text_buf = ""
    hex_buf = bytearray()

    while running_usb and usb_port and usb_port.is_open:
        try:
            if usb_port.in_waiting > 0:
                raw = usb_port.read(usb_port.in_waiting)

                # Try JSON text parsing
                text_buf += raw.decode('utf-8', errors='ignore')
                while '\n' in text_buf:
                    line, text_buf = text_buf.split('\n', 1)
                    line = line.strip()
                    if line:
                        parsed = parse_uwb_line(line)
                        for p in parsed:
                            _send_to_clients(p)
                        # Forward raw text for log display
                        _send_to_clients({"type": "raw", "text": line})

                # Try HEX parsing
                hex_buf.extend(raw)
                if len(hex_buf) > 0:
                    results, hex_buf = parse_hex_frame(hex_buf)
                    hex_buf = bytearray(hex_buf)
                    for r in results:
                        _send_to_clients(r)

                # Prevent hex buffer overflow
                if len(hex_buf) > 4096:
                    hex_buf = bytearray()
            else:
                time.sleep(0.005)
        except Exception as e:
            _send_to_clients({"type": "error", "msg": f"USB read error: {e}"})
            time.sleep(0.1)


def at_reader():
    """读取TTL AT口返回"""
    global at_port, running_at
    buf = ""
    while running_at and at_port and at_port.is_open:
        try:
            if at_port.in_waiting > 0:
                data = at_port.read(at_port.in_waiting).decode('utf-8', errors='ignore')
                buf += data
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if line:
                        _send_to_clients({"type": "at_response", "text": line})
            else:
                time.sleep(0.01)
        except:
            time.sleep(0.1)


def _send_to_clients(data):
    if loop and clients:
        asyncio.run_coroutine_threadsafe(broadcast(json.dumps(data)), loop)


async def broadcast(message):
    if clients:
        await asyncio.gather(*[c.send(message) for c in clients], return_exceptions=True)


# ─── WebSocket Handler ────────────────────────────────────────

async def ws_handler(websocket):
    global usb_port, usb_thread, running_usb
    global at_port, at_thread, running_at
    global angle_offset, dist_coeff_a, dist_coeff_b
    global logging_enabled, log_file

    clients.add(websocket)
    try:
        async for message in websocket:
            try:
                cmd = json.loads(message)
                action = cmd.get("action")

                if action == "list_ports":
                    await websocket.send(json.dumps({"type": "ports", "data": list_serial_ports()}))

                elif action == "connect_usb":
                    port_name = cmd.get("port")
                    if usb_port and usb_port.is_open:
                        running_usb = False
                        if usb_thread: usb_thread.join(timeout=2)
                        usb_port.close()
                    try:
                        usb_port = serial.Serial(port_name, 115200, timeout=1)
                        running_usb = True
                        usb_thread = threading.Thread(target=usb_reader, daemon=True)
                        usb_thread.start()
                        await websocket.send(json.dumps({"type": "usb_status", "connected": True, "port": port_name}))
                    except Exception as e:
                        await websocket.send(json.dumps({"type": "error", "msg": str(e)}))

                elif action == "disconnect_usb":
                    running_usb = False
                    if usb_thread: usb_thread.join(timeout=2)
                    if usb_port and usb_port.is_open: usb_port.close()
                    usb_port = None
                    await websocket.send(json.dumps({"type": "usb_status", "connected": False}))

                elif action == "connect_at":
                    port_name = cmd.get("port")
                    if at_port and at_port.is_open:
                        running_at = False
                        if at_thread: at_thread.join(timeout=2)
                        at_port.close()
                    try:
                        at_port = serial.Serial(port_name, 115200, timeout=1)
                        running_at = True
                        at_thread = threading.Thread(target=at_reader, daemon=True)
                        at_thread.start()
                        await websocket.send(json.dumps({"type": "at_status", "connected": True, "port": port_name}))
                    except Exception as e:
                        await websocket.send(json.dumps({"type": "error", "msg": str(e)}))

                elif action == "disconnect_at":
                    running_at = False
                    if at_thread: at_thread.join(timeout=2)
                    if at_port and at_port.is_open: at_port.close()
                    at_port = None
                    await websocket.send(json.dumps({"type": "at_status", "connected": False}))

                elif action == "send_at":
                    text = cmd.get("text", "")
                    if at_port and at_port.is_open:
                        at_port.write((text + "\r\n").encode())
                        await websocket.send(json.dumps({"type": "at_sent", "text": text}))
                    else:
                        await websocket.send(json.dumps({"type": "error", "msg": "AT口未连接"}))

                elif action == "set_calibration":
                    angle_offset = float(cmd.get("angle_offset", 0))
                    dist_coeff_a = float(cmd.get("dist_a", 1.0))
                    dist_coeff_b = float(cmd.get("dist_b", 0.0))
                    await websocket.send(json.dumps({
                        "type": "calibration_updated",
                        "angle_offset": angle_offset,
                        "dist_a": dist_coeff_a,
                        "dist_b": dist_coeff_b
                    }))

                elif action == "start_logging":
                    if not logging_enabled:
                        fname = f"uwb_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
                        fpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), fname)
                        log_file = open(fpath, 'w', encoding='utf-8')
                        logging_enabled = True
                        await websocket.send(json.dumps({"type": "logging", "enabled": True, "file": fname}))

                elif action == "stop_logging":
                    logging_enabled = False
                    if log_file:
                        log_file.close()
                        log_file = None
                    await websocket.send(json.dumps({"type": "logging", "enabled": False}))

                elif action == "get_tags":
                    await websocket.send(json.dumps({"type": "tags_snapshot", "tags": tags}))

                elif action == "clear_tags":
                    tags.clear()
                    await websocket.send(json.dumps({"type": "tags_cleared"}))

            except json.JSONDecodeError:
                pass
    finally:
        clients.discard(websocket)


# ─── HTTP Server ──────────────────────────────────────────────

class HTTPHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=os.path.dirname(os.path.abspath(__file__)), **kwargs)
    def log_message(self, fmt, *args):
        pass

def start_http(port=8080):
    http.server.HTTPServer(("0.0.0.0", port), HTTPHandler).serve_forever()


# ─── Main ─────────────────────────────────────────────────────

async def main():
    global loop
    loop = asyncio.get_event_loop()
    threading.Thread(target=start_http, args=(8080,), daemon=True).start()

    print("=" * 50)
    print("  UWB PDOA Viewer  (Full)")
    print("=" * 50)
    print("  http://localhost:8080")
    print("  ws://localhost:8765")
    print("=" * 50)

    import webbrowser
    webbrowser.open("http://localhost:8080")

    async with websockets.serve(ws_handler, "0.0.0.0", 8765):
        await asyncio.Future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down...")
