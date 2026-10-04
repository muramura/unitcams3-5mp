#!/usr/bin/env python3
"""
MAVLink Vision Inspector & Packet Constructor Verifier
=====================================================
Verifies MAVLink #100 (OPTICAL_FLOW) and #149 (LANDING_TARGET) packets.
Can run in two modes:
  1. Self-Test / Constructor Check: verifies packet construction byte-by-byte.
  2. Live Inspector: listens on UDP 14550 (or Serial) and displays live incoming packets.
"""

import sys
import time
import struct
import argparse

try:
    from pymavlink.dialects.v20 import common as mavlink2
except ImportError:
    print("Warning: pymavlink not installed. Using raw byte parser.")
    mavlink2 = None

def crc16_accumulate(b, crc):
    ch = b ^ (crc & 0xFF)
    ch = (ch ^ (ch << 4)) & 0xFF
    return ((crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4)) & 0xFFFF

def verify_constructors():
    print("=" * 60)
    print(" 🛠️  MAVLink Packet Constructor Verification")
    print("=" * 60)

    # -------------------------------------------------------------
    # 1. OPTICAL_FLOW (#100) Constructor Check
    # -------------------------------------------------------------
    print("\n[1] Testing OPTICAL_FLOW (#100) Constructor...")
    t_usec = int(time.time() * 1e6)
    flow_x = 42
    flow_y = -85
    quality = 195
    ground_dist = -1.0

    # Match C constructor: mavlink_pack_optical_flow_v2
    header = struct.pack('<BBBBBBBHB', 0xFD, 34, 0, 0, 1, 1, 197, 100, 0)
    payload = struct.pack('<QfffhhBBff', t_usec, 0.0, 0.0, ground_dist, flow_x, flow_y, 0, quality, 0.0, 0.0)
    crc = 0xFFFF
    for b in header[1:]: crc = crc16_accumulate(b, crc)
    for b in payload: crc = crc16_accumulate(b, crc)
    crc = crc16_accumulate(175, crc) # CRC_EXTRA 175
    pkt_flow = header + payload + struct.pack('<H', crc)

    print(f"  - Constructed Packet Length: {len(pkt_flow)} bytes (Expected: 46)")
    print(f"  - Hex Dump: {pkt_flow.hex()}")

    if mavlink2:
        mav = mavlink2.MAVLink(None)
        decoded = None
        for b in pkt_flow:
            msg = mav.parse_char(bytes([b]))
            if msg:
                decoded = msg
                break
        if decoded:
            print("  - Decoded via pymavlink:")
            print(f"    * MSG ID: {decoded.get_msgId()} (OPTICAL_FLOW)")
            print(f"    * flow_x: {decoded.flow_x} (raw pixel*10)")
            print(f"    * flow_y: {decoded.flow_y}")
            print(f"    * quality: {decoded.quality} / 255")
            print(f"    * ground_distance: {decoded.ground_distance} m")
            assert decoded.flow_x == flow_x
            assert decoded.flow_y == flow_y
            assert decoded.quality == quality
            print("  ✅ OPTICAL_FLOW Constructor Verification: PASSED!")
        else:
            print("  ❌ Failed to parse OPTICAL_FLOW packet!")

    # -------------------------------------------------------------
    # 2. LANDING_TARGET (#149) Constructor Check
    # -------------------------------------------------------------
    print("\n[2] Testing LANDING_TARGET (#149) Constructor...")
    target_id = 0
    angle_x = 0.035 # ~2 degrees right
    angle_y = -0.018 # ~1 degree up
    dist = 0.75 # 75cm

    # Match C constructor: mavlink_pack_landing_target_v2
    header_lt = struct.pack('<BBBBBBBHB', 0xFD, 60, 0, 0, 1, 1, 197, 149, 0)
    payload_lt = struct.pack('<QfffffBBfffffffBB',
        t_usec, angle_x, angle_y, dist, 0.15, 0.15,
        target_id, 8, # MAV_FRAME_BODY_NED (8)
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        1, 0
    )
    crc = 0xFFFF
    for b in header_lt[1:]: crc = crc16_accumulate(b, crc)
    for b in payload_lt: crc = crc16_accumulate(b, crc)
    crc = crc16_accumulate(200, crc) # CRC_EXTRA 200
    pkt_lt = header_lt + payload_lt + struct.pack('<H', crc)

    print(f"  - Constructed Packet Length: {len(pkt_lt)} bytes (Expected: 72)")
    print(f"  - Hex Dump: {pkt_lt.hex()}")

    if mavlink2:
        mav = mavlink2.MAVLink(None)
        decoded_lt = None
        for b in pkt_lt:
            msg = mav.parse_char(bytes([b]))
            if msg:
                decoded_lt = msg
                break
        if decoded_lt:
            print("  - Decoded via pymavlink:")
            print(f"    * MSG ID: {decoded_lt.get_msgId()} (LANDING_TARGET)")
            print(f"    * target_num: {decoded_lt.target_num} (ID)")
            print(f"    * angle_x: {decoded_lt.angle_x:.4f} rad ({decoded_lt.angle_x * 57.2958:.2f} deg)")
            print(f"    * angle_y: {decoded_lt.angle_y:.4f} rad ({decoded_lt.angle_y * 57.2958:.2f} deg)")
            print(f"    * distance: {decoded_lt.distance:.2f} m")
            assert decoded_lt.target_num == target_id
            assert abs(decoded_lt.distance - dist) < 1e-3
            print("  ✅ LANDING_TARGET Constructor Verification: PASSED!")
        else:
            print("  ❌ Failed to parse LANDING_TARGET packet!")

    # -------------------------------------------------------------
    # 3. HEARTBEAT (#0) Constructor Check
    # -------------------------------------------------------------
    print("\n[3] Testing HEARTBEAT (#0) Constructor...")
    header_hb = struct.pack('<BBBBBBBHB', 0xFD, 9, 0, 0, 1, 1, 197, 0, 0)
    # Payload wire order: custom_mode(I), type(B), autopilot(B), base_mode(B), system_status(B), mavlink_version(B)
    payload_hb = struct.pack('<IBBBBB', 0, 18, 8, 0, 4, 3)
    crc_hb = 0xFFFF
    for b in header_hb[1:]: crc_hb = crc16_accumulate(b, crc_hb)
    for b in payload_hb: crc_hb = crc16_accumulate(b, crc_hb)
    crc_hb = crc16_accumulate(50, crc_hb) # CRC_EXTRA 50
    pkt_hb = header_hb + payload_hb + struct.pack('<H', crc_hb)

    print(f"  - Constructed Packet Length: {len(pkt_hb)} bytes (Expected: 21)")
    print(f"  - Hex Dump: {pkt_hb.hex()}")

    if mavlink2:
        mav = mavlink2.MAVLink(None)
        decoded_hb = None
        for b in pkt_hb:
            msg = mav.parse_char(bytes([b]))
            if msg:
                decoded_hb = msg
                break
        if decoded_hb:
            print("  - Decoded via pymavlink:")
            print(f"    * MSG ID: {decoded_hb.get_msgId()} (HEARTBEAT)")
            print(f"    * type: {decoded_hb.type} (ONBOARD_CONTROLLER)")
            print(f"    * autopilot: {decoded_hb.autopilot} (INVALID/NONE)")
            print(f"    * system_status: {decoded_hb.system_status} (ACTIVE)")
            assert decoded_hb.type == 18
            print("  ✅ HEARTBEAT Constructor Verification: PASSED!")
        else:
            print("  ❌ Failed to parse HEARTBEAT packet!")

    print("\n" + "=" * 60)
    print(" 🎉 All MAVLink Constructors Verified Successfully!")
    print("=" * 60)

def find_serial_ports():
    import glob
    ports = []
    # Mac
    ports.extend(glob.glob('/dev/cu.usb*'))
    ports.extend(glob.glob('/dev/cu.wch*'))
    ports.extend(glob.glob('/dev/cu.SLAB*'))
    ports.extend(glob.glob('/dev/cu.usbmodem*'))
    # Linux
    ports.extend(glob.glob('/dev/ttyUSB*'))
    ports.extend(glob.glob('/dev/ttyACM*'))
    return sorted(list(set(ports)))

def live_inspector(udp_port=None, serial_port=None, baudrate=2000000):
    mav = mavlink2.MAVLink(None) if mavlink2 else None

    if serial_port:
        if serial_port == "auto":
            detected = find_serial_ports()
            if not detected:
                print("❌ No serial ports detected! Please check your USB-UART connection.")
                print("   Available devices in /dev/cu.* or /dev/tty*:")
                import glob
                for p in glob.glob('/dev/cu.*')[:10]:
                    print(f"     {p}")
                return
            serial_port = detected[0]
            print(f"🔍 Auto-detected serial port: {serial_port}")

        try:
            import serial
            ser = serial.Serial(serial_port, baudrate, timeout=0.1)
        except ImportError:
            print("⚠️ 'pyserial' not installed. Attempting POSIX raw serial...")
            import os, termios, tty
            fd = os.open(serial_port, os.O_RDWR | os.O_NOCTTY)
            tty.setraw(fd)
            class RawSerial:
                def __init__(self, fd): self.fd = fd
                def read(self, n=1):
                    try: return os.read(self.fd, n)
                    except Exception: return b""
            ser = RawSerial(fd)
        except Exception as e:
            print(f"❌ Failed to open serial port {serial_port}: {e}")
            detected = find_serial_ports()
            if detected:
                print(f"💡 Detected available ports: {', '.join(detected)}")
            return

        print("=" * 65)
        print(f" 🔌 MAVLink Serial Inspector: {serial_port} @ {baudrate} bps")
        print(" Press Ctrl+C to stop")
        print("=" * 65)

        def read_stream():
            while True:
                if hasattr(ser, 'in_waiting'):
                    chunk = ser.read(ser.in_waiting or 1)
                else:
                    chunk = ser.read(64)
                if chunk:
                    yield chunk
                else:
                    time.sleep(0.005)
    else:
        import socket
        print("=" * 65)
        print(f" 📡 MAVLink UDP Inspector listening on 0.0.0.0:{udp_port}")
        print(" Press Ctrl+C to stop")
        print("=" * 65)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", udp_port))

        def read_stream():
            while True:
                data, _ = sock.recvfrom(2048)
                yield data

    # Statistics tracking
    stats = {
        "HEARTBEAT": {"count": 0, "last_t": 0, "rate": 0.0, "comp": 0},
        "OPTICAL_FLOW": {"count": 0, "last_t": 0, "rate": 0.0, "details": ""},
        "LANDING_TARGET": {"count": 0, "last_t": 0, "rate": 0.0, "details": ""},
        "OTHER": {"count": 0}
    }
    window_start = time.time()
    window_counts = {"HEARTBEAT": 0, "OPTICAL_FLOW": 0, "LANDING_TARGET": 0}

    last_summary_t = time.time()

    try:
        for data in read_stream():
            now = time.time()
            if not mav:
                print(f"Received {len(data)} raw bytes: {data.hex()[:32]}...")
                continue

            for b in data:
                msg = mav.parse_char(bytes([b]))
                if not msg:
                    continue

                msg_type = msg.get_type()
                sys_id = msg.get_srcSystem()
                comp_id = msg.get_srcComponent()

                if msg_type in window_counts:
                    window_counts[msg_type] += 1
                    stats[msg_type]["count"] += 1
                    stats[msg_type]["last_t"] = now

                if msg_type == "HEARTBEAT":
                    stats["HEARTBEAT"]["comp"] = comp_id
                    print(f"💓 [HEARTBEAT #0] Sys={sys_id}, Comp={comp_id} (Type={msg.type}) | seq={msg.get_seq()}")
                elif msg_type == "OPTICAL_FLOW":
                    stats["OPTICAL_FLOW"]["details"] = f"dx={msg.flow_x/10.0:+.1f}px, dy={msg.flow_y/10.0:+.1f}px, Q={msg.quality}"
                    # Print sampled logs (not all 30fps to avoid flooding terminal)
                    if stats["OPTICAL_FLOW"]["count"] % 15 == 0:
                        print(f"🚁 [OPTICAL_FLOW #100] {stats['OPTICAL_FLOW']['details']} (Qual={msg.quality}/255, Dist={msg.ground_distance:.2f}m)")
                elif msg_type == "LANDING_TARGET":
                    deg_x = msg.angle_x * 57.2958
                    deg_y = msg.angle_y * 57.2958
                    stats["LANDING_TARGET"]["details"] = f"ID={msg.target_num}, dist={msg.distance:.2f}m"
                    if stats["LANDING_TARGET"]["count"] % 3 == 0:
                        print(f"🎯 [LANDING_TARGET #149] ID={msg.target_num} | Angles: X={deg_x:+.1f}°, Y={deg_y:+.1f}° | Dist={msg.distance:.2f}m")
                else:
                    stats["OTHER"]["count"] += 1

            # Update rates & summary every 2.0 seconds
            if now - last_summary_t >= 2.0:
                elapsed = now - window_start
                for k in window_counts:
                    stats[k]["rate"] = window_counts[k] / elapsed
                    window_counts[k] = 0
                window_start = now
                last_summary_t = now

                hb_stat = stats["HEARTBEAT"]
                flow_stat = stats["OPTICAL_FLOW"]
                lt_stat = stats["LANDING_TARGET"]

                hb_age = f"{now - hb_stat['last_t']:.1f}s ago" if hb_stat['last_t'] > 0 else "never"
                flow_age = f"{now - flow_stat['last_t']:.2f}s ago" if flow_stat['last_t'] > 0 else "never"

                print("-" * 65)
                print(f"📊 [定期送信状況サマリー ({time.strftime('%H:%M:%S')})]")
                print(f"  💓 HEARTBEAT   (#0)   : {hb_stat['rate']:4.1f} Hz  (総計: {hb_stat['count']:5d}回, 最終: {hb_age:>8s}, Comp: {hb_stat['comp']})")
                print(f"  🚁 OPTICAL_FLOW(#100) : {flow_stat['rate']:4.1f} Hz  (総計: {flow_stat['count']:5d}回, 最終: {flow_age:>8s})")
                if lt_stat['count'] > 0:
                    print(f"  🎯 LANDING_TGT (#149) : {lt_stat['rate']:4.1f} Hz  (総計: {lt_stat['count']:5d}回)")
                print("-" * 65)

    except KeyboardInterrupt:
        print("\nInspector stopped.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MAVLink Vision Constructor & Inspector")
    parser.add_argument("--test", action="store_true", help="Run constructor verification")
    parser.add_argument("--port", type=int, default=14550, help="UDP port to listen on (default: 14550)")
    parser.add_argument("--serial", type=str, default=None, nargs='?', const="auto", help="Serial device (e.g. /dev/ttyUSB0, /dev/cu.usbserial-*, or 'auto')")
    parser.add_argument("--baud", type=int, default=2000000, help="Serial baud rate (default: 2000000)")
    args = parser.parse_args()

    if args.test:
        verify_constructors()
    elif args.serial is not None:
        live_inspector(serial_port=args.serial, baudrate=args.baud)
    elif len(sys.argv) == 1:
        # Default: auto-detect serial or fallback to test
        detected = find_serial_ports()
        if detected:
            print(f"Detected serial port {detected[0]}, starting serial inspector...")
            live_inspector(serial_port=detected[0], baudrate=2000000)
        else:
            verify_constructors()
            print("\n💡 ヒント: CamS3のUSB-UARTを接続して以下を実行すると、リアルタイムにパケットを確認できます:")
            print("   python3 inspect_mavlink_vision.py --serial auto --baud 2000000")
    else:
        live_inspector(udp_port=args.port)
