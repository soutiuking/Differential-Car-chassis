#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四驱差速底盘 + 定位串口调试工具（与 Core/Src/chassis_protocol.c 对应）。
monitor 会同时解析 0x81 底盘状态帧和 0x82 定位帧。

依赖：
    pip install pyserial

示例：
    python chassis_host.py --port COM7 enable
    python chassis_host.py --port COM7 velocity 0.25 0.0
    python chassis_host.py --port COM7 wheel 100 100 100 100
    python chassis_host.py --port COM7 pid 255 1.2 0.6 0.02
    python chassis_host.py --port COM7 monitor

不带 --port 时只打印待发送帧，便于核对 CRC。
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional

SOF1 = 0xAA
SOF2 = 0x55
HEADER_LEN = 5
MAX_PAYLOAD = 48
STATUS_LEN = 44
NAV_LEN = 30

CMD_VELOCITY = 0x01
CMD_WHEEL = 0x02
CMD_PID = 0x03
CMD_ENABLE = 0x04
CMD_PING = 0x05
CMD_STOP = 0x06
RSP_STATUS = 0x81
RSP_NAV = 0x82


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(cmd: int, payload: bytes, seq: int = 0) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes([len(payload), seq & 0xFF, cmd & 0xFF]) + payload
    crc = crc16_modbus(body)
    return bytes([SOF1, SOF2]) + body + struct.pack("<H", crc)


def clamp_i16(value: int) -> int:
    return max(-32768, min(32767, int(round(value))))


@dataclass
class Status:
    faults: int
    flags: int
    linear_mm_s: int
    angular_mrad_s: int
    command_age_ms: int
    target_rpm: tuple
    measured_rpm: tuple
    pwm: tuple
    rx_ok: int
    rx_bad: int
    control_cycles: int
    last_cmd_status: int

    @property
    def enabled(self) -> bool:
        return bool(self.flags & 0x01)

    @property
    def mode(self) -> int:
        return (self.flags >> 1) & 0x03

    @property
    def version(self) -> int:
        return (self.flags >> 8) & 0xFF

    def format(self) -> str:
        mode_name = {0: "IDLE", 1: "VEL", 2: "WHEEL"}.get(self.mode, "?")
        targets = " ".join(f"{v / 10:7.1f}" for v in self.target_rpm)
        measured = " ".join(f"{v / 10:7.1f}" for v in self.measured_rpm)
        pwm = " ".join(f"{v:5d}" for v in self.pwm)
        return (
            f"en={int(self.enabled)} mode={mode_name} fault=0x{self.faults:04X} "
            f"v={self.linear_mm_s / 1000:+.3f}m/s w={self.angular_mrad_s / 1000:+.3f}rad/s "
            f"age={self.command_age_ms}ms\n"
            f"  target_rpm: {targets}\n"
            f"  meas_rpm  : {measured}\n"
            f"  pwm(‰)    : {pwm}\n"
            f"  rx_ok={self.rx_ok} rx_bad={self.rx_bad} cycles={self.control_cycles} "
            f"last_cmd={self.last_cmd_status}"
        )


@dataclass
class NavStatus:
    flags: int
    x_mm: int
    y_mm: int
    theta_mrad: int
    v_mm_s: int
    omega_mrad_s: int
    roll_mrad: int
    pitch_mrad: int
    tof_mm: tuple
    p_xx_cm2: int
    p_yy_cm2: int
    p_tt_mrad2: int
    tof_updates: tuple

    def format(self) -> str:
        tof = " ".join(
            f"{'F' if i == 0 else 'R'}={v}mm{'*' if not (self.flags & (1 << (2 + i))) else ''}"
            for i, v in enumerate(self.tof_mm)
        )
        return (
            f"NAV x={self.x_mm / 1000:+.3f}m y={self.y_mm / 1000:+.3f}m "
            f"th={self.theta_mrad / 1000:+.3f}rad v={self.v_mm_s / 1000:+.3f}m/s "
            f"w={self.omega_mrad_s / 1000:+.3f}rad/s\n"
            f"  roll={self.roll_mrad / 1000:+.3f} pitch={self.pitch_mrad / 1000:+.3f} "
            f"tof {tof} updates={self.tof_updates}\n"
            f"  Pxx={self.p_xx_cm2}cm2 Pyy={self.p_yy_cm2}cm2 Ptt={self.p_tt_mrad2}"
        )


def parse_nav(payload: bytes) -> NavStatus:
    if len(payload) != NAV_LEN:
        raise ValueError(f"nav payload must be {NAV_LEN} bytes, got {len(payload)}")
    flags = struct.unpack_from("<H", payload, 0)[0]
    x, y, theta, v, omega, roll, pitch = struct.unpack_from("<7h", payload, 2)
    tof0, tof1, pxx, pyy, ptt, upd0, upd1 = struct.unpack_from("<7H", payload, 16)
    return NavStatus(
        flags=flags,
        x_mm=x,
        y_mm=y,
        theta_mrad=theta,
        v_mm_s=v,
        omega_mrad_s=omega,
        roll_mrad=roll,
        pitch_mrad=pitch,
        tof_mm=(tof0, tof1),
        p_xx_cm2=pxx,
        p_yy_cm2=pyy,
        p_tt_mrad2=ptt,
        tof_updates=(upd0, upd1),
    )


def parse_status(payload: bytes) -> Status:
    if len(payload) != STATUS_LEN:
        raise ValueError(f"status payload must be {STATUS_LEN} bytes, got {len(payload)}")
    faults, flags, linear, angular, age = struct.unpack_from("<HHhhh", payload, 0)
    target = struct.unpack_from("<4h", payload, 10)
    measured = struct.unpack_from("<4h", payload, 18)
    pwm = struct.unpack_from("<4h", payload, 26)
    rx_ok, rx_bad = struct.unpack_from("<HH", payload, 34)
    cycles = struct.unpack_from("<I", payload, 38)[0]
    last_cmd = struct.unpack_from("<b", payload, 42)[0]
    return Status(
        faults=faults,
        flags=flags,
        linear_mm_s=linear,
        angular_mrad_s=angular,
        command_age_ms=age,
        target_rpm=target,
        measured_rpm=measured,
        pwm=pwm,
        rx_ok=rx_ok,
        rx_bad=rx_bad,
        control_cycles=cycles,
        last_cmd_status=last_cmd,
    )


def extract_frames(buffer: bytearray) -> Iterable[bytes]:
    """从累计缓冲中提取完整帧，返回 body（LEN 到 CRC 高字节）。"""
    while True:
        start = buffer.find(bytes([SOF1, SOF2]))
        if start < 0:
            if len(buffer) > 1:
                del buffer[:-1]
            return
        if start > 0:
            del buffer[:start]
        if len(buffer) < 3:
            return
        length = buffer[2]
        if length > MAX_PAYLOAD:
            del buffer[:1]
            continue
        total = HEADER_LEN + length + 2
        if len(buffer) < total:
            return
        frame = bytes(buffer[:total])
        del buffer[:total]
        body = frame[2:-2]
        if crc16_modbus(body) == struct.unpack_from("<H", frame, total - 2)[0]:
            yield body


def build_command(args) -> Optional[bytes]:
    if args.action == "enable":
        return build_frame(CMD_ENABLE, bytes([1, 1]))
    if args.action == "disable":
        return build_frame(CMD_ENABLE, bytes([0, 0]))
    if args.action == "stop":
        return build_frame(CMD_STOP, b"")
    if args.action == "ping":
        return build_frame(CMD_PING, b"")
    if args.action == "velocity":
        return build_frame(
            CMD_VELOCITY,
            struct.pack(
                "<hhH",
                clamp_i16(args.linear * 1000),
                clamp_i16(args.angular * 1000),
                max(0, min(65535, args.timeout)),
            ),
        )
    if args.action == "wheel":
        return build_frame(
            CMD_WHEEL,
            struct.pack("<4h", *(clamp_i16(v * 10) for v in args.rpm)),
        )
    if args.action == "pid":
        return build_frame(
            CMD_PID,
            struct.pack(
                "<BBhhh",
                args.wheel,
                0,
                clamp_i16(args.kp * 1000),
                clamp_i16(args.ki * 1000),
                clamp_i16(args.kd * 1000),
            ),
        )
    return None


def read_one_status(ser) -> Optional[Status]:
    buffer = bytearray()
    deadline = time.time() + 1.0
    while time.time() < deadline:
        chunk = ser.read(128)
        if chunk:
            buffer.extend(chunk)
            for body in extract_frames(buffer):
                if body[2] == RSP_STATUS:
                    return parse_status(body[3:])
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="四驱差速底盘串口调试工具")
    parser.add_argument("--port", help="串口号，例如 COM7；不填则只打印帧")
    parser.add_argument("--baud", type=int, default=115200)
    sub = parser.add_subparsers(dest="action", required=True)

    sub.add_parser("enable", help="使能并清除故障")
    sub.add_parser("disable", help="断使能")
    sub.add_parser("stop", help="停止")
    sub.add_parser("ping", help="心跳并立即读取状态")
    sub.add_parser("monitor", help="持续打印状态帧")

    p = sub.add_parser("velocity", help="线速度/角速度命令")
    p.add_argument("linear", type=float, help="m/s，前进为正")
    p.add_argument("angular", type=float, help="rad/s，逆时针为正")
    p.add_argument("--timeout", type=int, default=300, help="命令超时 ms")

    p = sub.add_parser("wheel", help="四轮转速命令")
    p.add_argument("rpm", type=float, nargs=4, help="M1 M2 M3 M4，rpm")

    p = sub.add_parser("pid", help="设置 PID 参数")
    p.add_argument("wheel", type=int, help="0~3 或 255 表示全部")
    p.add_argument("kp", type=float)
    p.add_argument("ki", type=float)
    p.add_argument("kd", type=float)

    args = parser.parse_args()

    if args.action == "monitor":
        if not args.port:
            parser.error("monitor 需要 --port")
        try:
            import serial
        except ImportError:
            print("缺少 pyserial，请先 pip install pyserial", file=sys.stderr)
            return 2
        buffer = bytearray()
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            print("监听中，Ctrl+C 退出...")
            while True:
                buffer.extend(ser.read(256))
                for body in extract_frames(buffer):
                    if body[2] == RSP_STATUS:
                        print(parse_status(body[3:]).format())
                    elif body[2] == RSP_NAV:
                        print(parse_nav(body[3:]).format())
                    else:
                        continue
                    print("-" * 72)

    frame = build_command(args)
    if frame is None:
        parser.error("未知命令")

    print("TX:", frame.hex(" ").upper())

    if not args.port:
        return 0

    try:
        import serial
    except ImportError:
        print("缺少 pyserial，请先 pip install pyserial", file=sys.stderr)
        return 2

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        ser.write(frame)
        ser.flush()
        if args.action in ("enable", "disable", "stop", "ping", "velocity", "wheel", "pid"):
            status = read_one_status(ser)
            if status is None:
                print("未收到状态帧")
                return 1
            print(status.format())

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
