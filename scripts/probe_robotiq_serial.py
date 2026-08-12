#!/usr/bin/env python3
"""Robotiq 2F-85 RS-485 链路诊断：绕开整个 ROS，直接发 Modbus RTU 帧。

用来区分"ROS/驱动配置错"和"物理链路不通"。夹爪亮红灯（已上电但通信不通）时
跑这个，能进一步把范围缩小到 接线 / 波特率 / 从站地址 / 适配器方向控制。

  python3 scripts/probe_robotiq_serial.py                 # 默认全套
  python3 scripts/probe_robotiq_serial.py --port /dev/ttyUSB0
  python3 scripts/probe_robotiq_serial.py --quick         # 只测出厂默认组合

出厂默认：115200 8N1，从站地址 9，Modbus RTU。
"""
import argparse
import collections
import glob
import sys
import time

try:
    import serial
    import serial.rs485  # 必须显式 import，`import serial` 不会带上这个子模块
except ImportError:
    sys.exit("需要 pyserial：pip install pyserial")

DEFAULT_PORT_GLOB = "/dev/serial/by-id/*RS-485*"
STATUS_REG = 0x07D0  # 2000: 夹爪状态输入寄存器起始地址


def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build(slave: int, func: int, addr: int, count: int) -> bytes:
    frame = bytes([slave, func, addr >> 8, addr & 0xFF, count >> 8, count & 0xFF])
    return frame + crc16(frame)


def transact(ser, frame, settle=0.05):
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(frame)
    ser.flush()
    time.sleep(settle)
    return ser.read(64)


def open_port(port, baud, parity, stopbits, rs485):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = parity
    ser.stopbits = stopbits
    ser.timeout = 0.3
    if rs485:
        ser.rs485_mode = serial.rs485.RS485Settings(
            rts_level_for_tx=True, rts_level_for_rx=False
        )
    ser.open()
    return ser


def describe(reply: bytes, frame: bytes) -> str:
    if not reply:
        return "无应答"
    if reply.startswith(frame):
        extra = reply[len(frame):]
        if not extra:
            return f"仅回显自己发的帧（半双工回显，夹爪未应答）: {reply.hex(' ')}"
        return f"回显 + 应答: {extra.hex(' ')}  ★"
    return f"应答 {len(reply)} 字节: {reply.hex(' ')}  ★"


def scan_slaves(port, baud=115200):
    """扫全部从站地址。地址被改过时这是唯一能找回来的办法。"""
    print(f"\n[1] 从站地址扫描 (1..247) @ {baud} 8N1")
    found = []
    try:
        ser = open_port(port, baud, serial.PARITY_NONE, serial.STOPBITS_ONE, False)
    except Exception as exc:
        print(f"    打开串口失败: {exc}")
        return found
    with ser:
        for slave in range(1, 248):
            frame = build(slave, 0x04, STATUS_REG, 3)
            reply = transact(ser, frame, settle=0.02)
            if reply and not reply == frame:
                print(f"    slave={slave}: {describe(reply, frame)}")
                found.append(slave)
    print("    未发现任何从站" if not found else f"    发现: {found}")
    return found


def scan_line_settings(port, slave=9):
    """扫波特率 x 校验位。出厂是 115200 8N1，被改过时才需要这一步。"""
    print(f"\n[2] 波特率 x 校验位扫描 (slave={slave})")
    hits = []
    parities = [("N", serial.PARITY_NONE), ("E", serial.PARITY_EVEN), ("O", serial.PARITY_ODD)]
    for baud in (115200, 57600, 38400, 19200, 9600):
        for pname, parity in parities:
            try:
                ser = open_port(port, baud, parity, serial.STOPBITS_ONE, False)
            except Exception as exc:
                print(f"    {baud:>6} 8{pname}1: 打开失败 {exc}")
                continue
            with ser:
                frame = build(slave, 0x04, STATUS_REG, 3)
                reply = transact(ser, frame)
            if reply:
                print(f"    {baud:>6} 8{pname}1: {describe(reply, frame)}")
                hits.append((baud, pname))
    if not hits:
        print("    全部无应答")
    return hits


def check_rs485_rts(port, slave=9, baud=115200):
    """有些适配器不做自动收发切换，需要 RTS 控制方向。"""
    print(f"\n[3] RTS 方向控制模式 (slave={slave} @ {baud} 8N1)")
    try:
        ser = open_port(port, baud, serial.PARITY_NONE, serial.STOPBITS_ONE, True)
    except Exception as exc:
        print(f"    该适配器不支持内核 RS485 模式（多数 FTDI 是自动方向，属正常）: {exc}")
        return False
    with ser:
        frame = build(slave, 0x04, STATUS_REG, 3)
        reply = transact(ser, frame)
    print(f"    {describe(reply, frame)}")
    return bool(reply)


def check_functions(port, slave=9, baud=115200):
    """夹爪同时支持 0x03/0x04 读；两个都试，排除功能码差异。"""
    print(f"\n[4] 功能码 0x03 / 0x04 (slave={slave} @ {baud} 8N1)")
    try:
        ser = open_port(port, baud, serial.PARITY_NONE, serial.STOPBITS_ONE, False)
    except Exception as exc:
        print(f"    打开串口失败: {exc}")
        return
    with ser:
        for func in (0x03, 0x04):
            frame = build(slave, func, STATUS_REG, 3)
            print(f"    FC 0x{func:02X}: {describe(transact(ser, frame), frame)}")


def check_tx_clocking(port):
    """确认 USB-UART 真的在按波特率把字节推出去，而不是吞掉。

    发 N 字节的耗时应当约等于 N*10/baud。若远小于该值，说明数据没真正出串口，
    问题在适配器/驱动，而不在夹爪那一侧。
    """
    print("\n[0] 适配器发送能力")
    for baud, count in ((9600, 960), (115200, 11520)):
        try:
            ser = open_port(port, baud, serial.PARITY_NONE, serial.STOPBITS_ONE, False)
        except Exception as exc:
            print(f"    {baud:>6}: 打开失败 {exc}")
            continue
        with ser:
            start = time.perf_counter()
            ser.write(b"\x55" * count)
            ser.flush()
            elapsed = time.perf_counter() - start
        expected = count * 10 / baud
        verdict = "正常" if elapsed > expected * 0.7 else "!! 远快于理论值，字节没真正发出"
        print(f"    {baud:>6}: 发 {count} 字节耗时 {elapsed:.3f}s / 理论 {expected:.3f}s -> {verdict}")


def check_idle_line(port, seconds=3.0):
    """只听不发，看总线空闲电平。

    完全静默 = 收发器看到的是 mark（正常空闲，或线悬空被失效保护偏置拉住）。
    持续 0x00 / 乱码流 = 线上有被错误驱动的电平，典型于 A/B 接反且对端在驱动总线。

    注意：静默并不能排除 A/B 接反 —— 总线空闲时没人驱动，接反也照样静默。
    """
    print(f"\n[5] 总线空闲电平（只听 {seconds:.0f}s，不发送）")
    for baud in (115200, 9600):
        try:
            ser = open_port(port, baud, serial.PARITY_NONE, serial.STOPBITS_ONE, False)
        except Exception as exc:
            print(f"    {baud:>6}: 打开失败 {exc}")
            continue
        with ser:
            ser.reset_input_buffer()
            deadline = time.time() + seconds
            buf = b""
            while time.time() < deadline:
                buf += ser.read(4096)
        if not buf:
            print(f"    {baud:>6}: 完全静默（空闲电平正常，或线悬空）")
        else:
            hist = collections.Counter(buf).most_common(4)
            top = ", ".join(f"0x{b:02X}x{c}" for b, c in hist)
            print(f"    {baud:>6}: 收到 {len(buf)} 字节噪声 -> {top}  ← 疑似 A/B 接反")


def resolve_port(explicit):
    if explicit:
        return explicit
    matches = sorted(glob.glob(DEFAULT_PORT_GLOB))
    if matches:
        return matches[0]
    fallback = sorted(glob.glob("/dev/ttyUSB*"))
    if fallback:
        print(f"!! 没找到 by-id 里的 RS-485 设备，回落到 {fallback[0]}")
        print("   注意 GELLO 的 Dynamixel 也是 FTDI，别测错设备。用 --port 显式指定更稳。")
        return fallback[0]
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None, help="串口路径，默认自动找 by-id 里的 RS-485")
    parser.add_argument("--slave", type=int, default=9, help="从站地址，出厂默认 9")
    parser.add_argument("--quick", action="store_true", help="只测出厂默认组合，跳过全扫描")
    args = parser.parse_args()

    port = resolve_port(args.port)
    if port is None:
        sys.exit("找不到任何串口设备。RS-485 适配器插上了吗？ls -l /dev/serial/by-id/")
    print(f"端口: {port}")
    print("提示: 任何带 ★ 的行都说明夹爪在总线上应答了。\n" + "=" * 68)

    check_tx_clocking(port)
    check_functions(port, args.slave)
    if not args.quick:
        scan_line_settings(port, args.slave)
        check_rs485_rts(port, args.slave)
        check_idle_line(port)
        scan_slaves(port)

    print("\n" + "=" * 68)
    print("""全程无应答、且夹爪亮红灯（说明 24V 正常）时，按可能性排序：

  1. RS-485 的 A/B 接反 —— 最常见。不同厂商对 A/B 的定义相反，
     把两根信号线对调再测一次，代价 10 秒，收益最高。
  2. 信号地没接 —— RS-485 需要共地参考。只接了 A/B、没把 24V 电源的 0V
     和适配器的 GND 连起来时，通信会完全不通或极不稳定。
  3. 线序/断线 —— 万用表量一下夹爪端到适配器端每根线的通断。
  4. 从站地址或波特率被改过 —— 上面的扫描若仍无应答，基本可排除。

夹爪亮红灯本身就证明它在等一个它能听懂的 Modbus 帧，
所以问题一定在"帧到不了它"或"它的回复到不了我们"，也就是物理层。""")


if __name__ == "__main__":
    main()
