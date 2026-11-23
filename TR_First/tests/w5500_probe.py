#!/usr/bin/env python3
"""从 Jetson 有线口检查 TR 主控 W5500 回显。"""

import argparse
import socket
import struct
import time


def frame(sequence):
    body = b"\xa5\x01" + struct.pack(">I", sequence)
    return body + bytes((sum(body) & 0xFF, 0x5A))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=100)
    args = parser.parse_args()
    if args.count < 1:
        parser.error("--count 至少为 1")

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("192.168.1.50", 0))
        sock.connect(("192.168.1.199", 5000))
        sock.settimeout(0.2)
        results = []
        for sequence in range(args.count):
            probe = frame(sequence)
            start = time.monotonic()
            sock.send(probe)
            try:
                if sock.recv(64) == probe:
                    results.append((time.monotonic() - start) * 1000)
            except (TimeoutError, ConnectionRefusedError):
                pass
            time.sleep(max(0.0, 0.05 - (time.monotonic() - start)))

        if results:
            # 逐项破坏边界和校验；坏包不得回显，也不得堵住下一条合法包。
            probe = frame(0xFFFFFFFF)
            for bad in (probe[:-1], b"\x00" + probe[1:], probe[:-2] + bytes((probe[-2] ^ 1, probe[-1])),
                        probe[:-1] + b"\x00"):
                sock.send(bad)
                try:
                    sock.recv(64)
                    print("主控错误地回显了非法帧")
                    return 1
                except TimeoutError:
                    pass
            sock.send(probe)
            try:
                if sock.recv(64) != probe:
                    print("非法帧之后的回显不匹配")
                    return 1
            except TimeoutError:
                print("非法帧之后未收到合法回显")
                return 1

    print(f"有效回显：{len(results)}/{args.count}")
    if results:
        print(f"往返：最小 {min(results):.2f} ms，平均 {sum(results) / len(results):.2f} ms，"
              f"最大 {max(results):.2f} ms；非法帧未回显")
    return 0 if len(results) == args.count else 1


if __name__ == "__main__":
    raise SystemExit(main())
