#!/usr/bin/env python3
"""用固定发包速率测 W5500 UDP 回显与 20 ms 内的到达率。"""

import argparse
import math
import select
import socket
import statistics
import time

from w5500_probe import frame


def measure(rate, seconds, offset):
    count = round(rate * seconds)
    sent, replies = {}, {}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("192.168.1.50", 0))
        sock.connect(("192.168.1.199", 5000))
        sock.setblocking(False)
        begin = time.monotonic()
        for i in range(count + 1):
            deadline = begin + (i / rate if i < count else seconds + 0.6)
            while (wait := deadline - time.monotonic()) > 0:
                if not select.select([sock], [], [], wait)[0]:
                    break
                try:
                    data = sock.recv(64)
                except ConnectionRefusedError:
                    continue
                received = time.monotonic()
                if data in sent and data not in replies:
                    replies[data] = (received - sent[data]) * 1000
            if i < count:
                data = frame(offset + i)
                sent[data] = time.monotonic()
                sock.send(data)

    rtt = sorted(replies.values())
    line = (f"{rate}/s, {seconds:g}s: {len(rtt)}/{count}; "
            f"within20ms={sum(value <= 20 for value in rtt)}/{count}")
    if rtt:
        line += (f"; RTT median={statistics.median(rtt):.2f}ms"
                 f" p95={rtt[math.ceil(0.95 * len(rtt)) - 1]:.2f}ms max={rtt[-1]:.2f}ms")
    print(line, flush=True)
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rates", type=int, nargs="+", default=[100, 150, 200, 300, 500, 100])
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--settle", type=float, default=1, help="两档之间留给队列排空的秒数")
    args = parser.parse_args()
    if (not math.isfinite(args.seconds) or args.seconds <= 0 or
            not math.isfinite(args.settle) or args.settle < 0 or
            min(args.rates) < 1 or min(round(rate * args.seconds) for rate in args.rates) < 1):
        parser.error("请提供正速率、有效的持续时间和非负的等待时间")
    offset = 0
    for rate in args.rates:
        time.sleep(args.settle)
        offset += measure(rate, args.seconds, offset)


if __name__ == "__main__":
    main()
