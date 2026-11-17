# 长时监视 E103 模块：诊断固件把异常瞬间的 AT 报文存在 RAM 环形缓冲里
# 用法:
#   python wifi_trace.py --dump        只看一次缓冲内容（验证用）
#   python wifi_trace.py --hours 2     挂着盯 2 小时，异常实时打印并存日志
import argparse
import datetime
import sys
import time

from pyocd.core.helpers import ConnectHelper

TRACE = 0x20004D00
WR = 0x200001D4
CNT = {"连上": 0x200001D8, "掉线": 0x200001DC, "连失败": 0x200001E0}
N = 512
REC = 40

EVT = {0: "连接成功", 1: "没拿到IP", 2: "建socket没回OK", 3: "socket状态不对", 4: "连接断开",
       0x11: "发送串口出错", 0x13: "发送超时(没回K/R)"}


def open_session():
    s = ConnectHelper.session_with_chosen_probe(
        target_override="cortex_m", options={"frequency": 2000000})
    s.open()
    if s.target.get_state().name != "RUNNING":
        s.target.resume()
    return s


def read_records(target, wr, count):
    """读出最近 count 条记录，按写入顺序返回"""
    out = []
    start = wr - count
    for i in range(start, wr):
        addr = TRACE + (i % N) * REC
        b = target.read_memory_block8(addr, REC)
        tick = b[0] | (b[1] << 8)
        d = b[2]
        n = b[3]
        if n > 36:
            continue
        out.append((tick, d, bytes(b[4:4 + n])))
    return out


def show(rec):
    tick, d, data = rec
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    if d == 3:
        sr = data[0] | (data[1] << 8) if len(data) >= 2 else 0
        flags = ""
        for bit, name in ((0x08, "ORE"), (0x02, "FE"), (0x04, "NF"), (0x01, "PE")):
            if sr & bit:
                flags += " " + name
        line = "[%s] 串口SR=0x%04X%s" % (ts, sr, flags if flags else " 正常")
    elif d == 2:
        code = data[0] if data else 0xFF
        line = "[%s] 事件 %s (t=%d)" % (ts, EVT.get(code, "?"), tick)
    else:
        txt = data.decode("ascii", "replace").replace("\r", "\\r").replace("\n", "\\n")
        line = "[%s] %s %s" % (ts, "RX<" if d == 0 else "TX>", txt)
    print(line, flush=True)
    return line


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hours", type=float, default=2.0)
    ap.add_argument("--dump", action="store_true")
    ap.add_argument("--poll", type=float, default=1.0)
    args = ap.parse_args()

    session = open_session()
    target = session.target

    if args.dump:
        wr = target.read32(WR)
        print("写入条数=%d" % wr)
        n = min(wr, N)
        for r in read_records(target, wr, n):
            show(r)
        session.close()
        return

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log = open("wifi_trace_%s.log" % stamp, "a", encoding="utf-8", buffering=1)
    last_wr = target.read32(WR)
    last_cnt = {k: target.read32(v) for k, v in CNT.items()}
    print("开始监视，当前条数=%d，计数=%s" % (last_wr, last_cnt), flush=True)
    end = time.time() + args.hours * 3600

    while time.time() < end:
        try:
            wr = target.read32(WR)
            cnt = {k: target.read32(v) for k, v in CNT.items()}
            for k in CNT:
                if cnt[k] != last_cnt[k]:
                    msg = "计数 %s: %d -> %d" % (k, last_cnt[k], cnt[k])
                    print(msg, flush=True)
                    log.write(msg + "\n")
            last_cnt = cnt

            if wr < last_wr:            # 板子复位过
                print("检测到复位，重新同步", flush=True)
                last_wr = wr
            elif wr > last_wr:
                delta = wr - last_wr
                if delta > N:
                    print("丢了 %d 条记录" % (delta - N), flush=True)
                    delta = N
                for r in read_records(target, wr, delta):
                    log.write(show(r) + "\n")
                last_wr = wr
        except Exception as e:          # USB 抖动就重连
            print("读取出错: %s，3 秒后重连" % e, flush=True)
            try:
                session.close()
            except Exception:
                pass
            time.sleep(3)
            session = open_session()
            target = session.target
            last_wr = target.read32(WR)
            last_cnt = {k: target.read32(v) for k, v in CNT.items()}

        time.sleep(args.poll)

    session.close()
    log.close()
    print("监视结束", flush=True)


if __name__ == "__main__":
    sys.exit(main())
