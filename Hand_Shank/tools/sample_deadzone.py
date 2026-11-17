# 摇杆死区采样：用 CMSIS-DAP 直接读 RAM 里的 adcBuffer[4]，统计静止噪声
# 用法: python sample_deadzone.py -t 20        （20 秒，摇杆别碰）
# 依赖: pip install pyocd
import argparse
import csv
import statistics
import time

from pyocd.core.helpers import ConnectHelper

# mymain.c: int16_t adcBuffer[4]，排在前面的变量不变时地址不变
ADC_ADDR = 0x2000016A
NAMES = ("adcBuffer[0]", "adcBuffer[1]", "adcBuffer[2]", "adcBuffer[3]")
# 现在代码里用的中心和死区（Rock_trans 里的参数）
CODE_CENTER = (2068, 2050, 0, 0)


def read_channels(target):
    b = target.read_memory_block8(ADC_ADDR, 8)
    return (b[0] | b[1] << 8, b[2] | b[3] << 8, b[4] | b[5] << 8, b[6] | b[7] << 8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--seconds", type=float, default=20.0)
    ap.add_argument("-r", "--rate", type=float, default=200.0, help="采样率 Hz")
    ap.add_argument("-o", "--out", default="deadzone_samples.csv")
    args = ap.parse_args()

    session = ConnectHelper.session_with_chosen_probe(
        target_override="cortex_m", options={"frequency": 2000000})
    session.open()
    target = session.target
    if target.get_state().name != "RUNNING":
        target.resume()
        time.sleep(0.2)

    rows = []
    period = 1.0 / args.rate
    start = time.perf_counter()
    next_t = start + 0.5          # 丢掉前 0.5 秒，避开刚连上时的抖动
    while True:
        now = time.perf_counter()
        if now - start >= args.seconds + 0.5:
            break
        if now < next_t:
            time.sleep(min(next_t - now, 0.002))
            continue
        rows.append((now - start,) + read_channels(target))
        next_t += period

    session.close()

    # 静止判定：峰峰值超过 300 认为采样期间被动过，这一路作废
    stats = []
    for i in range(4):
        v = [r[i + 1] for r in rows]
        stats.append({
            "name": NAMES[i],
            "mean": statistics.fmean(v),
            "min": min(v),
            "max": max(v),
            "sd": statistics.pstdev(v),
            "n": len(v),
        })

    print("采样 %d 点，%.1f 秒" % (len(rows), args.seconds))
    print("%-13s %7s %6s %6s %7s %8s %s" % ("通道", "均值", "最小", "最大", "峰峰", "标准差", "状态"))
    for s in stats:
        moved = (s["max"] - s["min"]) > 300
        print("%-13s %7.1f %6d %6d %7d %8.2f %s" % (
            s["name"], s["mean"], s["min"], s["max"],
            s["max"] - s["min"], s["sd"], "动过,作废" if moved else "静止"))

    print("\n建议值（按 均值±3σ 取整，再留 5 个码余量）:")
    for i in (0, 1):
        s = stats[i]
        if (s["max"] - s["min"]) > 300:
            print("  %s: 采样期间动过，重新采" % s["name"])
            continue
        center = round(s["mean"])
        half = max(abs(s["max"] - s["mean"]), abs(s["mean"] - s["min"]), 3 * s["sd"])
        dead = int(round(half)) + 5
        print("  %s: center=%d  deadzone=%d   (代码里现在是 center=%d  deadzone=30)"
              % (s["name"], center, dead, CODE_CENTER[i]))

    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t"] + list(NAMES))
        for r in rows:
            w.writerow(["%.4f" % r[0]] + list(r[1:]))
    print("\n原始数据: %s" % args.out)


if __name__ == "__main__":
    main()
