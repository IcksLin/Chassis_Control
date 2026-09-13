#!/usr/bin/env python3
"""统计 ESP32 IMU 网页接口的磁力计/姿态扰动。

示例：python3 tools/imu_stats.py --url http://192.168.43.19 --duration 30 --label motor_off
"""
import argparse, csv, json, math, statistics, time
from urllib.request import urlopen

def main():
    p = argparse.ArgumentParser(description="Collect and summarize /api/attitude")
    p.add_argument("--url", default="http://192.168.43.19")
    p.add_argument("--duration", type=float, default=30)
    p.add_argument("--rate", type=float, default=20, help="采样频率 Hz")
    p.add_argument("--label", default="sample")
    a = p.parse_args(); endpoint = a.url.rstrip("/") + "/api/attitude"
    rows=[]; period=1.0/a.rate; end=time.monotonic()+a.duration
    while time.monotonic()<end:
        t=time.time()
        try:
            with urlopen(endpoint, timeout=2) as r: d=json.load(r)
            if d.get("ready"):
                row={"time":t, **{k:d.get(k, float("nan")) for k in ("mx","my","mz","roll","pitch","yaw")}}
                row["norm"]=math.sqrt(sum(row[k]*row[k] for k in ("mx","my","mz"))); rows.append(row)
        except Exception as e: print(f"warning: {e}")
        time.sleep(period)
    if not rows: raise SystemExit("未收到有效数据")
    out=f"imu_{a.label}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    with open(out,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys()); w.writeheader(); w.writerows(rows)
    print(f"samples={len(rows)} csv={out}")
    for k in ("mx","my","mz","norm","roll","pitch","yaw"):
        v=[r[k] for r in rows]; print(f"{k:>5}: mean={statistics.fmean(v): .6f} std={statistics.stdev(v) if len(v)>1 else 0: .6f} min={min(v): .6f} max={max(v): .6f} p2p={max(v)-min(v): .6f}")

if __name__ == "__main__": main()
