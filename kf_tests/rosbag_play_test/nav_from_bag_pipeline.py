#!/usr/bin/env python3
"""
Run end-to-end live navigation generation from a rosbag.

Pipeline:
  1) Start navigation_node with use_sim_time:=true and remapped output topics
  2) Start ros2 bag record for generated navigation topics only
  3) Play input bag with --clock
  4) Stop recorder/navigation after playback
  5) Plot generated /navigation/odometry XY path and write summary
  6) If input bag contains onboard nav odometry, overlay and compare
"""

from __future__ import annotations

import argparse
import os
import signal
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import matplotlib
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _resolve_input_bag(arg: str, bag_root: Path) -> Path:
    p = Path(arg)
    if p.is_dir():
        return p.resolve()
    p2 = (bag_root / arg).resolve()
    if p2.is_dir():
        return p2
    raise FileNotFoundError(f"Input bag not found: {arg}")


def _start_proc(cmd: list[str], name: str) -> subprocess.Popen:
    print(f"[pipeline] start {name}: {' '.join(cmd)}")
    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )


def _terminate_proc(proc: Optional[subprocess.Popen], name: str, timeout_s: float = 3.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    print(f"[pipeline] stopping {name} (pid={proc.pid})")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except Exception:
        try:
            proc.terminate()
        except Exception:
            return
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if proc.poll() is not None:
            return
        time.sleep(0.05)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def _drain_output(proc: Optional[subprocess.Popen], name: str, max_chars: int = 6000) -> None:
    if proc is None or proc.stdout is None:
        return
    try:
        txt = proc.stdout.read() or ""
    except Exception:
        txt = ""
    if not txt:
        return
    tail = txt[-max_chars:]
    print(f"[{name} output tail]")
    print(tail)


def _extract_nav_xy(nav_bag_dir: Path, topic: str = "/navigation/odometry") -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    t_vals = []
    n_vals = []
    e_vals = []

    with Reader(nav_bag_dir) as reader:
        msgs = []
        for conn, ts_ns, raw in reader.messages():
            if conn.topic == topic:
                msgs.append((ts_ns, conn.msgtype, raw))
        msgs.sort(key=lambda m: m[0])
        if not msgs:
            return np.array([]), np.array([]), np.array([])
        t0_ns = msgs[0][0]
        for ts_ns, msgtype, raw in msgs:
            msg = typestore.deserialize_cdr(raw, msgtype)
            t_vals.append((ts_ns - t0_ns) * 1e-9)
            n_vals.append(float(msg.pose.pose.position.x))
            e_vals.append(float(msg.pose.pose.position.y))
    return np.array(t_vals), np.array(n_vals), np.array(e_vals)


def _save_outputs(
    out_dir: Path,
    run_label: str,
    t_gen: np.ndarray,
    n_gen: np.ndarray,
    e_gen: np.ndarray,
    t_ref: np.ndarray,
    n_ref: np.ndarray,
    e_ref: np.ndarray,
    ref_topic: str,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    summary_path = out_dir / "summary.txt"
    plot_path = out_dir / "nav_xy.png"

    if len(t_gen) == 0:
        summary_path.write_text(f"Run: {run_label}\nNo /navigation/odometry messages recorded.\n")
        print(f"[pipeline] no odometry samples found. summary: {summary_path}")
        return

    fig, ax = plt.subplots(figsize=(8.5, 7))
    ax.plot(e_gen, n_gen, "b-", lw=1.3, label="Generated nav odom")
    ax.plot(e_gen[0], n_gen[0], "go", ms=8, label="Generated start")
    ax.plot(e_gen[-1], n_gen[-1], "ro", ms=6, label="Generated end")
    has_ref = len(t_ref) > 0
    if has_ref:
        ax.plot(e_ref, n_ref, "m--", lw=1.1, alpha=0.9, label=f"Input bag ({ref_topic})")
        ax.plot(e_ref[0], n_ref[0], "c^", ms=6, label="Input start")
        ax.plot(e_ref[-1], n_ref[-1], "kv", ms=6, label="Input end")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(f"Navigation Path: generated vs input ({run_label})")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="best")
    plt.tight_layout()
    plt.savefig(plot_path, dpi=150)
    plt.close(fig)

    total_time = float(t_gen[-1] - t_gen[0]) if len(t_gen) > 1 else 0.0
    path_len = float(np.sum(np.hypot(np.diff(n_gen), np.diff(e_gen)))) if len(t_gen) > 1 else 0.0
    lines = [
        f"Run: {run_label}",
        f"Generated samples: {len(t_gen)}",
        f"Generated duration: {total_time:.3f} s",
        f"Generated path length (NE): {path_len:.3f} m",
        f"Generated N range: [{np.min(n_gen):.3f}, {np.max(n_gen):.3f}] m",
        f"Generated E range: [{np.min(e_gen):.3f}, {np.max(e_gen):.3f}] m",
    ]
    if has_ref:
        ref_time = float(t_ref[-1] - t_ref[0]) if len(t_ref) > 1 else 0.0
        ref_len = float(np.sum(np.hypot(np.diff(n_ref), np.diff(e_ref)))) if len(t_ref) > 1 else 0.0
        end_delta = float(np.hypot(n_gen[-1] - n_ref[-1], e_gen[-1] - e_ref[-1]))
        lines.extend(
            [
                f"Input topic: {ref_topic}",
                f"Input samples: {len(t_ref)}",
                f"Input duration: {ref_time:.3f} s",
                f"Input path length (NE): {ref_len:.3f} m",
                f"Input N range: [{np.min(n_ref):.3f}, {np.max(n_ref):.3f}] m",
                f"Input E range: [{np.min(e_ref):.3f}, {np.max(e_ref):.3f}] m",
                f"End-point delta (generated vs input): {end_delta:.3f} m",
            ]
        )
    else:
        lines.append(f"Input topic not found in input bag: {ref_topic}")
    lines.append(f"Plot: {plot_path}")
    summary_path.write_text("\n".join(lines) + "\n")
    print(f"[pipeline] saved: {plot_path}")
    print(f"[pipeline] saved: {summary_path}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate navigation topics live from bag playback and plot resulting odometry.")
    ap.add_argument("bag", help="Input bag directory path or run name under --bag-root (e.g. run5)")
    ap.add_argument("--bag-root", default=str(Path(__file__).resolve().parent.parent / "runs"), help="Root folder for run names")
    ap.add_argument("--rate", type=float, default=1.0, help="Playback rate for ros2 bag play")
    ap.add_argument("--vessel-file", default="", help="Optional vessel config path for navigation_node")
    ap.add_argument("--record-state", action="store_true", help="Also record /navigation/state")
    ap.add_argument(
        "--generated-nav-topic",
        default="/navigation/generated/odometry",
        help="Topic used for generated odometry output (default: /navigation/generated/odometry)",
    )
    ap.add_argument(
        "--generated-state-topic",
        default="/navigation/generated/state",
        help="Topic used for generated state output (default: /navigation/generated/state)",
    )
    ap.add_argument(
        "--input-nav-topic",
        default="/navigation/odometry",
        help="Reference topic in input bag for comparison overlay (default: /navigation/odometry)",
    )
    args = ap.parse_args()

    bag_root = Path(args.bag_root).resolve()
    in_bag = _resolve_input_bag(args.bag, bag_root)
    out_bag = Path(__file__).resolve().parent / f"{in_bag.name}_result"
    if out_bag.exists():
        shutil.rmtree(out_bag, ignore_errors=True)

    gen_nav_topic = args.generated_nav_topic.strip() or "/navigation/generated/odometry"
    gen_state_topic = args.generated_state_topic.strip() or "/navigation/generated/state"

    nav_cmd = [
        "ros2",
        "run",
        "auv_navigation",
        "navigation_node",
        "--ros-args",
        "-p",
        "use_sim_time:=true",
        "-r",
        f"/navigation/odometry:={gen_nav_topic}",
        "-r",
        f"/navigation/state:={gen_state_topic}",
    ]
    if args.vessel_file.strip():
        nav_cmd += ["-p", f"vessel_data_file:={args.vessel_file.strip()}"]

    rec_topics = [gen_nav_topic]
    if args.record_state:
        rec_topics.append(gen_state_topic)
    rec_cmd = ["ros2", "bag", "record", *rec_topics, "-o", str(out_bag)]
    play_cmd = ["ros2", "bag", "play", str(in_bag), "--clock", "--rate", f"{args.rate:g}"]

    nav_proc: Optional[subprocess.Popen] = None
    rec_proc: Optional[subprocess.Popen] = None
    play_proc: Optional[subprocess.Popen] = None

    try:
        nav_proc = _start_proc(nav_cmd, "navigation_node")
        time.sleep(1.0)
        rec_proc = _start_proc(rec_cmd, "bag_record")
        time.sleep(1.0)
        play_proc = _start_proc(play_cmd, "bag_play")
        print("[pipeline] waiting for playback to finish...")
        while True:
            code = play_proc.poll()
            if code is not None:
                print(f"[pipeline] bag play exited with code {code}")
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("[pipeline] interrupted by user")
    finally:
        _terminate_proc(play_proc, "bag_play")
        _terminate_proc(rec_proc, "bag_record")
        _terminate_proc(nav_proc, "navigation_node")
        _drain_output(play_proc, "bag_play")
        _drain_output(rec_proc, "bag_record")
        _drain_output(nav_proc, "navigation_node")

    print(f"[pipeline] recorded bag: {out_bag}")
    t_gen, n_gen, e_gen = _extract_nav_xy(out_bag, topic=gen_nav_topic)
    t_ref, n_ref, e_ref = _extract_nav_xy(in_bag, topic=args.input_nav_topic)
    _save_outputs(out_bag, in_bag.name, t_gen, n_gen, e_gen, t_ref, n_ref, e_ref, args.input_nav_topic)
    print("[pipeline] done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

