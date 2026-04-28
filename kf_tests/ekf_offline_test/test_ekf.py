#!/usr/bin/env python3
"""
Reusable EKF test script for any rosbag run.

Usage:
    python3 test_ekf.py run3
    python3 test_ekf.py run5

Runs: EKF with DR, EKF without DR, heading diagnostic.
Saves plots and a text summary into the run's folder (e.g. kf_tests/run3/).
Config is read from the current auv_navigation vessel_data.example.yml — no hardcoded sensor params.
"""

import sys
import os

os.environ["PYTHONUNBUFFERED"] = "1"

from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
KF_TESTS = Path(__file__).resolve().parent
REPO_ROOT = KF_TESTS.parent.parent
NAV_SRC = str(REPO_ROOT / "code_ws" / "src" / "auv_navigation")
sys.path.insert(0, NAV_SRC)

from navigation.ekf_nav import NavEKF
from navigation.nav_kinematics import eul_to_rotm, quat_to_eul, quat_to_rotm, ssa
from navigation.nav_observations import (
    jacobian_depth,
    jacobian_dr_position,
    jacobian_dvl,
    jacobian_imu_numerical,
    jacobian_range,
    predict_depth,
    predict_dr_position,
    predict_dvl_body,
    predict_imu,
    predict_range,
)
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.msg import get_types_from_msg

# ---------------------------------------------------------------------------
# Parse CLI
# ---------------------------------------------------------------------------
if len(sys.argv) < 2:
    print("Usage: python3 test_ekf.py <run_name>")
    print("  e.g. python3 test_ekf.py run3")
    sys.exit(1)

RUN_NAME = sys.argv[1]
RUNS_DIR = REPO_ROOT / "kf_tests" / "runs"
BAG_DIR = RUNS_DIR / RUN_NAME
if not BAG_DIR.is_dir():
    print(f"Error: {BAG_DIR} does not exist")
    sys.exit(1)
RESULT_DIR = KF_TESTS / f"{RUN_NAME}_result"
RESULT_DIR.mkdir(parents=True, exist_ok=True)

# ---------------------------------------------------------------------------
# Load config from auv_navigation vessel_data.example.yml
# ---------------------------------------------------------------------------
CONFIG_PATH = REPO_ROOT / "code_ws" / "src" / "auv_navigation" / "config" / "vessel_data.example.yml"
with open(CONFIG_PATH) as f:
    vessel = yaml.safe_load(f)

DT = float(vessel.get("time_step", 0.1))
POOL_DEPTH = float(vessel["pool_depth"])
DVL_DEPTH_FROM_SURFACE = float(vessel["dvl_depth_from_surface"])
SEABED_Z_NED = POOL_DEPTH - DVL_DEPTH_FROM_SURFACE
DVL_SURFACE_OFFSET = DVL_DEPTH_FROM_SURFACE
N_STATES = 15
IMU_SUBSAMPLE = 5

agent = vessel["agents"][0]
SENSORS = {}
for s in agent["sensors"]:
    SENSORS[s["sensor_type"]] = s

# ---------------------------------------------------------------------------
# Register custom message types
# ---------------------------------------------------------------------------
typestore = get_typestore(Stores.ROS2_HUMBLE)
pkg_root = REPO_ROOT / "packages"
for p in sorted((pkg_root / "sbg_ros2_driver" / "msg").glob("*.msg")):
    typestore.register(get_types_from_msg(p.read_text(), "sbg_driver/msg/" + p.stem))
for p in sorted((pkg_root / "dvl_msgs" / "msg").glob("*.msg")):
    typestore.register(get_types_from_msg(p.read_text(), "dvl_msgs/msg/" + p.stem))

# ---------------------------------------------------------------------------
# Read note
# ---------------------------------------------------------------------------
note_path = BAG_DIR / "mav_gui_note.txt"
run_note = note_path.read_text().strip() if note_path.exists() else RUN_NAME

# ---------------------------------------------------------------------------
# Load rosbag
# ---------------------------------------------------------------------------
messages = []
with Reader(BAG_DIR) as reader:
    for conn, ts_ns, raw in reader.messages():
        messages.append((ts_ns, conn.topic, conn.msgtype, raw))
messages.sort(key=lambda m: m[0])

print(f"{'=' * 60}")
print(f"  {RUN_NAME}: {run_note}")
print(f"{'=' * 60}")
print(f"Loaded {len(messages)} messages")
topics_seen = {}
for _, topic, _, _ in messages:
    topics_seen[topic] = topics_seen.get(topic, 0) + 1
for t, c in sorted(topics_seen.items()):
    print(f"  {t}: {c}")
duration_s = (messages[-1][0] - messages[0][0]) * 1e-9
print(f"  Duration: {duration_s:.1f}s")

has_dvl_dr = "/dvl/position" in topics_seen
has_ping = "/ping1d/range" in topics_seen
has_dvl_dr_sensor = "DVL_DR" in SENSORS
t0_ns = messages[0][0]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def sbg_imu_covariance(quat_msg, sensor):
    R = np.zeros((9, 9))
    ori_var = np.array([quat_msg.accuracy.x, quat_msg.accuracy.y, quat_msg.accuracy.z]) ** 2
    R[0:3, 0:3] = np.diag(ori_var)
    R[3:6, 3:6] = np.eye(3) * float(sensor["default_gyro_variance"])
    R[6:9, 6:9] = np.eye(3) * float(sensor["default_accel_variance"])
    return R


# ---------------------------------------------------------------------------
# Core EKF runner
# ---------------------------------------------------------------------------
def run_ekf(use_dvl_dr=True):
    pro_noise_cov = np.diag([0.01, 0.01, 0.01, 0.25, 0.25, 0.25])
    ekf = NavEKF(DT, n_states=N_STATES, n_inp=0, pro_noise_cov=pro_noise_cov)
    threshold = np.full(N_STATES, np.inf)

    first_imu_flag = True
    first_pos_flag = True
    dvl_dr_initialized = False
    dvl_dr_init_time = None
    dvl_dr_R_to_ned = None
    dvl_dr_origin = None
    sbg_latest_quat = None
    dvl_heading_delta = None
    n_imu_total = 0

    hist_t, hist_x, hist_P_diag = [], [], []
    dr_t, dr_ned = [], []
    dvl_vel_t, dvl_vel_v = [], []
    depth_t, depth_v = [], []
    imu_eul_t, imu_eul_v = [], []
    sbg_ht, sbg_yaw_v = [], []
    dvl_ht, dvl_yaw_v = [], []

    def dr_to_ned(raw):
        if dvl_dr_R_to_ned is None or dvl_dr_origin is None:
            return raw
        return dvl_dr_R_to_ned @ (raw - dvl_dr_origin)

    last_predict_ns = t0_ns
    predict_interval_ns = int(DT * 1e9)

    for ts_ns, topic, msgtype, raw_data in messages:
        t_sec = (ts_ns - t0_ns) * 1e-9

        while (ts_ns - last_predict_ns) >= predict_interval_ns:
            last_predict_ns += predict_interval_ns
            ekf.predict(u=np.zeros(ekf.n_inp), threshold=threshold)
            hist_t.append((last_predict_ns - t0_ns) * 1e-9)
            hist_x.append(ekf.x.flatten().copy())
            hist_P_diag.append(np.diag(ekf.P).copy())

        msg = typestore.deserialize_cdr(raw_data, msgtype)

        # ---- SBG quaternion ----
        if topic == "/sbg/ekf_quat":
            sbg_latest_quat = msg
            q = msg.quaternion
            eul = quat_to_eul(np.array([q.w, q.x, q.y, q.z]))
            sbg_ht.append(t_sec)
            sbg_yaw_v.append(np.degrees(eul[2]))
            continue

        # ---- SBG IMU ----
        if topic == "/sbg/imu_data":
            if sbg_latest_quat is None:
                continue
            n_imu_total += 1
            if n_imu_total % IMU_SUBSAMPLE != 0 and not first_imu_flag:
                continue
            sensor = SENSORS["IMU_SBG"]
            q = sbg_latest_quat.quaternion
            imu_quat = np.array([q.w, q.x, q.y, q.z])
            imu_eul = quat_to_eul(imu_quat)
            imu_omg = np.array([msg.gyro.x, msg.gyro.y, msg.gyro.z])
            imu_acc = np.array([msg.accel.x, msg.accel.y, msg.accel.z])
            imu_acc = imu_acc - quat_to_rotm(imu_quat).T @ np.array([0, 0, -9.81])
            y_imu = np.concatenate((imu_eul, imu_omg, imu_acc))[:, np.newaxis]
            r_bs_b = np.array(sensor["sensor_location"])
            Theta_bs = np.array(sensor["sensor_orientation"])
            R_imu = sbg_imu_covariance(sbg_latest_quat, sensor)
            H = jacobian_imu_numerical(ekf.x.flatten(), r_bs_b, Theta_bs, N_STATES)
            if first_pos_flag or first_imu_flag:
                ekf.x[3:6] = y_imu[0:3]
                ekf.x[9:12] = y_imu[3:6]
                ekf.x[12:15] = y_imu[6:9]
                if first_imu_flag:
                    first_imu_flag = False
            else:
                h_x = predict_imu(ekf.x.flatten(), r_bs_b, Theta_bs).reshape(-1, 1)
                ekf.update(y_imu, H, R_imu, h_x, threshold=threshold, imu_ssa=True)
            imu_eul_t.append(t_sec)
            imu_eul_v.append(np.degrees(imu_eul))
            continue

        # ---- DVL velocity ----
        if topic == "/dvl/data":
            if not msg.velocity_valid:
                continue
            sensor = SENSORS["DVL"]
            y = np.array([[msg.velocity.x], [msg.velocity.y], [msg.velocity.z]])
            r_bs_b = np.array(sensor["sensor_location"])
            Theta_bs = np.array(sensor["sensor_orientation"])
            R = np.eye(3) * float(sensor["default_velocity_variance"])
            H = jacobian_dvl(ekf.x.flatten(), r_bs_b, Theta_bs, N_STATES)
            h_x = predict_dvl_body(ekf.x.flatten(), r_bs_b, Theta_bs).reshape(-1, 1)
            if not first_imu_flag and first_pos_flag:
                ekf.x[6:9] = y
                first_pos_flag = False
            elif not first_pos_flag and not first_imu_flag:
                ekf.update(y, H, R, h_x, threshold=threshold, imu_ssa=False)
            dvl_vel_t.append(t_sec)
            dvl_vel_v.append([msg.velocity.x, msg.velocity.y, msg.velocity.z])
            continue

        # ---- DVL dead reckoning ----
        if topic == "/dvl/position":
            if not has_dvl_dr_sensor:
                continue
            sensor = SENSORS["DVL_DR"]
            y_raw = np.array([msg.position.x, msg.position.y, msg.position.z])
            r_bs_b = np.array(sensor["sensor_location"])
            Theta_bs = np.array(sensor["sensor_orientation"])

            dvl_ht.append(t_sec)
            dvl_yaw_v.append(float(msg.yaw))

            std = float(msg.pos_std)
            base = max(std ** 2, float(sensor["min_position_variance"]))
            scale = max(float(sensor["position_variance_scale"]), 1e-6)
            drift_rate = float(sensor.get("dr_drift_rate", 0.0))
            if dvl_dr_init_time is not None and drift_rate > 0:
                base += drift_rate * (t_sec - dvl_dr_init_time)
            R = np.eye(3) * (base * scale)

            if not dvl_dr_initialized:
                dvl_yaw = float(msg.yaw) * (np.pi / 180.0)
                if sbg_latest_quat is None:
                    continue
                dvl_dr_origin = y_raw.copy()
                q_init = sbg_latest_quat.quaternion
                sbg_eul_now = quat_to_eul(np.array([q_init.w, q_init.x, q_init.y, q_init.z]))
                sbg_yaw_at_init = float(sbg_eul_now[2])
                delta_psi = sbg_yaw_at_init - dvl_yaw
                c, s = np.cos(delta_psi), np.sin(delta_psi)
                dvl_dr_R_to_ned = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
                dvl_heading_delta = delta_psi
                print(f"  DVL DR init: t={t_sec:.3f}s, dvl_yaw={np.degrees(dvl_yaw):.1f}°, "
                      f"sbg_yaw={np.degrees(sbg_yaw_at_init):.1f}°, delta={np.degrees(delta_psi):.1f}°")
                dvl_dr_initialized = True
                dvl_dr_init_time = t_sec
                y_ned = dr_to_ned(y_raw)
                ekf.x[0:3] = y_ned.reshape(3, 1)
                if first_pos_flag:
                    first_pos_flag = False
            elif use_dvl_dr:
                y_ned = dr_to_ned(y_raw)
                y = y_ned.reshape(3, 1)
                if dvl_dr_R_to_ned is not None:
                    R = dvl_dr_R_to_ned @ R @ dvl_dr_R_to_ned.T
                H = jacobian_dr_position(ekf.x.flatten(), r_bs_b, Theta_bs, N_STATES)
                h_x = predict_dr_position(ekf.x.flatten(), r_bs_b, Theta_bs).reshape(-1, 1)
                ekf.update(y, H, R, h_x, threshold=threshold, imu_ssa=False)
            dr_t.append(t_sec)
            dr_ned.append(dr_to_ned(y_raw))
            continue

        # ---- Depth ----
        if topic == "/auv/depth":
            sensor = SENSORS["ARDUINO_DEPTH"]
            d = float(msg.data)
            if not np.isfinite(d):
                continue
            sign_scale = float(sensor.get("sign_scale", 1.0))
            z_off = float(sensor.get("z_offset", 0.0))
            y = np.array([[sign_scale * d + z_off - DVL_SURFACE_OFFSET]])
            r_bs_b = np.array(sensor["sensor_location"])
            Theta_bs = np.array(sensor["sensor_orientation"])
            R = np.array([[float(sensor["default_variance"])]])
            if first_pos_flag or first_imu_flag:
                ekf.x[2, 0] = float(y[0, 0])
                if first_pos_flag:
                    first_pos_flag = False
            else:
                H = jacobian_depth(ekf.x.flatten(), r_bs_b, Theta_bs, N_STATES)
                h_x = np.array([[predict_depth(ekf.x.flatten(), r_bs_b, Theta_bs)]])
                ekf.update(y, H, R, h_x, threshold=threshold, imu_ssa=False)
            depth_t.append(t_sec)
            depth_v.append(d)
            continue

        # ---- Ping range ----
        if topic == "/ping1d/range" and "PING_RANGE" in SENSORS:
            sensor = SENSORS["PING_RANGE"]
            r = float(msg.range)
            if not np.isfinite(r) or not (float(msg.min_range) <= r <= float(msg.max_range)):
                continue
            r_bs_b = np.array(sensor["sensor_location"])
            Theta_bs = np.array(sensor["sensor_orientation"])
            y = np.array([[r]])
            R = np.array([[float(sensor["default_variance"])]])
            if first_pos_flag or first_imu_flag:
                R_nb = eul_to_rotm(np.array([ekf.x[3, 0], ekf.x[4, 0], ekf.x[5, 0]]))
                ekf.x[2, 0] = float(SEABED_Z_NED - r - (R_nb @ r_bs_b)[2])
                if first_pos_flag:
                    first_pos_flag = False
            else:
                H = jacobian_range(ekf.x.flatten(), SEABED_Z_NED, r_bs_b, Theta_bs, N_STATES)
                h_x = np.array([[predict_range(ekf.x.flatten(), SEABED_Z_NED, r_bs_b, Theta_bs)]])
                ekf.update(y, H, R, h_x, threshold=threshold, imu_ssa=False)
            continue

    return {
        "hist_t": np.array(hist_t), "hist_x": np.array(hist_x), "hist_P": np.array(hist_P_diag),
        "dr_t": np.array(dr_t), "dr_pos": np.array(dr_ned) if dr_ned else np.empty((0, 3)),
        "dvl_vel_t": np.array(dvl_vel_t), "dvl_vel": np.array(dvl_vel_v) if dvl_vel_v else np.empty((0, 3)),
        "depth_t": np.array(depth_t), "depth_v": np.array(depth_v),
        "imu_eul_t": np.array(imu_eul_t), "imu_eul_v": np.array(imu_eul_v) if imu_eul_v else np.empty((0, 3)),
        "sbg_ht": np.array(sbg_ht), "sbg_yaw": np.array(sbg_yaw_v),
        "dvl_ht": np.array(dvl_ht), "dvl_yaw": np.array(dvl_yaw_v),
        "dvl_heading_delta": dvl_heading_delta,
    }


# ===================================================================
# Run tests
# ===================================================================
summary_lines = []
summary_lines.append(f"{RUN_NAME}: {run_note}")
summary_lines.append(f"Messages: {len(messages)}, Duration: {duration_s:.1f}s")
summary_lines.append(f"Topics: {topics_seen}")
summary_lines.append("")

# --- Test 1: With DVL DR ---
print(f"\n{'=' * 60}")
print("TEST 1: EKF WITH DVL DR")
print(f"{'=' * 60}")
if has_dvl_dr and not has_dvl_dr_sensor:
    print("  /dvl/position exists in the bag, but DVL_DR is not present in the current vessel config.")
    print("  Replaying without DVL DR updates so the test matches the current navigation config.")
    summary_lines.append("EKF WITH DVL DR:")
    summary_lines.append("  Skipped DVL DR updates because DVL_DR is absent from the current vessel config.")
    r_dr = run_ekf(use_dvl_dr=False)
else:
    r_dr = run_ekf(use_dvl_dr=True)
print(f"  Final pos: N={r_dr['hist_x'][-1,0]:.3f}, E={r_dr['hist_x'][-1,1]:.3f}, D={r_dr['hist_x'][-1,2]:.3f} m")
print(f"  Final euler: roll={np.degrees(r_dr['hist_x'][-1,3]):.1f}, pitch={np.degrees(r_dr['hist_x'][-1,4]):.1f}, yaw={np.degrees(r_dr['hist_x'][-1,5]):.1f} deg")
print(f"  Final 1σ: N={np.sqrt(r_dr['hist_P'][-1,0]):.3f}, E={np.sqrt(r_dr['hist_P'][-1,1]):.3f}, D={np.sqrt(r_dr['hist_P'][-1,2]):.3f} m")
has_nan = np.any(np.isnan(r_dr["hist_x"])) or np.any(np.isnan(r_dr["hist_P"]))
print(f"  NaN detected: {has_nan}")

if not (has_dvl_dr and not has_dvl_dr_sensor):
    summary_lines.append("EKF WITH DVL DR:")
summary_lines.append(f"  Final pos: N={r_dr['hist_x'][-1,0]:.3f}, E={r_dr['hist_x'][-1,1]:.3f}, D={r_dr['hist_x'][-1,2]:.3f} m")
summary_lines.append(f"  Final 1σ: N={np.sqrt(r_dr['hist_P'][-1,0]):.3f}, E={np.sqrt(r_dr['hist_P'][-1,1]):.3f}, D={np.sqrt(r_dr['hist_P'][-1,2]):.3f} m")

# --- Test 2: Without DVL DR ---
print(f"\n{'=' * 60}")
print("TEST 2: EKF WITHOUT DVL DR")
print(f"{'=' * 60}")
r_nodr = run_ekf(use_dvl_dr=False)
print(f"  Final pos: N={r_nodr['hist_x'][-1,0]:.3f}, E={r_nodr['hist_x'][-1,1]:.3f}, D={r_nodr['hist_x'][-1,2]:.3f} m")
print(f"  Final 1σ: N={np.sqrt(r_nodr['hist_P'][-1,0]):.3f}, E={np.sqrt(r_nodr['hist_P'][-1,1]):.3f}, D={np.sqrt(r_nodr['hist_P'][-1,2]):.3f} m")

summary_lines.append("")
summary_lines.append("EKF WITHOUT DVL DR:")
summary_lines.append(f"  Final pos: N={r_nodr['hist_x'][-1,0]:.3f}, E={r_nodr['hist_x'][-1,1]:.3f}, D={r_nodr['hist_x'][-1,2]:.3f} m")
summary_lines.append(f"  Final 1σ: N={np.sqrt(r_nodr['hist_P'][-1,0]):.3f}, E={np.sqrt(r_nodr['hist_P'][-1,1]):.3f}, D={np.sqrt(r_nodr['hist_P'][-1,2]):.3f} m")

# --- Comparison vs DVL DR ---
dr_t = r_dr["dr_t"]
dr_pos = r_dr["dr_pos"]

summary_lines.append("")
if len(dr_t) > 0:
    print(f"\n{'=' * 60}")
    print("COMPARISON vs DVL DR")
    print(f"{'=' * 60}")
    summary_lines.append("COMPARISON vs DVL DR:")
    for label, res in [("With DR", r_dr), ("No DR", r_nodr)]:
        est = np.zeros((len(dr_t), 3))
        for i, t in enumerate(dr_t):
            est[i] = res["hist_x"][np.argmin(np.abs(res["hist_t"] - t)), :3]
        diff = est - dr_pos
        ne_rms = np.sqrt(np.mean(diff[:, 0] ** 2 + diff[:, 1] ** 2))
        line = f"  {label}: NE RMS={ne_rms:.4f}m, final N={diff[-1,0]:.4f}m, final E={diff[-1,1]:.4f}m"
        print(line)
        summary_lines.append(line)
else:
    print("\n  No DVL DR data — skipping position comparison")
    summary_lines.append("No DVL DR data in this bag.")

# --- Heading diagnostic ---
sbg_yaw = r_dr["sbg_yaw"]
dvl_yaw_raw = r_dr["dvl_yaw"]
sbg_ht = r_dr["sbg_ht"]
dvl_ht = r_dr["dvl_ht"]
delta = r_dr["dvl_heading_delta"]
yaw_drift = None
drift_rate = 0.0

summary_lines.append("")
if len(dvl_ht) > 0 and delta is not None:
    dvl_yaw_mag = dvl_yaw_raw + np.degrees(delta)
    yaw_drift = np.zeros(len(dvl_ht))
    for i, t in enumerate(dvl_ht):
        d = sbg_yaw[np.argmin(np.abs(sbg_ht - t))] - dvl_yaw_mag[i]
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        yaw_drift[i] = d
    if dvl_ht[-1] > dvl_ht[0]:
        drift_rate = (yaw_drift[-1] - yaw_drift[0]) / ((dvl_ht[-1] - dvl_ht[0]) / 60)
    print(f"\n{'=' * 60}")
    print("HEADING DIAGNOSTIC")
    print(f"{'=' * 60}")
    print(f"  DVL→NED delta: {np.degrees(delta):.2f}°")
    print(f"  Heading drift: mean={np.mean(yaw_drift):.2f}°, range=[{np.min(yaw_drift):.2f}°, {np.max(yaw_drift):.2f}°]")
    print(f"  Drift rate: {drift_rate:.3f} °/min")
    summary_lines.append("HEADING DIAGNOSTIC:")
    summary_lines.append(f"  DVL→NED delta: {np.degrees(delta):.2f}°")
    summary_lines.append(f"  Drift rate: {drift_rate:.3f} °/min")


# ===================================================================
# PLOTS
# ===================================================================
print("\nGenerating plots...", flush=True)

# ---- Main results (2x2) ----
fig, axes = plt.subplots(2, 2, figsize=(16, 14))
fig.suptitle(f"{RUN_NAME}: {run_note}", fontsize=14)

ax = axes[0, 0]
if len(dr_pos) > 0:
    ax.plot(dr_pos[:, 1], dr_pos[:, 0], "k-", lw=1.5, label="DVL DR")
ax.plot(r_dr["hist_x"][:, 1], r_dr["hist_x"][:, 0], "b-", lw=1.2, label="EKF (with DR)")
ax.plot(r_nodr["hist_x"][:, 1], r_nodr["hist_x"][:, 0], "r--", lw=1, label="EKF (no DR)")
ax.plot(0, 0, "go", ms=10, label="Start")
ax.set_xlabel("East (m)")
ax.set_ylabel("North (m)")
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)
ax.set_title("Top-down trajectory")
ax.set_aspect("equal")

ax = axes[0, 1]
ax.plot(r_dr["hist_t"], r_dr["hist_x"][:, 0], "b-", lw=1, label="EKF N (DR)")
ax.plot(r_dr["hist_t"], r_dr["hist_x"][:, 1], "r-", lw=1, label="EKF E (DR)")
if len(dr_t) > 0:
    ax.plot(dr_t, dr_pos[:, 0], "b--", lw=0.7, alpha=0.6, label="DVL DR N")
    ax.plot(dr_t, dr_pos[:, 1], "r--", lw=0.7, alpha=0.6, label="DVL DR E")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Position (m)")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)
ax.set_title("Position (NED) vs time")

ax = axes[1, 0]
ax.plot(r_dr["hist_t"], np.degrees(r_dr["hist_x"][:, 5]), "g-", lw=1, label="EKF Yaw")
if len(r_dr["imu_eul_v"]) > 0:
    ax.scatter(r_dr["imu_eul_t"], r_dr["imu_eul_v"][:, 2], s=2, alpha=0.3, c="gray", label="SBG Yaw")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Yaw (deg)")
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)
ax.set_title("Yaw over time")

ax = axes[1, 1]
ax.plot(r_dr["hist_t"], r_dr["hist_x"][:, 6], label="u (fwd)")
ax.plot(r_dr["hist_t"], r_dr["hist_x"][:, 7], label="v (stbd)")
ax.plot(r_dr["hist_t"], r_dr["hist_x"][:, 8], label="w (down)")
if len(r_dr["dvl_vel"]) > 0:
    ax.scatter(r_dr["dvl_vel_t"], r_dr["dvl_vel"][:, 0], s=2, alpha=0.3, label="DVL u")
    ax.scatter(r_dr["dvl_vel_t"], r_dr["dvl_vel"][:, 1], s=2, alpha=0.3, label="DVL v")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Velocity (m/s)")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)
ax.set_title("Body Velocity")

plt.tight_layout()
out_results = RESULT_DIR / "results.png"
plt.savefig(out_results, dpi=150)
plt.close(fig)
print(f"  Saved: {out_results}")

# ---- Heading diagnostic ----
if yaw_drift is not None:
    fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5))
    fig2.suptitle(f"{RUN_NAME}: Heading Diagnostic", fontsize=14)

    ax = axes2[0]
    ax.plot(sbg_ht, sbg_yaw, "b-", lw=0.8, label="SBG yaw (magnetic)")
    ax.plot(dvl_ht, dvl_yaw_mag, "r-", lw=0.8, label="DVL yaw (converted)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Yaw (deg)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_title("SBG vs DVL heading (both magnetic NED)")

    ax = axes2[1]
    ax.plot(dvl_ht, yaw_drift, "m-", lw=0.8)
    ax.axhline(0, color="k", lw=0.5, ls=":")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("SBG − DVL heading (deg)")
    ax.grid(True, alpha=0.3)
    ax.set_title(f"Heading drift (rate={drift_rate:.2f}°/min)")

    plt.tight_layout()
    out_heading = RESULT_DIR / "heading.png"
    plt.savefig(out_heading, dpi=150)
    plt.close(fig2)
    print(f"  Saved: {out_heading}")

# ---- Save summary text ----
out_summary = RESULT_DIR / "summary.txt"
out_summary.write_text("\n".join(summary_lines) + "\n")
print(f"  Saved: {out_summary}")

print("\nDone.")
