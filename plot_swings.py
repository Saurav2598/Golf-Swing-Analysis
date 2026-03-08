#!/usr/bin/env python3
"""
Golf Swing Analysis - Plot Metrics Only

Plots golf swing metrics from JSON file.
Computes only what's necessary for plotting.

Usage:
    python plot_swing.py <path_to_json_file> [options]

Options:
    --p4 FRAME           P4 frame number (optional, computed if not provided)
    --impact FRAME       Impact frame number (optional)
    --cutoff FREQ        Lowpass filter cutoff frequency in Hz (default: 10.0)
    --no-annotate        Disable peak annotations on plots

Examples:
    python plot_swing.py data/1772542049.json
    python plot_swing.py data/1772542049.json --p4 111 --impact 145
    python plot_swing.py data/1772542049.json --cutoff 8.0 --no-annotate
"""

import sys
import json
import re
import argparse
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt


JOINT_LABELS = [
    "Pelvis", "Hip Right", "Knee Right", "Ankle Right", "Midfoot Right",
    "Foot Right", "Hip Left", "Knee Left", "Ankle Left", "Midfoot Left",
    "Foot Left", "Nose", "Spine Low", "Spine High", "Shoulder Right",
    "Elbow Right", "Wrist Right", "Hand Right", "Shoulder Left", "Elbow Left",
    "Wrist Left", "Hand Left", "Neck", "Skull Base", "Skull Top",
    "Clavicle Right", "Clavicle Left", "Ear Right", "Ear Left"
]


def load_json_clean(path):
    """Load JSON file and clean trailing commas."""
    with open(path, "r") as f:
        raw = f.read()
    raw = re.sub(r",(\s*[}\]])", r"\1", raw)
    return json.loads(raw)


def create_joint_dict(pose_data, joint_labels=None):
    """Create joint dictionary from pose data array."""
    if joint_labels is None:
        joint_labels = JOINT_LABELS
    return {
        joint_labels[i]: np.array(pose_data[i])
        for i in range(len(joint_labels))
    }


def butter_lowpass_filtfilt(x, fs, cutoff_hz, order=4):
    """Apply Butterworth lowpass filter."""
    x = np.asarray(x, dtype=float)
    if x.ndim == 1:
        x = x[:, np.newaxis]
        squeeze = True
    else:
        squeeze = False

    if cutoff_hz <= 0 or cutoff_hz >= fs / 2:
        raise ValueError(
            f"cutoff_hz must be in (0, fs/2). Got cutoff={cutoff_hz}, fs={fs}"
        )

    b, a = butter(order, cutoff_hz / (fs / 2), btype="low")
    result = filtfilt(b, a, x, axis=0)

    return result.squeeze() if squeeze else result


def normalize(v, eps=1e-12):
    """Normalize vectors."""
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.clip(n, eps, None)


def make_frame_lr_up(lr_vec, up_vec):
    """Construct orthonormal coordinate frame."""
    e_lr = normalize(lr_vec)
    e_up = normalize(up_vec)
    e_fwd = normalize(np.cross(e_up, e_lr))
    e_lr = normalize(np.cross(e_fwd, e_up))
    return np.stack([e_lr, e_fwd, e_up], axis=-1)


def yaw_from_R(R):
    """Extract yaw angle from rotation matrix."""
    yaw = np.arctan2(R[:, 1, 0], R[:, 0, 0])
    return np.degrees(np.unwrap(yaw))


def get_file_stem_from_path(file_path):
    """
    Extract filename without extension from a path.
    Example:
        C:\\Users\\Saurav\\...\\1768733281.json -> 1768733281
    """
    return Path(file_path).stem


def compute_p4_frame(joint_dict, fs):
    """Compute P4 frame (maximum shoulder rotation)."""
    ShoulderR = joint_dict["Shoulder Right"]
    ShoulderL = joint_dict["Shoulder Left"]
    SpineHigh = joint_dict["Spine High"]
    Neck = joint_dict["Neck"]

    R_thorax = make_frame_lr_up(
        lr_vec=(ShoulderR - ShoulderL),
        up_vec=(Neck - SpineHigh)
    )

    thorax_yaw = yaw_from_R(R_thorax)

    # Restrict search to first half of the frames
    T = len(thorax_yaw)
    search_end = T // 2
    y_search = thorax_yaw[:search_end]

    # Find maximum absolute rotation within this region
    local_idx = int(np.argmax(np.abs(y_search)))
    p4_frame = local_idx

    return p4_frame


def compute_impact_frame(ball_data, fs=120.0, threshold=1.0):
    """
    Compute impact frame from ball velocity.
    Impact is detected when ball speed exceeds threshold for 2 consecutive frames.

    Args:
        ball_data: List or array of ball positions (T, 3)
        fs: Sampling frequency in Hz
        threshold: Speed threshold in m/s for detecting impact (default: 1.0)

    Returns:
        impact_frame: Frame index where impact occurs, or None if not detected
    """
    ball_pos = np.array(ball_data, dtype=float)

    if len(ball_pos) < 2:
        return None

    dt = 1.0 / fs

    displacement = ball_pos[1:] - ball_pos[:-1]
    speed = np.linalg.norm(displacement, axis=1) / dt

    impact_frame = None
    for i in range(len(speed) - 1):
        if speed[i] > threshold and speed[i + 1] > threshold:
            impact_frame = i + 1
            break

    return impact_frame


def extract_club_point_array(club_data, point="end"):
    """Extract club point positions as array."""
    keys_sorted = sorted(club_data.keys(), key=lambda k: int(k))
    arr = np.array([club_data[k][point] for k in keys_sorted], dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f"Expected (T,3), got {arr.shape}")
    return arr


def plot_swing_analysis(
    joint_dict,
    club_data,
    file_id=None,
    fs=120.0,
    p4_frame=None,
    impact_frame=None,
    cutoff_hz=10.0,
    annotate=True
):
    """
    Plot golf swing analysis graphs.
    """

    # ========== COMPUTE HAND SPEED ==========
    hand_left = np.asarray(joint_dict["Hand Left"], dtype=float)
    hand_right = np.asarray(joint_dict["Hand Right"], dtype=float)
    pos_all = 0.5 * (hand_left + hand_right)
    pos_filtered = butter_lowpass_filtfilt(pos_all, fs, cutoff_hz=cutoff_hz, order=4)
    dt = 1.0 / fs
    vel = np.gradient(pos_filtered, dt, axis=0)
    hand_speed = np.linalg.norm(vel, axis=1)

    # ========== PELVIS FRAME ==========
    HipR = joint_dict["Hip Right"]
    HipL = joint_dict["Hip Left"]
    Pelvis = joint_dict["Pelvis"]
    SpineLow = joint_dict["Spine Low"]
    R_pelvis = make_frame_lr_up(
        lr_vec=HipR - HipL,
        up_vec=SpineLow - Pelvis
    )

    # ========== THORAX FRAME ==========
    ShoulderR = joint_dict["Shoulder Right"]
    ShoulderL = joint_dict["Shoulder Left"]
    SpineHigh = joint_dict["Spine High"]
    Neck = joint_dict["Neck"]
    R_thorax = make_frame_lr_up(
        lr_vec=ShoulderR - ShoulderL,
        up_vec=Neck - SpineHigh
    )

    # ========== RIGHT KNEE FRAME ==========
    # Primary axis: Knee Right -> Ankle Right
    # Secondary axis: Ankle Right -> Midfoot Right
    KneeR = joint_dict["Knee Right"]
    AnkleR = joint_dict["Ankle Right"]
    MidfootR = joint_dict["Midfoot Right"]

    R_knee = make_frame_lr_up(
        lr_vec=(AnkleR - KneeR),
        up_vec=(MidfootR - AnkleR)
    )

    # ========== YAW ANGLES ==========
    yaw_pelvis = yaw_from_R(R_pelvis)
    yaw_pelvis = yaw_pelvis - yaw_pelvis[0]

    yaw_thorax = yaw_from_R(R_thorax)
    yaw_thorax = yaw_thorax - yaw_thorax[0]

    yaw_knee = yaw_from_R(R_knee)
    yaw_knee = yaw_knee - yaw_knee[0]

    # ========== X-FACTOR ==========
    xfactor = yaw_thorax - yaw_pelvis
    xfactor = np.degrees(np.unwrap(np.radians(xfactor)))

    # ========== FILTER ANGLES ==========
    yaw_pelvis_f = butter_lowpass_filtfilt(yaw_pelvis, fs, cutoff_hz=cutoff_hz)
    yaw_thorax_f = butter_lowpass_filtfilt(yaw_thorax, fs, cutoff_hz=cutoff_hz)
    yaw_knee_f = butter_lowpass_filtfilt(yaw_knee, fs, cutoff_hz=cutoff_hz)
    xfactor_f = butter_lowpass_filtfilt(xfactor, fs, cutoff_hz=cutoff_hz)

    # ========== ANGULAR VELOCITIES ==========
    yaw_vel_pelvis = np.gradient(yaw_pelvis_f, axis=0) * fs
    yaw_vel_thorax = np.gradient(yaw_thorax_f, axis=0) * fs
    yaw_vel_xfactor = np.gradient(xfactor_f, axis=0) * fs

    # ========== CLUB ANGULAR VELOCITY ==========
    grip_pos = extract_club_point_array(club_data, point="start")
    head_pos = extract_club_point_array(club_data, point="end")

    grip_pos_f = butter_lowpass_filtfilt(grip_pos, fs, cutoff_hz=cutoff_hz, order=4)
    head_pos_f = butter_lowpass_filtfilt(head_pos, fs, cutoff_hz=cutoff_hz, order=4)

    r = head_pos_f - grip_pos_f
    r_dot = np.gradient(r, axis=0) * fs

    r_cross_rdot = np.cross(r, r_dot)
    r_norm_sq = np.sum(r * r, axis=1)
    r_norm_sq = np.clip(r_norm_sq, 1e-12, None)

    club_omega_rad_s = np.linalg.norm(r_cross_rdot, axis=1) / r_norm_sq
    club_omega_deg_s = np.degrees(club_omega_rad_s)

    # ========== FIND PEAKS FOR ANNOTATIONS ==========
    if annotate:
        T = len(yaw_pelvis_f)
        start_idx = 0
        end_idx = T // 2

        pel_local = int(np.argmax(np.abs(yaw_pelvis_f[start_idx:end_idx])))
        pel_i = start_idx + pel_local
        pel_v = yaw_pelvis_f[pel_i]

        tho_local = int(np.argmax(np.abs(yaw_thorax_f[start_idx:end_idx])))
        tho_i = start_idx + tho_local
        tho_v = yaw_thorax_f[tho_i]

        x_local = int(np.argmax(np.abs(xfactor_f[start_idx:end_idx])))
        x_i = start_idx + x_local
        x_v = xfactor_f[x_i]

        shin_local = int(np.argmax(np.abs(yaw_knee_f[start_idx:end_idx])))
        sh_i = start_idx + shin_local
        sh_v = yaw_knee_f[x_i]

        pel_w_i = int(np.argmax(np.abs(yaw_vel_pelvis)))
        pel_w_v = yaw_vel_pelvis[pel_w_i]

        tho_w_i = int(np.argmax(np.abs(yaw_vel_thorax)))
        tho_w_v = yaw_vel_thorax[tho_w_i]

        x_w_i = int(np.argmax(np.abs(yaw_vel_xfactor)))
        x_w_v = yaw_vel_xfactor[x_w_i]

        hand_i = int(np.argmax(hand_speed))
        hand_v = hand_speed[hand_i]

        clubang_i = int(np.argmax(club_omega_deg_s))
        clubang_v = club_omega_deg_s[clubang_i]

    # ========== PLOT ==========
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    if file_id is not None:
        fig.suptitle(f"Golf Swing Analysis - {file_id}", fontsize=14, fontweight="bold")

    # Top: angles
    axes[0].plot(yaw_thorax_f, label=f"Thorax yaw (P4: {tho_v:.1f}°)")
    axes[0].plot(yaw_pelvis_f, label=f"Pelvis yaw (P4: {pel_v:.1f}°)")
    axes[0].plot(xfactor_f, label=f"X-factor (P4: {x_v:.1f}°)")
    axes[0].plot(yaw_knee_f, label=f"Right knee yaw (P4: {sh_v:.1f}°)")

    if p4_frame is not None:
        axes[0].axvline(
            int(p4_frame),
            linestyle="--",
            linewidth=1.5,
            color="grey",
            label=f"P4: {int(p4_frame)}"
        )
    if impact_frame is not None:
        axes[0].axvline(
            int(impact_frame),
            linestyle="--",
            linewidth=1.5,
            color="black",
            alpha=0.7,
            label=f"Impact: {int(impact_frame)}"
        )


    if annotate:
        axes[0].scatter(tho_i, tho_v, s=70, zorder=5)
        axes[0].text(tho_i, tho_v + 5, f"{tho_i}", ha="center")

        axes[0].scatter(pel_i, pel_v, s=70, zorder=5)
        axes[0].text(pel_i, pel_v + 5, f"{pel_i}", ha="center")

        axes[0].scatter(x_i, x_v, s=70, zorder=5)
        axes[0].text(x_i, x_v + 5, f"{x_i}", ha="center")

        axes[0].scatter(sh_i, sh_v, s=70, zorder=5)
        axes[0].text(sh_i, sh_v + 5, f"{sh_i}", ha="center")

    axes[0].set_title("Pelvis / Thorax / Right Knee Yaw and X-factor")
    axes[0].set_ylabel("Degrees")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    # Bottom: angular velocities + hand speed + club angular velocity
    axes[1].plot(yaw_vel_thorax, label="Thorax yaw velocity")
    axes[1].plot(yaw_vel_pelvis, label="Pelvis yaw velocity")
    axes[1].plot(yaw_vel_xfactor, label="X-factor velocity")
    axes[1].plot(hand_speed, label="Hand speed")
    axes[1].plot(club_omega_deg_s, label="Club angular velocity")

    if p4_frame is not None:
        axes[1].axvline(
            int(p4_frame),
            linestyle="--",
            linewidth=1.5,
            color="grey",
            label=f"P4: {int(p4_frame)}"
        )
    if impact_frame is not None:
        axes[1].axvline(
            int(impact_frame),
            linestyle="--",
            linewidth=1.5,
            color="black",
            alpha=0.7,
            label=f"Impact: {int(impact_frame)}"
        )

    if annotate:
        axes[1].scatter(tho_w_i, tho_w_v, s=70, zorder=5)
        axes[1].text(tho_w_i, tho_w_v + 20, f"{tho_w_i}", ha="center")

        axes[1].scatter(pel_w_i, pel_w_v, s=70, zorder=5)
        axes[1].text(pel_w_i, pel_w_v + 20, f"{pel_w_i}", ha="center")

        axes[1].scatter(x_w_i, x_w_v, s=70, zorder=5)
        axes[1].text(x_w_i, x_w_v + 20, f"{x_w_i}", ha="center")

        axes[1].scatter(hand_i, hand_v, s=70, zorder=5)
        axes[1].text(hand_i, hand_v + 2, f"{hand_i}", ha="center")

        axes[1].scatter(clubang_i, clubang_v, s=70, zorder=5)
        axes[1].text(clubang_i, clubang_v + 20, f"{clubang_i}", ha="center")

    axes[1].set_title("Yaw Angular Velocities + Hand Speed + Club Angular Velocity")
    axes[1].set_xlabel("Frame")
    axes[1].set_ylabel("deg/s | m/s")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()


def main():
    """Main entry point for CLI usage."""
    parser = argparse.ArgumentParser(
        description="Plot golf swing analysis graphs",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument("json_file", help="Path to JSON file")
    parser.add_argument("--p4", type=int, help="P4 frame (auto-computed if not provided)")
    parser.add_argument("--impact", type=int, help="Impact frame (auto-computed if not provided)")
    parser.add_argument(
        "--threshold",
        type=float,
        default=1.0,
        help="Ball speed threshold for impact detection in m/s (default: 1.0)"
    )
    parser.add_argument("--cutoff", type=float, default=10.0, help="Filter cutoff Hz (default: 10.0)")
    parser.add_argument("--no-annotate", action="store_true", help="Disable peak annotations")

    args = parser.parse_args()

    try:
        data = load_json_clean(args.json_file)
        pose_data = data["data"]["pose"]
        club_data = data["data"]["club"]
        ball_data = data["data"]["ball"]
        fs = data["configuration"]["frame_rate"]

        file_id = get_file_stem_from_path(args.json_file)
        joint_dict = create_joint_dict(pose_data)

        p4_frame = args.p4 if args.p4 is not None else compute_p4_frame(joint_dict, fs)

        if args.impact is not None:
            impact_frame = args.impact
        else:
            impact_frame = compute_impact_frame(ball_data, fs, threshold=args.threshold)
            if impact_frame is None:
                print(f"Warning: Impact not detected with threshold {args.threshold} m/s")

        print(f"File ID: {file_id}")
        print(f"P4 Frame: {p4_frame}")
        if impact_frame is not None:
            print(f"Impact Frame: {impact_frame}")
        else:
            print("Impact Frame: Not detected")
        print("Generating plot...")

        plot_swing_analysis(
            joint_dict=joint_dict,
            club_data=club_data,
            file_id=file_id,
            fs=fs,
            p4_frame=p4_frame,
            impact_frame=impact_frame,
            cutoff_hz=args.cutoff,
            annotate=not args.no_annotate
        )

    except FileNotFoundError:
        print(f"Error: File not found: {args.json_file}")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()