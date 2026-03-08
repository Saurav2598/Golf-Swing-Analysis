#!/usr/bin/env python3
"""
Golf Swing Analysis - P4 Frame and Maximum Hand Speed

Analyzes golf swing pose data from JSON files to detect:
1. P4 position (maximum shoulder rotation)
2. Maximum hand speed

Usage:
    python swing_analysis.py <path_to_json_file> [options]
    
Options:
    --metric METRIC      What to compute: p4, hand_speed, both (default: both)
    --cutoff FREQ        Lowpass filter cutoff frequency in Hz (default: 8.0)
    --frames START END   Frame range to analyze hand speed (optional)
    
Examples:
    python swing_analysis.py data/1772542049.json
    python swing_analysis.py data/1772542049.json --metric p4
    python swing_analysis.py data/1772542049.json --metric hand_speed
    python swing_analysis.py data/1772542049.json --cutoff 10.0 --frames 50 150
"""

import sys
import json
import re
import argparse
import numpy as np
from scipy.signal import butter, filtfilt


# Joint labels mapping (corresponds to pose data indices)
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
    
    # Remove trailing commas before } or ]
    raw = re.sub(r",(\s*[}\]])", r"\1", raw)
    return json.loads(raw)


def create_joint_dict(pose_data, joint_labels=None):
    """
    Create joint dictionary from pose data array.
    
    Args:
        pose_data: List of joint position arrays from JSON
        joint_labels: List of joint names (uses JOINT_LABELS if None)
        
    Returns:
        Dictionary mapping joint names to (T, 3) position arrays
    """
    if joint_labels is None:
        joint_labels = JOINT_LABELS
    
    joint_dict = {
        joint_labels[i]: np.array(pose_data[i])
        for i in range(len(joint_labels))
    }
    
    return joint_dict


def butter_lowpass_filtfilt(x, fs, cutoff_hz, order=4):
    """
    Apply Butterworth lowpass filter with zero-phase filtering.
    
    Args:
        x: (T, C) array of positions
        fs: Sampling frequency in Hz
        cutoff_hz: Cutoff frequency in Hz
        order: Filter order
        
    Returns:
        Filtered array with same shape as input
    """
    if cutoff_hz <= 0 or cutoff_hz >= fs / 2:
        raise ValueError(
            f"cutoff_hz must be in (0, fs/2). Got cutoff={cutoff_hz}, fs={fs}"
        )
    
    # Normalize cutoff by Nyquist frequency
    b, a = butter(order, cutoff_hz / (fs / 2), btype="low")
    return filtfilt(b, a, x, axis=0)


def normalize(v, eps=1e-12):
    """Normalize vectors with numerical stability."""
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.clip(n, eps, None)


def make_frame_lr_up(lr_vec, up_vec):
    """
    Construct orthonormal coordinate frame from left-right and up vectors.
    
    Returns rotation matrix R with columns [e_lr, e_fwd, e_up].
    """
    # Left-right axis
    e_lr = normalize(lr_vec)
    
    # Up axis
    e_up = normalize(up_vec)
    
    # Forward/depth axis (right-handed cross product)
    e_fwd = normalize(np.cross(e_up, e_lr))
    
    # Re-orthogonalize left-right
    e_lr = normalize(np.cross(e_fwd, e_up))
    
    # Stack as column vectors
    R = np.stack([e_lr, e_fwd, e_up], axis=-1)
    return R


def yaw_from_R(R):
    """Extract yaw angle from rotation matrix and convert to degrees."""
    yaw = np.arctan2(R[:, 1, 0], R[:, 0, 0])
    return np.degrees(np.unwrap(yaw))


def compute_p4_frame(joint_dict, fs=120.0):
    """
    Compute P4 frame (maximum shoulder/thorax rotation) from joint dictionary,
    searching only in the plausible backswing region (first half of the trial).

    Args:
        joint_dict: Dictionary mapping joint names to (T, 3) position arrays
        fs: Sampling frequency in Hz (unused but kept for compatibility)

    Returns:
        tuple: (p4_frame_index, max_rotation_degrees)
    """

    # Extract required joints
    ShoulderR = joint_dict["Shoulder Right"]
    ShoulderL = joint_dict["Shoulder Left"]
    SpineHigh = joint_dict["Spine High"]
    Neck = joint_dict["Neck"]

    # Build thorax frame
    R_thorax = make_frame_lr_up(
        lr_vec=(ShoulderR - ShoulderL),
        up_vec=(Neck - SpineHigh)
    )

    # Extract yaw angle
    thorax_yaw = yaw_from_R(R_thorax)

    # Restrict search to first half of the frames
    T = len(thorax_yaw)
    search_end = T // 2

    y_search = thorax_yaw[:search_end]

    # Find maximum absolute rotation within this region
    local_idx = int(np.argmax(np.abs(y_search)))
    p4_frame = local_idx
    max_rotation = thorax_yaw[p4_frame]

    return p4_frame, max_rotation

def compute_max_hand_speed_from_joints(
    joint_dict,
    fs=120.0,
    cutoff_hz=10.0,
    frames=None
):
    """
    Compute maximum hand speed using midpoint of left and right hands.
    
    Args:
        joint_dict: Dictionary mapping joint names to (T, 3) position arrays
        fs: Sampling frequency in Hz
        cutoff_hz: Lowpass filter cutoff frequency
        frames: Optional tuple (start_frame, end_frame) to limit analysis
        
    Returns:
        tuple: (max_speed, max_frame, speed_array, frame_ids, filtered_position)
    """
    # Extract hand positions
    hand_left = np.asarray(joint_dict["Hand Left"], dtype=float)
    hand_right = np.asarray(joint_dict["Hand Right"], dtype=float)
    pos_all = 0.5 * (hand_left + hand_right)
    
    # Frame selection
    T = pos_all.shape[0]
    if frames is not None:
        f0, f1 = int(frames[0]), int(frames[1])
        f0 = max(0, f0)
        f1 = min(T - 1, f1)
        pos = pos_all[f0:f1+1]
        frame_ids = np.arange(f0, f1+1)
    else:
        pos = pos_all
        frame_ids = np.arange(T)
    
    # Filter positions
    pos_filtered = butter_lowpass_filtfilt(
        pos, fs=fs, cutoff_hz=cutoff_hz, order=4
    )
    
    # Compute velocity and speed using gradient
    dt = 1.0 / fs
    vel = np.gradient(pos_filtered, dt, axis=0)
    speed = np.linalg.norm(vel, axis=1)
    
    # Find maximum
    i_max = np.argmax(speed)
    max_speed = float(speed[i_max])
    max_frame = int(frame_ids[i_max])
    
    return max_speed, max_frame, speed, frame_ids, pos_filtered


def main():
    """Main entry point for CLI usage."""

    parser = argparse.ArgumentParser(
        description="Analyze golf swing P4 position and hand speed"
    )

    parser.add_argument(
        "json_file",
        help="Path to JSON file containing swing data"
    )

    args = parser.parse_args()

    try:
        # Load data
        data = load_json_clean(args.json_file)
        pose_data = data["data"]["pose"]
        frame_rate = data["configuration"]["frame_rate"]

        # Create joint dictionary
        joint_dict = create_joint_dict(pose_data, JOINT_LABELS)

        # ===== Compute P4 =====
        p4_frame, max_rotation = compute_p4_frame(joint_dict, fs=frame_rate)
        print(f"P4 Frame: {p4_frame}")

        # ===== Compute Hand Speed =====
        max_speed, max_frame, speed_trace, frame_ids, pos_filtered = \
            compute_max_hand_speed_from_joints(
                joint_dict=joint_dict,
                fs=frame_rate,
                cutoff_hz=10,
                frames=None
            )

        # Convert to mph
        max_speed_mph = max_speed * 2.23694

        print(f"Maximum Hand Speed: {max_speed:.2f} m/s ({max_speed_mph:.2f} mph)")

    except FileNotFoundError:
        print(f"Error: File not found: {args.json_file}")
        sys.exit(1)

    except KeyError as e:
        print(f"Error: Missing expected data in JSON: {e}")
        sys.exit(1)

    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
