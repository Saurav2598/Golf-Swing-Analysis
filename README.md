# Golf-Swing-Analysis

## Instructions to run the code

1. Create a virtual environment eg: using python -m venv .venv
2. Activate venv and run pip install -r requirements.txt
3. From CLI run "python swing_analysis.py data/1772542049.json" to get the metrics ( P4 frame and max_hand_speed)
4. From CLI run "python plot_swing.py data/1772542049.json" to get swing plots

## Shoulder Rotation and P4 frame identification

Shoulder rotation is calculated as the thorax segment (shoulder line) yaw angle in the global co-ordinate system. Construct a local co-ordinate system for the segment using the markers: Shoulder Right, Shoulder Left, Spine High and Neck.
 
Here:
lr_vec = ShoulderR - ShoulderL defines the left-right axis of the thorax

up_vec = Neck - SpineHigh defines the upward axis of the thorax

They are then orthogonolied and re-orthogonalized to perfectly orthogonalized axes which are then stacked to get the rotation matrix from which yaw is calculate using

 					 yaw = np.arctan2(R[:, 1, 0], R[:, 0, 0])

P4 frame is identified as the point where of maximum absolute yaw in the plausible region of downswing which is assumed to be first half of the frames.

### Hand Speed Calculation

The midpoint of both the hands were used an better approximation for hand speed because the leading and trailing hand move differently and how the athlete grips the handle gives a better estimate of the hand system motion.

Markers Used : "Hand Left", "Hand Right" to calculate midpoint.

Butterworth Lowpass filter 4th order with a cutoff of 10Hz which is the range below human movement kinematics usually occur.

Marker trajectories were low-pass filtered prior to differentiation to reduce high-frequency measurement noise.

### Impact Frame Calculation 

Impact frame calculated from ball co-ordinates. 

Ball speed calculated from frame to frame displacements and the impact frame is taken as the first instance where the ball_speed is above a threshold(1m/s) consistently for at least 2 consecutive. This is done to avoid false detections caused by marker noise or tracking jitter.

## Assumptions

1. Swing start (P0):
The recording is assumed to start at address (P0) or very close to it. All segment rotations (yaw angles) are therefore computed relative to the first frame of the recording.

2. Backswing region for P4 detection:
The top of the backswing (P4) is assumed to occur within the first half of the recorded frames, and P4 is identified as the frame with maximum absolute shoulder (thorax) yaw within this plausible backswing region.

3. Yaw as the primary rotation metric:
Segment rotations are analyzed using yaw (rotation about the vertical axis), assuming that the dominant rotational motion of the golf swing occurs in the transverse plane.

4. Butterworth filtering for body markers:
A 4th-order Butterworth low-pass filter with a 10 Hz cutoff is assumed to adequately capture the relevant human movement kinematics of the golf swing while removing high-frequency measurement noise from the marker trajectories.

5. Savitzky–Golay filtering for ball motion:
A Savitzky–Golay filter is used for the ball trajectory because it preserves sharp motion changes and local velocity peaks, which is important for accurately detecting the sudden acceleration of the ball immediately after impact.

6. Impact detection threshold:
The impact frame is detected when ball speed exceeds 1 m/s for at least two consecutive frames, assuming this threshold is sufficiently above marker noise and small tracking jitter to reliably indicate true ball motion after club contact.

## Results
"1772542049.json"
&nbsp;
P4 Frame: 111
&nbsp;
Maximum Hand Speed: 5.95 m/s (13.31 mph)

"1768733281.json"
&nbsp;
P4 Frame: 135
&nbsp;
Maximum Hand Speed: 8.78 m/s (19.63 mph)

## Comparison of 2 swings


For Swing A 

<img width="1200" height="800" alt="49_PLOTS" src="https://github.com/user-attachments/assets/5ffabc1b-3e72-4180-ac4e-b1d1099445cb" />




For Swing B 



<img width="1200" height="800" alt="81_PLOTS" src="https://github.com/user-attachments/assets/97559ca8-a963-4d86-a3dc-97c759363d9b" />

At the top of the backswing (P4), Swing A shows greater overall body rotation than Swing B, with approximately 10° more thorax rotation, 12° more pelvis rotation, and 7° more knee rotation. Despite these differences in absolute segment rotations, both swings exhibit a nearly identical X-factor (~57–58°), indicating similar torso–pelvis separation at the top of the backswing.

The rotational profiles for shoulder, pelvis, knee and X-factor shows a very similar pattern in both swings suggesting that they are performed by the same player

However, there are some differences in the kinematic sequence of angular velocity peaks. In Swing B, the thorax angular velocity peak occurs about 8 frames later relative to the pelvis peak, whereas in Swing A the thorax peak occurs almost immediately (about 1 frame later).

The peak angular velocities of the pelvis, thorax, and club head are noticeably higher in Swing B compared to Swing A, indicating a faster overall rotational motion during the downswing in Swing B.

The differences in kinematic sequencing and the irregular timing of some peak velocities indicate less consistent coordination of segmental motion, which is more characteristic of an amateur golfer than a highly consistent professional.

Final Conclusion : Strong similarities in their rotational profiles and nearly identical X factor that they are performed by the same athlete, probably an amateur golfer with the first swing possibly a warm-up swing.

