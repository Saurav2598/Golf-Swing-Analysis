# Golf-Swing-Analysis

## Instructions to run the code

1. Create a virtual environment eg: using python -m venv .venv
2. Activate venv and run pip install -r requirements.txt
3. From CLI run python swing_analysis.py <path_to_json_file>  to get the metrics ( P4 frame and max_hand_speed)
4. From CLI run python plot_swings.py <path_to_json_file>  to get swing plots

## Shoulder Rotation and P4 frame identification

Shoulder rotation is calculated as the thorax segment (shoulder line) yaw angle in the global coordinate system.

A local coordinate system is constructed for the segment using the markers: Shoulder Right(15), Shoulder Left(19), Spine High(14) and Neck(23). We need a primary axis and secondary axis for this purpose 
<br>
Primary Axis: 
lr_vec = ShoulderR - ShoulderL defines the left-right axis of the thorax

Secodary Axis:
up_vec = Neck - SpineHigh defines the upward axis of the thorax

The third axis is identified by taking the cross product of the two axes and then the primary axes is re-orthogonalized.

Normalized to unit vectors and a rotation matrix is built.

$$
\mathbf{e}_{lr} = \frac{\mathbf{lr\_vec}}{\|\mathbf{lr\_vec}\|}
$$

The unit vectors are then stacked to build the rotation matrix: 

$$
R =
\begin{bmatrix}
e_{lr,x} & e_{fwd,x} & e_{up,x} \\
e_{lr,y} & e_{fwd,y} & e_{up,y} \\
e_{lr,z} & e_{fwd,z} & e_{up,z}
\end{bmatrix}
$$

Yaw is calculated as:

$$
\psi = \text{arctan2}(e_{lr,y}, e_{lr,x})
$$

The resulting angle is then unwrapped to remove discontinuities and converted to degrees:

$$
\psi_{deg} = \text{unwrap}(\psi)\frac{180}{\pi}
$$

P4 frame is identified as the instance where shoulder rotation is maximum. This is assumed to fall in the first half of the frames so as to avoid false identification of shoulder rotation maxima in the follow through.

### Hand Speed Calculation

Hand speed is calculated from the midpoint of the left and right hand markers instead of using leading or trailing hands as it is a more robust approximation of hand speed.

In each frame, the 3D hand position was defined as the average of the two wrist marker coordinates. The time derivative of this position was computed to obtain hand velocity, and the magnitude of the velocity vector was taken as hand speed. The maximum value of this hand-speed profile during the swing is reported as the maximum hand speed.

Markers Used : "Hand Left"(22), "Hand Right"(18) 

4th order Butterworth Lowpass filter with a cutoff of 10Hz ( which is the  human movement kinematics usually occur). 

In the present dataset, filtering had minimal influence on the computed hand-speed values because the joint coordinates are already relatively smooth. Nevertheless filtering was applied to stick to standard biomechanical signal processing practices.

Marker trajectories were low-pass filtered prior to differentiation.

### Impact Frame Calculation 

Impact frame is calculated from ball coordinates. 

Ball speed is calculated from frame to frame displacements and the impact frame is taken as the first instance where the ball speed is above a threshold of 1m/s consistently for at least 2 consecutive frames. This is done to avoid false detections caused by marker noise or tracking jitter.

## Assumptions

1. Swing start (P0):
The recording is assumed to start at address (P0) or very close to it and all segment rotations (yaw angles) are therefore computed relative to the first frame.

2. Backswing region for P4 detection:
The top of the backswing (P4) is assumed to occur within the first half of the recorded frames, and P4 is identified as the frame with maximum absolute shoulder (thorax) yaw within this plausible backswing region.

3. Butterworth filtering for body markers:
A 4th-order Butterworth low-pass filter with a 10 Hz cutoff is assumed to adequately capture the relevant human movement kinematics of the golf swing while removing high-frequency measurement noise from the marker trajectories.

4. Impact detection threshold:
The impact frame is detected when ball speed exceeds 1 m/s for at least two consecutive frames as this threshold is assumed to be sufficiently above marker noise and small tracking jitter to reliably indicate true ball motion after club contact.

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

At the top of the backswing (P4), Swing A shows greater overall segment rotation than Swing B, with approximately 10° more thorax rotation, 12° more pelvis rotation, and 7° more knee rotation. Both swings exhibit a nearly identical X-factor (~57–58°).

X-factor is the seperation between thorax and pelvis rotation at the top of the backswing(P4).

The rotational profiles for shoulder, pelvis, knee and X-factor shows a very similar pattern in both swings suggesting that they are performed by the same player.

However, there are some differences in the kinematic sequence of angular velocity peaks. In Swing B, the thorax angular velocity peak occurs about 8 frames later relative to the pelvis peak, whereas in Swing A the thorax peak occurs almost immediately (about 1 frame later).

The peak angular velocities of the pelvis, thorax, and club head are much higher in Swing B compared to Swing A. 

Final Conclusion : Very similar rotational profiles and nearly identical X factor suggest that both swings are performed by the same athlete. The differences in kinematic sequences and the peak angular velocities which may be because of the skill level of the athlete or completely different swing types  ( Lower effort swing vs Full Swing ).


## References
1. Zhou JY, Richards A, Schadl K, Ladd A and Rose J (2022) The swing performance Index: Developing a single-score index of golf swing rotational biomechanics quantified with 3D kinematics. Front. Sports Act. Living 4:986281. doi: 10.3389/fspor.2022.986281
2. B. F. Taylor, Biomechanical Golf Swing Analysis using Markerless Motion Capture, Master’s thesis, Massachusetts Institute of Technology, Cambridge, MA, USA, 2025.
<br>
