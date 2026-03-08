# Golf-Swing-Analysis
# Golf-Swing-Analysis



Instructions to run the code 



1. Create a virtual environment eg: using python -m venv .venv
2. Activate venv and run pip install -r requirements.txt
3. From CLI run "python swing\_analysis.py data/1772542049.json" to get the metrics ( P4 frame and max\_hand\_speed)
4. From CLI run "python plot\_swing.py data/1772542049.json" to get swing plots



Approach



Shoulder Rotation and P4 frame identification



Shoulder rotation is calculated as the thorax segment (shoulder line) yaw angle in the global co-ordinate system. Construct a local co-ordinate system for the segment using the markers Shoulder Right, Shoulder Left, Spine High and Neck.

&nbsp;

Here:

lr\_vec = ShoulderR - ShoulderL defines the left-right axis of the thorax



up\_vec = Neck - SpineHigh defines the upward axis of the thorax



They are then orthogonolied and re-orthogonalized to perfectly orthogonalized axes which are then stacked to get the rotation matrix from which yaw is calculate using



&nbsp;					 yaw = np.arctan2(R\[:, 1, 0], R\[:, 0, 0])



P4 frame is identified as the point where of maximum absolute yaw in the plausible region of downswing which is assumed to be first half of the frames. 



Hand Speed Calculation



The midpoint of both the hands were used an better approximation for hand speed because the leading and trailing hand move differently and how the athlete grips the handle gives a better estimate of the hand system motion.



Markers Used : "Hand Left", "Hand Right" to calculate midpoint.



Butterworth Lowpass filter 4th order with a cutoff of 10Hz which is the range below human movement kinematics usually occur. 



Marker trajectories were low-pass filtered prior to differentiation to reduce high-frequency measurement noise.



Results: 

"1772542049.json"

P4 Frame: 111

Maximum Hand Speed: 5.95 m/s (13.31 mph)



"1768733281.json"

P4 Frame: 135

Maximum Hand Speed: 8.78 m/s (19.63 mph)



Comparison of 2 swings



Assuming the swings are not from a complete beginner but a player who has been coached and i analysed the sequence of peaks of angular velocities of segments as shown in the figures below 









