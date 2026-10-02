# F1TENTH Gym ROS 2 Labs

This ROS 2 workspace contains my implementations of three F1TENTH labs.  Each
algorithm is developed here in `f110_labs_ws` and tested with the
[F1TENTH Gym ROS simulator](https://github.com/f1tenth/f1tenth_gym_ros) in
`sim_ws`. For simulator installation and launch instructions, see the
[F1TENTH Gym ROS repository](https://github.com/f1tenth/f1tenth_gym_ros).

## Lab 1 — Automatic Emergency Braking (AEB)

Implementation: [`AEBnode.py`](src/lab1_pkg/scripts/AEBnode.py)

The AEB node subscribes to LiDAR scans and the race car odometry. For every
LiDAR beam, it estimates the instantaneous time-to-collision (iTTC) from the
measured range, vehicle speed, and beam angle. If any beam reports an iTTC
below the 1.3-second safety threshold, the node publishes a zero-speed
Ackermann drive command to `/drive`, bringing the car to a stop before a
collision.

This is the only lab that uses `teleop_keyboard`: use it to drive the car
toward an obstacle while the AEB node monitors the scan and overrides the
motion when braking is required.

Video: [The AEB demonstration video](https://drive.google.com/file/d/1oF3pqGn_ApMWYdBttso9lYNAU1ZkJnZe/view?usp=sharing)

## Lab 2 — Wall Following with a PID Controller

Implementation: [`ErrorCalculator.py`](src/lab1_pkg/scripts/ErrorCalculator.py)

The wall-following node estimates the distance and orientation of the wall
from two LiDAR rays. It projects that measurement forward to compute the
expected wall distance, compares it with the desired 1.2 m clearance, and
uses proportional and derivative terms to generate the steering command. The
node also reduces speed as the steering demand increases, helping the vehicle
follow the wall more reliably through turns.

Video: [Wall-following demonstration video](https://drive.google.com/file/d/1euR5QiHqPVDwnNdUPbMCdBbRwL5HIoXk/view?usp=sharing)

## Lab 3 — Follow the Gap

Implementation: [`FollowGap.py`](src/lab1_pkg/scripts/FollowGap.py)

This lab uses a reactive Follow-the-Gap planner. The node preprocesses the
LiDAR scan, removes a safety bubble around the closest obstacle, finds the
best open gap, and steers toward that gap's midpoint. It commands a constant
forward speed while selecting a collision-free direction from the latest
scan. `FollowGap.py` is the implementation used for this lab

Video: [Follow-the-Gap demonstration video](https://drive.google.com/file/d/10f5Hq7ixPyKmH0xfSh2Pjpp1TgBdKQiz/view?usp=sharing)
