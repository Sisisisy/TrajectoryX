# F1TENTH controller — submission 2

Package: `team_controller`
Run command: `ros2 run team_controller controller`
Target: CIE-KFUPM competition V1.1, ROS 2 Jazzy, Ubuntu 24.04 x86_64.

## Validation status

Submission 2 fixes the problems seen with submission 1 (the car crawled in
corners and parked in the dead-end pocket south of the junction).

Tested outside ROS in a small Python re-creation of the Levine map (same map file,
819-beam LiDAR, kinematic car): three laps with zero collisions at 0.8 m/s and at
0.4 m/s, several noise seeds, and recovery from a start inside the dead-end pocket
and from a start facing a wall. NOT yet tested in the real ROS 2 simulator: build
it and run the three-lap check in RViz before relying on it. Not validated for a
real vehicle.

## Algorithm

1. Route following (Stages 1 and 2): when `/ego_racecar/odom` is fresh and the car
   is within 2 m of the official Levine centerline (embedded in `levine_route.py`,
   released for those stages), it pure-pursues a point 1.4 m ahead on that route.
   LiDAR still has priority: if the way to that point is blocked it steers to the
   nearest open gap, so obstacles (Stage 2) are avoided.
2. Reactive fallback (any other track, e.g. Stage 3, or stale odometry): LiDAR-only
   gap following with disparity extension. No map or waypoint is used.
3. Recovery: if the nose is pinned against a wall for 0.4 s, it reverses (steering
   the nose toward the more open side) until the front is clear, then drives on.
4. Safety: stops on missing/garbage scans or stale scans; never reverses blindly.

The route is only used near the Levine track; on any other map the car is more
than 2 m from it and the controller is purely reactive.

## Dependencies

ROS: ament_python (build), rclpy, sensor_msgs, nav_msgs, ackermann_msgs.
Python: standard library and setuptools only. Competition simulator underlay
must already be installed using the official V1.1 installation guide.
No additional pip installation is required.

## Fast verification on another laptop

Use the outer `START_HERE.txt` first. The included scripts build in an isolated
`controller_ws` folder beside this package and avoid copying into an existing team
workspace. Example, from the extracted submission folder:

```bash
bash build.sh /path/to/competition-repo
# In another terminal: start the official simulator, stop other controllers.
bash verify.sh /path/to/competition-repo
bash run.sh /path/to/competition-repo 0.4
```

`verify.sh` keeps speed at zero and checks ROS communication; it does not verify
collision-free driving. For that, watch RViz for three timed laps and zero collisions.

## Manual build alternative

Replace `/absolute/path/to/competition-repo` below with the actual official
competition checkout path. Extract this ZIP first. Open a terminal in the
extracted `f1tenth_controller_submission2` folder (the folder containing
`team_controller` and `START_HERE.txt`):

```bash
source /opt/ros/jazzy/setup.bash
export COMPETITION_REPO=/absolute/path/to/competition-repo
source "$COMPETITION_REPO/starter_ws/.venv/bin/activate"
source "$COMPETITION_REPO/starter_ws/install/setup.bash"
mkdir -p ~/controller_ws/src
cp -r team_controller ~/controller_ws/src/
cd ~/controller_ws
python /usr/bin/colcon build --symlink-install --packages-select team_controller
source install/setup.bash
ros2 pkg executables team_controller
```

Expected executable: `team_controller controller`.

## Run and check

Terminal 1, from the official competition repository:

```bash
./scripts/launch_sim.sh
```

Terminal 2 (set COMPETITION_REPO to your checkout path again):

```bash
source /opt/ros/jazzy/setup.bash
export COMPETITION_REPO=/absolute/path/to/competition-repo
source "$COMPETITION_REPO/starter_ws/.venv/bin/activate"
source "$COMPETITION_REPO/starter_ws/install/setup.bash"
source ~/controller_ws/install/setup.bash
ros2 run team_controller controller --ros-args -p max_speed:=0.4
```

Stop PS5/manual teleoperation before starting. There must be only one publisher
on `/drive`. In another sourced terminal, check `ros2 topic info /drive` and the
RViz Race Dashboard. First finish-line crossing starts timing (out-lap); then
complete three timed laps and confirm zero collisions. For the baseline default,
use `ros2 run team_controller controller` (maximum requested speed 0.8 m/s).
Use an organizer-announced cap if lower; do not assume 0.8 m/s is the official cap.
Stop with Ctrl+C, then run the official `./scripts/stop_car.sh` if needed.

## Parameters

`max_speed`: maximum requested speed in m/s; default 0.8, hard upper bound 0.8.
Set at launch with `--ros-args -p max_speed:=0.4` (or a lower announced cap).
Parameter values are read once at startup. Do not change code/settings during
an official run. Other algorithm constants are in `algorithm.py`: forward field
±80 degrees, front stop distance 0.45 m, clearance threshold 1.0 m,
obstacle margin 0.30 m, steering bound ±0.40 rad. Scan timeout: 0.30 s.
Commands publish at 20 Hz while ROS is processing normally. Stale-scan stopping
assumes the node remains running; it is not an independent emergency stop.

## Submission

This ZIP includes controller source and README, with no simulator, virtual
environment, or generated build/install/log directories. Rename the outer ZIP
with your actual team name/version if requested. Keep Python package names
unchanged unless you update all matching files. The MIT license permits use and
modification. Submit via the channel the organizers specified; the GitHub docs
describe USB ZIP or repository URL + exact full commit hash, not a submission portal.

Official instructions:
https://github.com/KayiiX/CIE-KFUPM-F1tenth-Autonomous-Racing-Competition/blob/v1.1/docs/CONTROLLER_SUBMISSION_GUIDE.md
