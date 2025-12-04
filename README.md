#  nav_way  
### ROS2 Humble • TurtleBot3 • Trajectory Tracking • Path Smoothing • Pure Pursuit • Obstacle Avoidance

This repository contains a complete **ROS2 Humble workspace (`ros2_ws`)** implementing:

- ✔ Path smoothing (Cubic Spline Interpolation)  
- ✔ Time-parameterized trajectory generation  
- ✔ Pure Pursuit tracking controller  
- ✔ Obstacle avoidance using LiDAR (Extra Credit)  
- ✔ RViz visualization of:  
  - Original waypoints  
  - Smoothed trajectory  
  - Target point  
- ✔ Simulation of TurtleBot3 Waffle in the **Empty World**

This fulfills all assignment requirements, including extra credit.

---

#  Assignment Requirements Covered

| Requirement | Status |
|------------|--------|
| Path smoothing | ✔ Implemented (`path_smoothing.py`) |
| Trajectory generation | ✔ Implemented (`trajectory_generator.py`) |
| Trajectory tracking controller | ✔ Implemented (Pure Pursuit) |
| Simulation using differential drive robot | ✔ TurtleBot3 Waffle |
| Documentation | ✔ In this README |
| Extra Credit: Obstacle avoidance | ✔ Implemented using repulsive vector method |
| Demonstration video | ✔ See video below |

---

#  Workspace Structure

ros2_ws/
│── src/
│ └── navigation_assignment_py/
│ ├── package.xml
│ ├── setup.py
│ ├── navigation_assignment_py/
│ │ ├── controller_node.py
│ │ ├── path_smoothing.py
│ │ ├── trajectory_generator.py
│ ├── resource/
│ ├── config/
│── README.md
│── .gitignore
│── ros2_ws.zip (clean zipped workspace) 

Launch Simulation (Empty World)

The empty world is used to clearly visualize trajectories and markers:

export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py

 Start the Pure Pursuit Controller
source install/setup.bash
ros2 run navigation_assignment_py controller_node


This single node performs:

Path smoothing

Trajectory creation

Pure pursuit control

Obstacle avoidance

RViz visualization publishing

RViz Setup

Start RViz:

rviz2


Then set:

Fixed Frame → odom

Add the following displays:

Display	Topic	Description
Marker	/waypoints_marker	Original waypoints (red spheres)
Marker	/trajectory_marker	Smoothed trajectory (blue line)
Marker	/target_point	Pure-Pursuit target (orange sphere)
Path	/planned_path	Full smoothed path
Odometry	/odom	Robot position
LaserScan	/scan	For obstacle avoidance visualization
 Algorithms Explained
1.** Path Smoothing (Cubic Spline)**

Discrete waypoints are interpolated using cubic splines to generate a smooth, continuous curve.

Benefits:

Removes sharp corners

Better for tracking

More realistic robot motion

2.** Time-Parameterized Trajectory**

Each smoothed point is given a timestamp based on robot speed:

t(i+1) = t(i) + distance / velocity


Outputs:

(x, y, t, v)

3. **Pure Pursuit Controller**

A geometric tracking controller.

Steps:

Find a point on the path at lookahead distance L

Compute steering angle:

heading_error = atan2(target - robot)
angular_z = k * heading_error


Linear speed decreases during sharp turns

Advantages:

Smooth

Simple

Works extremely well on differential-drive robots

 4. **Obstacle Avoidance (Extra Credit)**

LiDAR (/scan) is used to generate a repulsive vector:

Final heading = goal_heading + repulsive_heading


Robot slows down and turns away from close obstacles.

