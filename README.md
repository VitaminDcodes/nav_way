#  nav_way  
### ROS2 Humble • TurtleBot3 • Trajectory Tracking • Path Smoothing • Pure Pursuit • Obstacle Avoidance
(https://github.com/VitaminDcodes/nav_way/blob/main/video_nav.mp4)

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
  Algorithms & Design Choices
🔹1 Path Smoothing (Cubic Spline Interpolation)

Robots cannot follow sharp waypoint corners.
Thus, we use cubic spline interpolation, producing a curve that is:

Continuous

Differentiable

Smooth in curvature

This improves both tracking accuracy and vehicle stability.

Key advantages:

Removes discontinuities

Generates realistic motion

Works well with Pure Pursuit

🔹2 Trajectory Generation (Time Parameterization)

The smoothed path is converted into a trajectory with timestamps.

For each pair of points:

distance = √((x₂−x₁)² + (y₂−y₁)²)
dt = distance / desired_velocity
t += dt


This yields:

(x, y, t, velocity)


Why important?

The robot knows where it should be at any time

Enables velocity profiling

Helps evaluate tracking performance

🔹3 Pure Pursuit Controller (Main Tracking Algorithm)

Pure Pursuit is a geometric tracking controller widely used in autonomous robots.

How it works:

Select a lookahead point on the trajectory

Compute directional error:

heading_error = atan2(target_y - y, target_x - x) - robot_yaw


Compute angular velocity:

angular_z = k * heading_error


Adjust linear speed during turns for stability.

Why we choose Pure Pursuit:

Simple yet powerful

Provides smooth tracking

Robust to minor noise

Works well for differential-drive robots

This method allows the robot to behave like it is "pulling itself" toward a point on the path.

⭐4 Obstacle Avoidance (Extra Credit Requirement)

LaserScan data is used to compute a repulsive vector field:

Obstacles closer than a threshold exert repulsive influence

Repulsive direction is combined with the pure pursuit direction:

final_heading = goal_heading + repulsive_heading


If any obstacle is extremely close, robot’s velocity → 0 (safety stop)

This creates a lightweight but effective local avoidance layer.


Robot slows down and turns away from close obstacles.

