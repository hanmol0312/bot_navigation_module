## 1. Clone the repository.

```
mkdir my_ws
cd my_ws
mkdir src
cd src
git clone https://github.com/hanmol0312/bot_navigation_module.git

```

## 2. Build the project.

```
cd ..
colcon build
```

## 3. Launch the simulation

```
ros2 launch nav2_bot_world robot.launch.xml 
```

## 4. **Run the waypoint navigation node.**

```
ros2 run nav2_bot wp_navigation 
```

## 5. **Optimization in the navigation paramaters**

- Addition of rotation shim controller for proper on axis rotation without relying on the dwb_planner. Proper on axis rotation with reduced velocity.

```python

    FollowPath:      plugin: "nav2_rotation_shim_controller::RotationShimController"  # Added rotation shim controller for better handling of on axis rotation      primary_controller: "dwb_core::DWBLocalPlanner"      angular_dist_threshold: 0.785      forward_sampling_distance: 0.5      rotate_to_heading_angular_vel: 1.0      max_angular_accel: 1.0      simulate_ahead_time: 1.0      rotate_to_goal_heading: false      debug_trajectory_details: True

```

- Reduced max_vel_theta: 1.0
- Local_Costmap adjusted robot radius according to the wheelbase to 0.25
- Inflation radius reduced to 0.4 for moving in compact spaces as well.
- Adjusted velocity range in velocity smoother. For smooth motion of the robot.

```python
    max_velocity: [0.26, 0.0, 0.4]  # reduced max velocity threshold    min_velocity: [-0.26, 0.0, -0.4]  # reduced max velocity threshold
   
```

## 6. Adding navigation waypoints if required.

- Add extra waypoints in yaml file in waypoints.yaml in nav2_bot_world/maps:

```python
waypoints:  wp1: {x: 4.76, y: 0.0, yaw: 0.0}  wp2: {x: 6.73, y: -2.15, yaw: 0.0}  wp3: {x: 2.45, y: -4.26, yaw: 0.0}  wp4: {x: 9.51, y: -0.80, yaw: 0.0}

```

## 7. Challenges

- The lidar data is distorting inspite of reduced velocities causing the localization to have an effective error.

## 8. Conclusion

- The robot is able to navigate to the waypoints properly with proper feedback.