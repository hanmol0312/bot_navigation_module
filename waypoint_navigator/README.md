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
ros2 launch waypoint_navigator robot.launch.xml 
```

## 4. **Run the navigation node.**

```
ros2 run waypoint_navigator waypoint_navigation --ros-args -p waypoint_1_x:=2.0 -p waypoint_1_y:=1.0 -p waypoint_2_x:=4.0 -p waypoint_2_y:=3.0
```

## 5. Conclusion

The robot navigates to the locations mentioned using odometry and publishing velocities with pid controller.