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
ros2 launch robot_controller robot_controller.launch.xml 
```

## 4. **Run the controller node.**

```
ros2 run robot_controller rpm_vel_controller
```

## 5. Run Telep_Twist_Keyboard

```python
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 6. Conclusion

The rpm values are generated through node which are taken as an input by the custom plugin to publish joint states on both of the wheels.