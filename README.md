## 1. Clone the repository.

```
mkdir my_ws
cd my_ws
mkdir src
cd src
git clone https://github.com/hanmol0312/leader_follower_project.git
```

## 2. Build the project.

```
cd ..
colcon build
```

## 3. Launch the simulation

```
ros2 launch leader_follower_navigation gazebo.launch.py
```

## 4. Run the Follow_Node

```
ros2 run leader_follower_navigation follower_node
```

## 5. Run the teleop_twist_keyboard to control the leader_bot

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/leader_bot/cmd_vel
```

## 6. Conclusion

The follower bot will continuously maintain a distance of 1m and align along the leader robot.