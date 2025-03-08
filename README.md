# self-driving-arduino-car

A very basic self driving arduino car

# Features
- path planning
- obstacle avoidance
- lane detection/holding
- motion plaing
- manual control

# Prepare your ros2 environment
1. According to [official documentation](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files) for every new shell you will need to run this command to have access to the ROS 2 commands:
```
source /opt/ros/jazzy/setup.bash
```
2. Alternative way is to [add sourcing to your shell startup script](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files)
```
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```
3. But for build rule and especially for install directory you have mannualy set the environment:
```
source install/local_setup.bash
```

# Build
```
make all
```

# Run
1. Choose module to run
```
ros2 run main-pipeline main_pipeline
ros2 run manual-control manual_control
ros2 run path-planning path_planning
ros2 run obstacle-avoidance obstacle_avoidance
ros2 run lane-detection lane-detection
ros2 run motion-calibration motion_calibration
```
