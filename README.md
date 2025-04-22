# Self-Driving Arduino Car

A very basic self driving arduino car

# Features
- path planning
- obstacle avoidance
- lane detection/holding
- motion planing
- manual control

# Requirements
- ubuntu 24.04
- latest cmake version
- latest gcc or clang version
- arduino ide
- gtest
- [wiringpi](https://github.com/WiringPi/WiringPi/tree/master)
  - do not forget to add user to gpio group
  - or `sudo chmod g+rw /dev/gpiomem0`
- opencv
  - `sudo ln -s /usr/include/opencv4/opencv2 /usr/include/opencv2`
- [ros2](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [LLCV](https://github.com/kbarni/LCCV/tree/main) LCCV (libcamera bindings for OpenCV) is a small wrapper library that provides access to the Raspberry Pi camera in OpenCV.


# Prepare your ROS2 environment
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

# Test
```
make tests
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
