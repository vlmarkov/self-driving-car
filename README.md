# self-driving-arduino-car

A very basic self driving arduino car

# Features
- path planning
- obstacle avoidance
- line holding
- motion plaing
- manual control

# Build
1. Prepare your ros2 environment
```
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
```

2. Build
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
ros2 run line-detection line_detection
ros2 run motion-calibration motion_calibration
```
