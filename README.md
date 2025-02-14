# self-driving-arduino-car

A very basic self driving arduino car

# Features
- path planning
- obastacle avoidance
- line holding
- motion plaing
- manual control

# Build
colcon build --packages-select main-pipeline
colcon build --packages-select line-detection
colcon build --packages-select manual-control
colcon build --packages-select obstacle-avoidance
colcon build --packages-select path-planning
colcon build --packages-select motion-calibration

# Run
ros2 run main-pipeline main_pipeline
ros2 run line-detection line_detection
ros2 run manual-control manual_control
ros2 run obstacle-avoidance obstacle_avoidance
ros2 run path-planning path_planning
ros2 run motion-calibration motion_calibration
