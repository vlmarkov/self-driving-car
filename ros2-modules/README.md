# ROS2 Modules

ROS2 modules combines into a pipeline:
- `lane detection` gets `motion vector`
- `main-pipeline` sends `motion vector` to the `chassis-control`
- `chassis-control` converts `motion vector` to control command for `motors`
- `chassis-control` also implements a simple motion-planner
- addionaly `remote-control` allows to turn on/off auto-pilot mode and control the robot car

## Motion Vector
[Structure represents vehicle motion](https://github.com/vlmarkov/self-driving-car/blob/main/ros2-modules/interfaces/msg/MotionVector.msg): `acceleration` and `steer angle`
