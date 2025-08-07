# ROS2 Modules

## Basic Information
ROS2 modules infrastructure to achive self-driving car functionality:
- each module implements it's own step
- all modules gather into exectution pipeline
- **main module is main-pipeline**
- manual-control module used to turn on/off auto-pilot mode and remotly control
  - has the highest command priority
  - to minimize input lag between remote command and actual vehicle event

## Motion Vector
[Structure represents vehicle motion](https://github.com/vlmarkov/self-driving-car/blob/main/ros2-modules/interfaces/msg/MotionVector.msg):
- acceleration
- steer angle
