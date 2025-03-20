# ROS2 Modules

## Basic Information
ROS2 modules infrastructure to achive self-driving car functionality:
- each module implements it's own step
- all modules gather into exectution pipeline
- **main module is main-pipeline**
- manual-control module used to turn on/off auto-pilot mode and remotly control
  - has the highest command priority
  - to minimize input lag between remote command and actual vehicle event

## Execution Pipeline
```
                                          (auto-pilot on/of)
                       Main-Pipeline   <---------------------- Manual-Control
                   +-----+-----+-----+                         |
Path-Planning      | MV* |     |     |                         |
                   +-----+-----+-----+                         | (MV* directly send to Motion-Calibration)
Line-Detecion      |     | MV* |     |                         |
                   +-----+-----+-----+                         |
Obstacle-Avoidance |     |     | MV* | -> Motion-Calibration <-+
                   +-----+-----+-----+           ||
                                                 \/
                                           +------------+
                                           | Pin Signal |
                                           +------------+
                                                 ||
                                                 \/
                                            Arduino-Board

*MV - Motion Vector
```

## Motion Vector
[Structure represents vehicle motion](https://github.com/vlmarkov/self-driving-arduino-car/blob/main/ros2-modules/interfaces/msg/MotionVector.msg):
- acceleration
- steer angle
