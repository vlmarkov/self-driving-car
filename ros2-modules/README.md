# ROS2 Modules

## Basic Information
ROS2 modules infrastructure to achive self-driving car functionality:
- each module implements it's own step
- all modules gather into exectution pipeline
- **main module is main-pipeline**
- manual-control module used to turn on/of auto/pilot mode, and has the highest command priority to minimize input lag between remote command and stop vehicle event

## Exectuion Pipeline
```
                                          (disable pipeline)
                       Main-Pipeline   <---------------------- Manual-Control
                   +-----+-----+-----+                         |
Path-Planning      | MV  |     |     |                         |
                   +-----+-----+-----+                         | (directly send to Motion-Calibration)
Line-Detecion      |     | MV  |     |                         |
                   +-----+-----+-----+                         |
Obstacle-Avoidance |     |     | MV  | -> Motion-Calibration <-+
                   +-----+-----+-----+           ||
                                                 \/
                                               +-----+
                                               | MV  |
                                               +-----+
                                                 ||
                                                 \/
                                             Arduino-Board
```
