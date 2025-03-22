# Motion Calibration

## Basic Information
This module responsible to transfer C++ structure fields values to the pin digital value for Arduino board.

## Feature
- store history of the previos MotionVector structre values
- filter motion peaks (to make a very smooth motion)
- transfer C++ structure fields values to pin digital value

## Connection Scheme
Connection between RaspberryPi and Arduino is made through simple pin connection.

```
RaspberryPi Pins        Arduino Pins
     [21]   -----------> [ 0]
     [22]   -----------> [ 1]
     [23]   -----------> [ 2]
     [24]   -----------> [ 3]
                         [ 4]
                         ....
```

## Pins Descriptions
- Pin 21 forward command
- Pin 22 backward command
- Pin 23 left turn command
- Pin 24 right turn command

## How To Run
```
ros2 run motion-calibration motion_calibration
```
