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
     [31]   -----------> [D0]
     [33]   -----------> [D1]
     [35]   -----------> [D2]
     [37]   -----------> [D3]
                         ....
```

## Pins Descriptions
- Pin 31 forward command
- Pin 33 backward command
- Pin 35 left turn command
- Pin 37 right turn command

## How To Run
```
ros2 run motion-calibration motion_calibration
```
