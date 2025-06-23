# Motion Calibration

## Basic Information
This module responsible to process a basic motion calibration actions:
- smooth forward/backward
- smooth stop
- smooth turn left/right

## Feature
- low latency car movement changing

## Connection Scheme
Connection between Raspberry Pi5 and L298n DC motor driver board.

```
Raspberry Pi5                L298n Pins
     [PHYS11]   -----------> [IN1]
     [PHYS13]   -----------> [IN2]
     [PHYS15]   -----------> [IN3]
     [PHYS16]   -----------> [IN4]
     [PHYS12]   -----------> [ENA]
     [PHYS35]   -----------> [ENB]
```

## Pins Descriptions
- Pin 11 forward command motor a
- Pin 13 reverse command motor a
- Pin 15 forward command motor b
- Pin 36 reverse command motor b
- Pin 12 pwm motor a
- Pin 35 pwm motor b

## How To Run

It is better to run it under root permission.
Because wiringPi library requires access to `/dev/mem` for PWM managment.

```
sudo bash -c "chmod g+rw /dev/gpiomem0 && source /opt/ros/jazzy/setup.bash && source install/local_setup.bash && ros2 run motion-calibration motion_calibration"
```
