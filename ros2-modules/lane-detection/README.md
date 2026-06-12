# Lane Detection Module

## Basic Information
This module allows to detect road lanes from video stream.

## Features
- read raw video stream from raspberry pi camera
- transfrom video frame through pipeline: graysscale, gaussian blur, extract roi, transfrom persepctive, canny edge detection
- after that run extract road lanes procedure
- and detect steer angle

## How To Run
```
ros2 run lane-detection lane-detection --ros-args --params-file ./ros2-modules/lane-detection/lane_detection.param.yaml
```
