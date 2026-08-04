# Lane Detection Module

Uses OpenCV to extract from video stream images road lanes and steer angle to drive self-driving robot car.

## Features
- read raw video stream from Raspberry Pi camera
- transfrom video frame through pipeline: graysscale, gaussian blur, extract roi, transfrom persepctive, canny edge detection
- detect road lanes
- convert road lane to steer angle

## How To Run
```
ros2 run lane-detection lane-detection --ros-args --params-file ./ros2-modules/lane-detection/lane_detection.param.yaml
```
