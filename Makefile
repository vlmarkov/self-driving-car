all: main-pipeline line-detection manual-control obstacle-avoidance path-planning motion-calibration

main-pipeline:
	colcon build --packages-select main-pipeline

line-detection:
	colcon build --packages-select line-detection

manual-control:
	colcon build --packages-select manual-control

obstacle-avoidance:
	colcon build --packages-select obstacle-avoidance

path-planning:
	colcon build --packages-select path-planning

motion-calibration:
	colcon build --packages-select motion-calibration
