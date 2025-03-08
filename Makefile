all: interfaces common lane-detection manual-control obstacle-avoidance path-planning motion-calibration main-pipeline

interfaces:
	colcon build --packages-select interfaces

common:
	colcon build --packages-select common

main-pipeline:
	colcon build --packages-select main-pipeline

lane-detection:
	colcon build --packages-select lane-detection

manual-control:
	colcon build --packages-select manual-control

obstacle-avoidance:
	colcon build --packages-select obstacle-avoidance

path-planning:
	colcon build --packages-select path-planning

motion-calibration:
	colcon build --packages-select motion-calibration

tests:
	colcon test --event-handlers console_direct+

clean:
	rm -Rf build
	rm -Rf install
	rm -Rf log
