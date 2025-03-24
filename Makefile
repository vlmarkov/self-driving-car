BUILD_DIR                 := build
INSTALL_DIR               := install
LOG_DIR                   := log
# Allows to manually set or disable unit tests
ENABLE_TESTS              := True
# Allows to manually set or disable compiler warings VALID FOR GCC OR CLANG
ADVANCED_COMPILER_OPTIONS := True
# Allows to manually set or disable c++23 standard features
LATEST_CPP_STANDARD       := True

all: interfaces common lane-detection manual-control obstacle-avoidance path-planning motion-calibration main-pipeline arduino-code

arduino-code:
	mkdir -p $(BUILD_DIR)
	cd ./$(BUILD_DIR) && cmake -DBUILD_TESTING=$(ENABLE_TESTS) -DADVANCED_COMPILER_OPTIONS=$(ADVANCED_COMPILER_OPTIONS) -DLATEST_CPP_STANDARD=$(LATEST_CPP_STANDARD) ../arduino/ && make

interfaces:
	colcon build --packages-select interfaces --cmake-args -DBUILD_TESTING=$(ENABLE_TESTS) -DADVANCED_COMPILER_OPTIONS=$(ADVANCED_COMPILER_OPTIONS) -DLATEST_CPP_STANDARD=$(LATEST_CPP_STANDARD)

common:
	colcon build --packages-select common --cmake-args -DBUILD_TESTING=$(ENABLE_TESTS) -DADVANCED_COMPILER_OPTIONS=$(ADVANCED_COMPILER_OPTIONS) -DLATEST_CPP_STANDARD=$(LATEST_CPP_STANDARD)

main-pipeline:
	colcon build --packages-select main-pipeline --cmake-args -DBUILD_TESTING=$(ENABLE_TESTS) -DADVANCED_COMPILER_OPTIONS=$(ADVANCED_COMPILER_OPTIONS) -DLATEST_CPP_STANDARD=$(LATEST_CPP_STANDARD)

lane-detection:
	colcon build --packages-select lane-detection --cmake-args -DBUILD_TESTING=$(ENABLE_TESTS) -DADVANCED_COMPILER_OPTIONS=$(ADVANCED_COMPILER_OPTIONS) -DLATEST_CPP_STANDARD=$(LATEST_CPP_STANDARD)

manual-control:
	colcon build --packages-select manual-control --cmake-args -DBUILD_TESTING=$(ENABLE_TESTS) -DADVANCED_COMPILER_OPTIONS=$(ADVANCED_COMPILER_OPTIONS) -DLATEST_CPP_STANDARD=$(LATEST_CPP_STANDARD)

obstacle-avoidance:
	colcon build --packages-select obstacle-avoidance --cmake-args -DBUILD_TESTING=$(ENABLE_TESTS) -DADVANCED_COMPILER_OPTIONS=$(ADVANCED_COMPILER_OPTIONS) -DLATEST_CPP_STANDARD=$(LATEST_CPP_STANDARD)

path-planning:
	colcon build --packages-select path-planning --cmake-args -DBUILD_TESTING=$(ENABLE_TESTS) -DADVANCED_COMPILER_OPTIONS=$(ADVANCED_COMPILER_OPTIONS) -DLATEST_CPP_STANDARD=$(LATEST_CPP_STANDARD)

motion-calibration:
	colcon build --packages-select motion-calibration --cmake-args -DBUILD_TESTING=$(ENABLE_TESTS) -DADVANCED_COMPILER_OPTIONS=$(ADVANCED_COMPILER_OPTIONS) -DLATEST_CPP_STANDARD=$(LATEST_CPP_STANDARD)

tests:
	colcon test --packages-select interfaces --event-handlers console_direct+
	colcon test --packages-select common --event-handlers console_direct+
	colcon test --packages-select main-pipeline --event-handlers console_direct+
	colcon test --packages-select lane-detection --event-handlers console_direct+
	colcon test --packages-select manual-control --event-handlers console_direct+
	colcon test --packages-select obstacle-avoidance --event-handlers console_direct+
	colcon test --packages-select path-planning --event-handlers console_direct+
	colcon test --packages-select motion-calibration --event-handlers console_direct+
	cd ./$(BUILD_DIR)/tests && ctest

clean:
	rm -Rf $(BUILD_DIR)
	rm -Rf $(INSTALL_DIR)
	rm -Rf $(LOG_DIR)
