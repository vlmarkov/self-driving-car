BUILD_DIR   := build
INSTALL_DIR := install
LOG_DIR     := log

# Allows to manually set or disable unit tests
ENABLE_TESTS               := ON
# Allows to manually set or disable compiler warings VALID FOR GCC OR CLANG
ENABLE_COMPILER_OPTIONS    := ON
# Allows to manually set or disable c++23 standard features
ENABLE_LATEST_CPP_STANDARD := ON
# To debug purpose on your laptop (NOT Raspberry board) you can disable gpio write feature
ENABLE_WIRING_PI_LIB       := OFF
ENABLE_RASPBERRY_BUILD     := OFF
ENABLE_RASPBERRY_DEBUG_IMG := OFF

all: interfaces common lane-detection manual-control motion-calibration main-pipeline

interfaces:
	colcon build --packages-select interfaces --cmake-args -DENABLE_TESTS=$(ENABLE_TESTS) -DENABLE_COMPILER_OPTIONS=$(ENABLE_COMPILER_OPTIONS) -DENABLE_LATEST_CPP_STANDARD=$(ENABLE_LATEST_CPP_STANDARD)

common:
	colcon build --packages-select common --cmake-args -DENABLE_TESTS=$(ENABLE_TESTS) -DENABLE_COMPILER_OPTIONS=$(ENABLE_COMPILER_OPTIONS) -DENABLE_LATEST_CPP_STANDARD=$(ENABLE_LATEST_CPP_STANDARD)

main-pipeline:
	colcon build --packages-select main-pipeline --cmake-args -DENABLE_TESTS=$(ENABLE_TESTS) -DENABLE_COMPILER_OPTIONS=$(ENABLE_COMPILER_OPTIONS) -DENABLE_LATEST_CPP_STANDARD=$(ENABLE_LATEST_CPP_STANDARD)

lane-detection:
	colcon build --packages-select lane-detection --cmake-args -DENABLE_TESTS=$(ENABLE_TESTS) -DENABLE_COMPILER_OPTIONS=$(ENABLE_COMPILER_OPTIONS) -DENABLE_LATEST_CPP_STANDARD=$(ENABLE_LATEST_CPP_STANDARD) -DENABLE_RASPBERRY_BUILD=$(ENABLE_RASPBERRY_BUILD) -DENABLE_RASPBERRY_DEBUG_IMG=$(ENABLE_RASPBERRY_DEBUG_IMG)

manual-control:
	colcon build --packages-select manual-control --cmake-args -DENABLE_TESTS=$(ENABLE_TESTS) -DENABLE_COMPILER_OPTIONS=$(ENABLE_COMPILER_OPTIONS) -DENABLE_LATEST_CPP_STANDARD=$(ENABLE_LATEST_CPP_STANDARD)

motion-calibration:
	colcon build --packages-select motion-calibration --cmake-args -DENABLE_TESTS=$(ENABLE_TESTS) -DENABLE_COMPILER_OPTIONS=$(ENABLE_COMPILER_OPTIONS) -DENABLE_LATEST_CPP_STANDARD=$(ENABLE_LATEST_CPP_STANDARD) -DENABLE_WIRING_PI_LIB=$(ENABLE_WIRING_PI_LIB)

tests:
	colcon test --packages-select interfaces --event-handlers console_direct+
	colcon test --packages-select common --event-handlers console_direct+
	colcon test --packages-select main-pipeline --event-handlers console_direct+
	colcon test --packages-select lane-detection --event-handlers console_direct+
	colcon test --packages-select manual-control --event-handlers console_direct+
	colcon test --packages-select motion-calibration --event-handlers console_direct+

clean:
	rm -Rf $(BUILD_DIR)
	rm -Rf $(INSTALL_DIR)
	rm -Rf $(LOG_DIR)
