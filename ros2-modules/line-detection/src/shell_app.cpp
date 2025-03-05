#include <line-detection/lane_detection_module.h>

int main(int argc, char* argv[]) {
    LaneDetectionModule lm;
    lm.detectLane(argv[1]);
    return 0;
}
