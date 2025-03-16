#include <main-pipeline/main_pipeline.h>

#include <manual-control/ros_module.h>
#include <path-planning/path_planning.h>
#include <obstacle-avoidance/obstacle_avoidance.h>
#include <lane-detection/lane_detection.h>
#include <motion-calibration/ros_module.h>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto main_pipeline = std::make_shared<MainPipeline>();

    main_pipeline->add_module(ManualControl::kName);
    main_pipeline->add_module(PathPlanning::kName);
    main_pipeline->add_module(ObstacleAvoidance::kName);
    main_pipeline->add_module(LaneDetection::kName);
    main_pipeline->add_module(MotionCalibration::kName);

    rclcpp::spin(main_pipeline);
    rclcpp::shutdown();

    return 0;
}
