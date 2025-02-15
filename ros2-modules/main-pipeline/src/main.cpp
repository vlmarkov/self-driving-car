#include <main-pipeline/main_pipeline.h>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainPipeline>());
    rclcpp::shutdown();
    return 0;
}
