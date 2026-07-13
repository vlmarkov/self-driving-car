#include <chasiss-control/ros_module.h>

#include <chasiss-control/chasiss_control.h>

#ifdef WIRING_PI_LIB
#include <wiringPi.h>
#endif // WIRING_PI_LIB

using namespace std::chrono_literals;

void run(std::stop_token stop_token, std::shared_ptr<ChasisControl> chasiss) {
    while(!stop_token.stop_requested()) {
        chasiss->process_motion_vector();
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto cfg = PubSubCfg{
        .name = ChasisControl::kName,
        .topic_publiser = std::string(ChasisControl::kName) + "Out",
        .topic_subscription = std::string(ChasisControl::kName) + "In",
        .duration = 500ms
    };
    auto pub_sub_node = std::make_shared<BasePubSubNode>(cfg);

    ChasissConfig chasiss_cfg;
    pub_sub_node->declare_parameter<int>("engine_left_pwm_pin", 1);
    pub_sub_node->declare_parameter<int>("engine_left_reverse_pin", 0);
    pub_sub_node->declare_parameter<int>("engine_left_forward_pin", 2);
    pub_sub_node->declare_parameter<int>("engine_right_forward_pin", 3);
    pub_sub_node->declare_parameter<int>("engine_right_reverse_pin", 4);
    pub_sub_node->declare_parameter<int>("engine_right_pwm_pin", 24);
    pub_sub_node->declare_parameter<int>("pwm_medium", 256);
    pub_sub_node->declare_parameter<int>("pwm_max", 512);
    pub_sub_node->declare_parameter<int>("pwm_turn", 300);
    pub_sub_node->declare_parameter<int>("timeout_before_turn", 100);
    pub_sub_node->declare_parameter<int>("engine_turn_delay", 25);
    pub_sub_node->declare_parameter<int>("engine_turn_angle", 3);
    pub_sub_node->declare_parameter<int>("frames_to_get_trajectory", 10);

    chasiss_cfg.engine_left_pwm_pin = pub_sub_node->get_parameter("engine_left_pwm_pin").as_int();
    chasiss_cfg.engine_left_reverse_pin = pub_sub_node->get_parameter("engine_left_reverse_pin").as_int();
    chasiss_cfg.engine_left_forward_pin = pub_sub_node->get_parameter("engine_left_forward_pin").as_int();

    chasiss_cfg.engine_right_forward_pin = pub_sub_node->get_parameter("engine_right_forward_pin").as_int();
    chasiss_cfg.engine_right_reverse_pin = pub_sub_node->get_parameter("engine_right_reverse_pin").as_int();
    chasiss_cfg.engine_right_pwm_pin = pub_sub_node->get_parameter("engine_right_pwm_pin").as_int();

    chasiss_cfg.pwm_medium = pub_sub_node->get_parameter("pwm_medium").as_int();
    chasiss_cfg.pwm_max = pub_sub_node->get_parameter("pwm_max").as_int();
    chasiss_cfg.pwm_turn = pub_sub_node->get_parameter("pwm_turn").as_int();

    chasiss_cfg.timeout_before_turn = pub_sub_node->get_parameter("timeout_before_turn").as_int();
    chasiss_cfg.engine_turn_delay = pub_sub_node->get_parameter("engine_turn_delay").as_int();
    chasiss_cfg.engine_turn_angle = pub_sub_node->get_parameter("engine_turn_angle").as_int();
    chasiss_cfg.frames_to_get_trajectory = pub_sub_node->get_parameter("frames_to_get_trajectory").as_int();

    std::cout << "engine_left_pwm_pin      " << chasiss_cfg.engine_left_pwm_pin << std::endl;
    std::cout << "engine_left_reverse_pin  " << chasiss_cfg.engine_left_reverse_pin << std::endl;
    std::cout << "engine_left_forward_pin  " << chasiss_cfg.engine_left_forward_pin << std::endl;
    std::cout << "engine_right_forward_pin " << chasiss_cfg.engine_right_forward_pin << std::endl;
    std::cout << "engine_right_reverse_pin " << chasiss_cfg.engine_right_reverse_pin << std::endl;
    std::cout << "engine_right_pwm_pin     " << chasiss_cfg.engine_right_pwm_pin << std::endl;
    std::cout << "pwm_medium               " << chasiss_cfg.pwm_medium << std::endl;
    std::cout << "pwm_max                  " << chasiss_cfg.pwm_max << std::endl;
    std::cout << "pwm_turn                 " << chasiss_cfg.pwm_turn << std::endl;
    std::cout << "timeout_before_turn      " << chasiss_cfg.timeout_before_turn << std::endl;
    std::cout << "engine_turn_delay        " << chasiss_cfg.engine_turn_delay << std::endl;
    std::cout << "engine_turn_angle        " << chasiss_cfg.engine_turn_angle << std::endl;
    std::cout << "frames_to_get_trajectory " << chasiss_cfg.frames_to_get_trajectory << std::endl;

    auto chasiss = std::make_shared<ChasisControl>(pub_sub_node, chasiss_cfg);

    std::jthread thread(run, chasiss);

    rclcpp::spin(pub_sub_node);
    rclcpp::shutdown();

    return 0;
}
