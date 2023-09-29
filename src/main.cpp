#include "hovermower_base_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto controller = std::make_shared<HoverMowerBaseController>("base_controller");

    rclcpp::Rate rate(20.0);
    int rate_counter = 0;

    while (rclcpp::ok())
    {
        rate_counter++;
        controller->read();
        if (rate_counter > 9)
        {
            controller->write(); // reduce this rate
            rate_counter = 0;
        }
        rclcpp::spin_some(controller);
        rate.sleep();
    }

    return 0;
}
