#include <ros/ros.h>
#include "hovermower_base_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HoverMower_base_controller");

    HoverMowerBaseController controller;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate rate(100.0);
    int rate_counter = 0;

    while (ros::ok())
    {
        rate_counter++;
        controller.read();
        if (rate_counter > 9)
        {
            controller.write(); // reduce this rate
            rate_counter = 0;
        }

        rate.sleep();
    }

    return 0;
}
