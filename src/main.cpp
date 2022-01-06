#include <ros/ros.h>
#include "hovermower_base_controller.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "HoverMower_base_controller");

    HoverMowerBaseController controller;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate rate(100.0);

    while (ros::ok()) {

        controller.read();
        rate.sleep();
    }

    return 0;
}
