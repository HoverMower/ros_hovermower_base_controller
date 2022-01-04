#ifndef _HOVERMOWER_BASE_CONTROLLER_H
#define _HOVERMOWER_BASE_CONTROLLER_H

#include <ros/ros.h>
#include "protocol.h"
#include "rosmower_msgs/PerimeterMsg.h"
#include "rosmower_msgs/Bumper.h"
#include "std_msgs/Int32.h"
#include <dynamic_reconfigure/server.h>
#include <HoverMowerBaseControllerConfig.h>

class HoverMowerBaseController
{
public:
    HoverMowerBaseController();
    ~HoverMowerBaseController();

    void read();
    void write();
    void dyn_callback(hovermower_base_controller::HoverMowerBaseControllerConfig &config, uint32_t level);

private:
    void protocol_recv(unsigned char c);
    
    // Publishers
    ros::NodeHandle nh;
    ros::Publisher peri_pub;
    ros::Publisher button_pub;
    ros::Publisher bumper_pub;

    ros::Time last_read;

    // Perimeter protocol
    int port_fd;
    int msg_len = 0;
    unsigned char prev_byte = 0;
    uint16_t start_frame = 0;
    unsigned char *p;
    SerialFeedback msg;

    // additional attributes
    int peri_timeout_smag_; // timeout if smag below
    int peri_timeout_;      // timeout if one coil is not inside peri loop
    ros::Time lastTime_left_inside_;
    ros::Time lastTime_right_inside_;

    // dynamic reconfigure
    typedef dynamic_reconfigure::Server<hovermower_base_controller::PerimeterConfig> DynamicReconfigServer;
    boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
    DynamicReconfigServer::CallbackType param_reconfig_callback_;
};

#ENDIF