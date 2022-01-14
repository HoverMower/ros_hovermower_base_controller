#ifndef _HOVERMOWER_BASE_CONTROLLER_H
#define _HOVERMOWER_BASE_CONTROLLER_H

#include <ros/ros.h>
#include "protocol.h"
#include "rosmower_msgs/PerimeterMsg.h"
#include "rosmower_msgs/Bumper.h"
#include "rosmower_msgs/Battery.h"
#include "rosmower_msgs/MowMotor.h"
#include "rosmower_msgs/setMowMotor.h"
#include "rosmower_msgs/doCalibration.h"
#include "rosmower_msgs/setSwitch.h"
#include "std_msgs/Int32.h"
#include <dynamic_reconfigure/server.h>
#include <ros_hovermower_base_controller/HoverMowerBaseControllerConfig.h>

class HoverMowerBaseController
{
public:
    HoverMowerBaseController();
    ~HoverMowerBaseController();

    void read();
    void write();
    void dyn_callback(ros_hovermower_base_controller::HoverMowerBaseControllerConfig &config, uint32_t level);


private:
    void protocol_recv(unsigned char c);
    bool setMowMotorSpeed(rosmower_msgs::setMowMotor::Request &req,
                          rosmower_msgs::setMowMotor::Response &resp );
    
    bool RequestCalibration(rosmower_msgs::doCalibration::Request &req,
                          rosmower_msgs::doCalibration::Response &resp );

    bool setSwitch(rosmower_msgs::setSwitch::Request &req,
                          rosmower_msgs::setSwitch::Response &resp );
    // Publishers
    ros::NodeHandle nh;
    ros::Publisher peri_pub;
    ros::Publisher button_pub;
    ros::Publisher bumper_pub;
    ros::Publisher mow_pub;
    ros::Publisher battery_pub;

    // Services
    ros::ServiceServer mow_service;
    ros::ServiceServer calibration_service;
    ros::ServiceServer setSwitch_service;

    ros::Time last_read;

    // base controller protocol
    int port_fd;
    int msg_len = 0;
    unsigned char prev_byte = 0;
    uint16_t start_frame = 0;
    unsigned char *p;
    SerialFeedback msg;

    // additional perimeter attributes
    int peri_timeout_smag_; // timeout if smag below
    int peri_timeout_;      // timeout if one coil is not inside peri loop
    ros::Time lastTime_left_inside_;
    ros::Time lastTime_right_inside_;

    //mow motor attribures
    uint16_t mow_target_speed_ = 0; // target speed of mow motor

    // calibration requested?
    bool doCalibration = false;

    // values of user switches (Relais etc.)
    uint8_t switch1 = 0;
    uint8_t switch2 = 0;
    uint8_t switch3 = 0;

    // dynamic reconfigure
    typedef dynamic_reconfigure::Server<ros_hovermower_base_controller::HoverMowerBaseControllerConfig> DynamicReconfigServer;
    boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
    DynamicReconfigServer::CallbackType param_reconfig_callback_;
};

#endif