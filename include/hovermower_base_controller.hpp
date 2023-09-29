#ifndef _HOVERMOWER_BASE_CONTROLLER_H
#define _HOVERMOWER_BASE_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include "protocol.hpp"
#include "rosmower_msgs/msg/perimeter.hpp"
#include "rosmower_msgs/msg/bumper.hpp"
#include "rosmower_msgs/msg/battery.hpp"
#include "rosmower_msgs/msg/mow_motor.hpp"
#include "rosmower_msgs/msg/switches.hpp"
#include "rosmower_msgs/srv/set_mow_motor.hpp"
#include "rosmower_msgs/srv/calibration.hpp"
#include "rosmower_msgs/srv/set_switch.hpp"
#include "rosmower_msgs/srv/press_switch.hpp"
#include "std_msgs/msg/int32.hpp"
#include <string>

class HoverMowerBaseController : public rclcpp::Node
{
public:
    HoverMowerBaseController(std::string name);
    ~HoverMowerBaseController();

    void read();
    void write();

private:
    void protocol_recv(unsigned char c);
    void setMowMotorSpeed(const std::shared_ptr<rosmower_msgs::srv::SetMowMotor::Request> req,
                          std::shared_ptr<rosmower_msgs::srv::SetMowMotor::Response> resp);

    void RequestCalibration(const std::shared_ptr<rosmower_msgs::srv::Calibration::Request> req,
                            std::shared_ptr<rosmower_msgs::srv::Calibration::Response> resp);

    void setSwitch(const std::shared_ptr<rosmower_msgs::srv::SetSwitch::Request> req,
                   std::shared_ptr<rosmower_msgs::srv::SetSwitch::Response> resp);
    void pressSwitch(const std::shared_ptr<rosmower_msgs::srv::PressSwitch::Request> req,
                     std::shared_ptr<rosmower_msgs::srv::PressSwitch::Response> resp);

    void timerCallback();

    // Publishers
    rclcpp::Publisher<rosmower_msgs::msg::Perimeter>::SharedPtr peri_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr button_pub;
    rclcpp::Publisher<rosmower_msgs::msg::Bumper>::SharedPtr bumper_pub;
    rclcpp::Publisher<rosmower_msgs::msg::MowMotor>::SharedPtr mow_pub;
    rclcpp::Publisher<rosmower_msgs::msg::Battery>::SharedPtr battery_pub;
    rclcpp::Publisher<rosmower_msgs::msg::Switches>::SharedPtr switches_pub;
    // rclcpp::Publisher bumper_pointcloud_pub;

    // Services
    rclcpp::Service<rosmower_msgs::srv::SetMowMotor>::SharedPtr mow_service;
    rclcpp::Service<rosmower_msgs::srv::Calibration>::SharedPtr calibration_service;
    rclcpp::Service<rosmower_msgs::srv::SetSwitch>::SharedPtr setSwitch_service;
    rclcpp::Service<rosmower_msgs::srv::PressSwitch>::SharedPtr pressSwitch_service;

    rclcpp::Time last_read = get_clock()->now();;
    rclcpp::Time last_valid_message = get_clock()->now();;

    // base controller protocol
    int port_fd;
    std::string port;
    unsigned long int msg_len = 0;
    unsigned char prev_byte = 0;
    uint16_t start_frame = 0;
    unsigned char *p;
    SerialFeedback msg;

    // additional perimeter attributes
    int peri_timeout_smag_; // timeout if smag below
    int peri_timeout_;      // timeout if one coil is not inside peri loop
    rclcpp::Time lastTime_left_inside_ = get_clock()->now();
    rclcpp::Time lastTime_right_inside_= get_clock()->now();

    // mow motor attribures
    uint16_t mow_target_speed_ = 0; // target speed of mow motor

    // calibration requested?
    bool doCalibration = false;

    // values of user switches (Relais etc.)
    uint8_t switch1 = 0;
    uint8_t switch2 = 0;
    uint8_t switch3 = 0;
    bool switch1_pressed_ = false;
    bool switch2_pressed_ = false;
    bool switch3_pressed_ = false;

    // Define a timer for periodic check
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif