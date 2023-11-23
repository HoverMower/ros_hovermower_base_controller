#include "hovermower_base_controller.hpp"
#include "config.hpp"
#include "protocol.hpp"

#include <fcntl.h>
#include <termios.h>

HoverMowerBaseController::HoverMowerBaseController(std::string name) : Node(name)
{
     declare_parameter("peri_timeout_below_smag", 200);
     declare_parameter("peri_timeout", 10);
     declare_parameter("port", DEFAULT_PORT);
     // declare_parameter("bumper_pc_radius",0.25);
     // declare_parameter("bumper_pc_height",0.04);
     // declare_parameter("bumper_pc_angle",0.34906585);

     get_parameter("peri_timeout_below_smag", peri_timeout_smag_);
     get_parameter("peri_timeout", peri_timeout_);
     get_parameter("port", port);

     // Register  publisher
     peri_pub = create_publisher<rosmower_msgs::msg::Perimeter>("hovermower/sensors/Perimeter", 3);
     if (BUTTON)
     {
          button_pub = create_publisher<std_msgs::msg::Int32>("hovermower/sensors/Button", 3);
     }
     if (BUMPER)
     {
          bumper_left_pub = create_publisher<std_msgs::msg::Bool>("hovermower/sensors/bumper/left", 3);
          bumper_right_pub = create_publisher<std_msgs::msg::Bool>("hovermower/sensors/bumper/right", 3);
     }

     if (MOW)
     {
          mow_pub = create_publisher<rosmower_msgs::msg::MowMotor>("hovermower/sensors/MowMotor", 3);
     }
     battery_pub = create_publisher<sensor_msgs::msg::BatteryState>("hovermower/sensors/Battery", 3);
     switches_pub = create_publisher<rosmower_msgs::msg::Switches>("hovermower/switches", 3);

     // register Services
     mow_service = create_service<rosmower_msgs::srv::SetMowMotor>("hovermower/setMowMotorSpeed", std::bind(&HoverMowerBaseController::setMowMotorSpeed, this, std::placeholders::_1, std::placeholders::_2));
     calibration_service = create_service<rosmower_msgs::srv::Calibration>("hovermower/doCalibration", std::bind(&HoverMowerBaseController::RequestCalibration, this, std::placeholders::_1, std::placeholders::_2));
     setSwitch_service = create_service<rosmower_msgs::srv::SetSwitch>("hovermower/setSwitch", std::bind(&HoverMowerBaseController::setSwitch, this, std::placeholders::_1, std::placeholders::_2));
     pressSwitch_service = create_service<rosmower_msgs::srv::PressSwitch>("hovermower/pressSwitch", std::bind(&HoverMowerBaseController::pressSwitch, this, std::placeholders::_1, std::placeholders::_2));

     RCLCPP_INFO(get_logger(), "Using port %s", port.c_str());

     // Prepare serial port
     if ((port_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
     {
          RCLCPP_FATAL(get_logger(), "Cannot open serial port to hovermower base controller");
          exit(-1);
     }

     // CONFIGURE THE UART -- connecting to the board
     // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
     struct termios options;
     tcgetattr(port_fd, &options);
     cfmakeraw(&options);
     options.c_cflag |= (CLOCAL | CREAD);
     options.c_iflag &= ~(IXOFF | IXANY);

     // set vtime, vmin, baud rate...
     options.c_cc[VMIN] = 0;  // you likely don't want to change this
     options.c_cc[VTIME] = 0; // or this

     cfsetispeed(&options, BAUDRATE);
     cfsetospeed(&options, BAUDRATE);

     tcflush(port_fd, TCIFLUSH);
     tcsetattr(port_fd, TCSANOW, &options);

     RCLCPP_INFO(get_logger(), "Timeout smag % i", peri_timeout_smag_);
     RCLCPP_INFO(get_logger(), "Timeout % i", peri_timeout_);

     // register parameter change callback handle
     callback_handle_ = this->add_on_set_parameters_callback(
         std::bind(&HoverMowerBaseController::parametersCallback, this, std::placeholders::_1));
}
HoverMowerBaseController::~HoverMowerBaseController()
{
     if (port_fd != -1)
          close(port_fd);
}

void HoverMowerBaseController::write()
{
     if (port_fd == -1)
     {
          RCLCPP_ERROR(get_logger(), "Attempt to write on closed serial");
          return;
     }

     // maybe do some accelleration to slow spped up mow motor

     SerialCommand command;
     command.start = (uint16_t)CMD_FRAME;
     command.mow_rpm = (uint16_t)mow_target_speed_;
     command.switch1 = (uint8_t)switch1;
     command.switch2 = (uint8_t)switch2;
     command.switch3 = (uint8_t)switch3;
     command.calibrate = doCalibration;
     command.checksum = (uint16_t)(command.start ^ command.mow_rpm ^
                                   command.switch1 ^ command.switch2 ^ command.switch3 ^ command.calibrate);

     int rc = ::write(port_fd, (const void *)&command, sizeof(command));
     if (rc < 0)
     {
          RCLCPP_ERROR(get_logger(), "Error writing to rosmower base controller serial port");
     }

     if (doCalibration == true)
     {
          doCalibration = false;
     }

     // if a switch should be pressed for short period only,
     // reset the value back to 0
     if (switch1_pressed_ == true)
     {
          switch1 = 0;
          switch1_pressed_ = false;
     }

     if (switch2_pressed_ == true)
     {
          switch2 = 0;
          switch2_pressed_ = false;
     }

     if (switch3_pressed_ == true)
     {
          switch3 = 0;
          switch3_pressed_ = false;
     }
}

void HoverMowerBaseController::read()
{
     if (port_fd != -1)
     {
          unsigned char c;
          int i = 0, r = 0;

          while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < 1024)
               protocol_recv(c);

          if (i > 0)
               last_read = get_clock()->now();

          if (r < 0 && errno != EAGAIN)
               RCLCPP_ERROR(get_logger(), "Reading from serial %s failed: %d", port.c_str(), r);
     }

     if ((get_clock()->now().seconds() - last_read.seconds()) > 1)
     {
          RCLCPP_FATAL(get_logger(), "Timeout reading from serial %s failed", port.c_str());
     }

     if ((get_clock()->now() - last_valid_message).seconds() > 2)
     {
          RCLCPP_FATAL(get_logger(), "Timeout reading valid message()");
     }
}

void HoverMowerBaseController::protocol_recv(unsigned char byte)
{
     start_frame = ((uint16_t)(byte) << 8) | prev_byte;

     // Read the start frame
     if (start_frame == START_FRAME && msg_len == 0)
     {
          p = (unsigned char *)&msg;
          *p++ = prev_byte;
          *p++ = byte;
          msg_len = 2;
     }
     else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback))
     {
          // Otherwise just read the message content until the end
          *p++ = byte;
          msg_len++;
     }

     if (msg_len == sizeof(SerialFeedback))
     {
          uint16_t checksum = (uint16_t)(msg.start ^
                                         msg.left_mag ^
                                         msg.right_mag ^
                                         msg.left_smag ^
                                         msg.right_smag ^
                                         msg.left_inside ^
                                         msg.right_inside ^
                                         msg.left_timeout ^
                                         msg.right_timeout ^
                                         msg.bumperLeft ^
                                         msg.bumperRight ^
                                         msg.buttonCount ^
                                         msg.calibrated ^
                                         msg.batVoltage ^
                                         msg.chgVoltage ^
                                         msg.chgCurrent ^
                                         msg.chgStatus ^
                                         msg.mowCurrent ^
                                         msg.mowPower ^
                                         msg.mowSpeed ^
                                         msg.mowAlarm ^
                                         msg.switch1 ^
                                         msg.switch2 ^
                                         msg.switch3 ^
                                         msg.calibrate);

          if (msg.start == START_FRAME && msg.checksum == checksum)
          {
               last_valid_message = get_clock()->now();
               rosmower_msgs::msg::Perimeter peri;
               peri.left_mag = msg.left_mag;
               peri.right_mag = msg.right_mag;
               peri.left_smag = msg.left_smag;
               peri.right_smag = msg.right_smag;
               peri.left_inside = msg.left_inside;
               peri.right_inside = msg.right_inside;
               peri.left_timeout = msg.left_timeout;
               peri.right_timeout = msg.right_timeout;
               peri.calibrated = msg.calibrated;

               // additional safety checks
               // first check for timeout based on poor signal
               if (peri.left_smag < peri_timeout_smag_)
                    peri.left_timeout = true;

               if (peri.right_smag < peri_timeout_smag_)
                    peri.right_timeout = true;

               // check for timeout if one coil is outside
               if (peri.left_inside == true)
                    lastTime_left_inside_ = get_clock()->now();

               if (peri.right_inside == true)
                    lastTime_right_inside_ = get_clock()->now();

               rclcpp::Time time = get_clock()->now();
               rclcpp::Duration timeout_left = time - lastTime_left_inside_;
               rclcpp::Duration timeout_right = time - lastTime_right_inside_;

               if (timeout_left.seconds() > peri_timeout_ || timeout_right.seconds() > peri_timeout_)
               {
                    peri.left_timeout = true;
                    peri.right_timeout = true;
               }

               // Publish perimeter message
               peri_pub->publish(peri);

               // Battery message
               sensor_msgs::msg::BatteryState battery;
               battery.voltage = (float)msg.batVoltage / 100;
               battery.current = (float)msg.chgCurrent / 100;
               switch (msg.chgStatus)
               {
               case 0:
                    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
                    break;

               case 1:
                    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
                    break;

               case 2:
                    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
                    break;
               }

               battery_pub->publish(battery);

               // publish additional sensor data
               if (BUMPER)
               {
                    std_msgs::msg::Bool bumperleft;
                    std_msgs::msg::Bool bumperright;
                    if (BUMPER_SWAP_LR)
                    {
                         bumperright.data = msg.bumperLeft;
                         bumperleft.data = msg.bumperRight;
                    }
                    else
                    {
                         bumperleft.data = msg.bumperLeft;
                         bumperright.data = msg.bumperRight;
                    }

                    bumper_left_pub->publish(bumperleft);
                    bumper_right_pub->publish(bumperright);
               }
               if (BUTTON)
               {
                    std_msgs::msg::Int32 button;
                    button.data = msg.buttonCount;
                    button_pub->publish(button);
               }
               if (MOW)
               {
                    rosmower_msgs::msg::MowMotor mow;
                    mow.current = (float)msg.mowCurrent / 100.0;
                    mow.power = (float)msg.mowPower / 100.0;
                    mow.speed = msg.mowSpeed;
                    mow.alarm = msg.mowAlarm;
                    mow_pub->publish(mow);
               }
               rosmower_msgs::msg::Switches switches;
               switches.switch1 = msg.switch1;
               switches.switch2 = msg.switch2;
               switches.switch3 = msg.switch3;
               switches_pub->publish(switches);
          }
          else
          {
               RCLCPP_WARN(get_logger(), "Base Controller checksum mismatch: %d vs %d", msg.checksum, checksum);
          }
          msg_len = 0;
     }
     prev_byte = byte;
}

void HoverMowerBaseController::setMowMotorSpeed(const std::shared_ptr<rosmower_msgs::srv::SetMowMotor::Request> req,
                                                std::shared_ptr<rosmower_msgs::srv::SetMowMotor::Response> resp)
{
     if (!MOW || req->speed > MOW_MAX_SPEED)
     {
          resp->success = false;
          return;
     }

     if (req->speed <= 0)
     {
          mow_target_speed_ = 0;
     }
     else
     {
          mow_target_speed_ = (uint16_t)req->speed;
     }
     resp->success = true;
     RCLCPP_INFO(get_logger(), "set Speed to %i", mow_target_speed_);
     return;
}

void HoverMowerBaseController::RequestCalibration(const std::shared_ptr<rosmower_msgs::srv::Calibration::Request> req,
                                                  std::shared_ptr<rosmower_msgs::srv::Calibration::Response> resp)
{
     doCalibration = req->calibrate;
     return;
}

void HoverMowerBaseController::setSwitch(const std::shared_ptr<rosmower_msgs::srv::SetSwitch::Request> req,
                                         std::shared_ptr<rosmower_msgs::srv::SetSwitch::Response> resp)
{
     switch (req->switch_id)
     {
     case 1:
          switch1 = req->value;
          break;

     case 2:
          switch2 = req->value;
          break;

     case 3:
          switch3 = req->value;
          break;

     default:
          return;
          break;
     }
     return;
}

void HoverMowerBaseController::pressSwitch(const std::shared_ptr<rosmower_msgs::srv::PressSwitch::Request> req,
                                           std::shared_ptr<rosmower_msgs::srv::PressSwitch::Response> resp)
{
     switch (req->switch_id)
     {
     case 1:
          switch1 = 255;
          switch1_pressed_ = true;
          break;

     case 2:
          switch2 = 255;
          switch2_pressed_ = true;
          break;

     case 3:
          switch3 = 255;
          switch3_pressed_ = true;
          break;

     default:
          return;
          break;
     }

     return;
}

rcl_interfaces::msg::SetParametersResult HoverMowerBaseController::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
     rcl_interfaces::msg::SetParametersResult result;
     result.successful = true;
     result.reason = "success";
     // Here update class attributes, do some actions, etc.
     for (const auto &param : parameters)
     {
          if (param.get_name() == "peri_timeout_below_smag")
          {
               peri_timeout_smag_ = param.as_int();
               RCLCPP_INFO(get_logger(), "new value for perimeter timeout smag: %i", peri_timeout_smag_);
          }
          if (param.get_name() == "peri_timeout")
          {
               peri_timeout_ = param.as_int();
               RCLCPP_INFO(get_logger(), "new value for perimeter timeout: %i", peri_timeout_);
          }
     }

     return result;
}
