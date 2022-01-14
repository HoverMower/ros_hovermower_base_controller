#include "hovermower_base_controller.h"
#include "config.h"
#include "protocol.h"

#include <fcntl.h>
#include <termios.h>

HoverMowerBaseController::HoverMowerBaseController()
{

    // Register  publisher
    peri_pub = nh.advertise<rosmower_msgs::PerimeterMsg>("sensors/perimeter", 3);
    if (BUTTON)
    {
        button_pub = nh.advertise<std_msgs::Int32>("sensors/Button", 3);
    }
    if (BUMPER)
    {
        bumper_pub = nh.advertise<rosmower_msgs::Bumper>("sensors/Bumper", 3);
    }
    if (MOW)
    {
        mow_pub = nh.advertise<rosmower_msgs::MowMotor>("sensors/MowMotor", 3);
    }
    battery_pub = nh.advertise<rosmower_msgs::Battery>("sensors/Battery", 3);

    // register Services
    mow_service = nh.advertiseService("setMowMotorSpeed", &HoverMowerBaseController::setMowMotorSpeed, this);
    calibration_service = nh.advertiseService("doCalibration", &HoverMowerBaseController::RequestCalibration, this);
    setSwitch_service = nh.advertiseService("setSwitch", &HoverMowerBaseController::setSwitch, this);

    // Prepare serial port
    if ((port_fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    {
        ROS_FATAL("Cannot open serial port to hovermower base controller");
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

    param_reconfig_callback_ = boost::bind(&HoverMowerBaseController::dyn_callback, this, _1, _2);

    param_reconfig_server_.reset(new DynamicReconfigServer());
    param_reconfig_server_->setCallback(param_reconfig_callback_);
    ROS_INFO_NAMED("base controller", "Initialized dynamic reconfigure");
    ROS_INFO_NAMED("base controller", "Timeout smag % i", peri_timeout_smag_);
    ROS_INFO_NAMED("base controller", "Timeout % i", peri_timeout_);
}

HoverMowerBaseController::~HoverMowerBaseController()
{
    if (port_fd != -1)
        close(port_fd);
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
            last_read = ros::Time::now();

        if (r < 0 && errno != EAGAIN)
            ROS_ERROR("Reading from serial %s failed: %d", PORT, r);
    }

    if ((ros::Time::now() - last_read).toSec() > 1)
    {
        ROS_FATAL("Timeout reading from serial %s failed", PORT);
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
            rosmower_msgs::PerimeterMsg peri;

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
                lastTime_left_inside_ = ros::Time::now();

            if (peri.right_inside == true)
                lastTime_right_inside_ = ros::Time::now();

            ros::Time time = ros::Time::now();
            ros::Duration timeout_left = time - lastTime_left_inside_;
            ros::Duration timeout_right = time - lastTime_right_inside_;

            if (timeout_left.toSec() > peri_timeout_ || timeout_right.toSec() > peri_timeout_)
            {
                peri.left_timeout = true;
                peri.right_timeout = true;
            }

            // Publish perimeter message
            peri_pub.publish(peri);

            // Battery message
            rosmower_msgs::Battery battery;
            battery.battery_voltage = (float)msg.batVoltage / 100;
            battery.charge_voltage = (float)msg.chgVoltage / 100;
            battery.charge_current = (float)msg.chgCurrent / 100;
            battery.charge_status = msg.chgStatus;
            battery_pub.publish(battery);

            // publish additional sensor data
            if (BUMPER)
            {
                rosmower_msgs::Bumper bumper;
                bumper.left = msg.bumperLeft;
                bumper.right = msg.bumperRight;

                bumper_pub.publish(bumper);
            }
            if (BUTTON)
            {
                std_msgs::Int32 button;
                button.data = msg.buttonCount;
                button_pub.publish(button);
            }
            if (MOW)
            {
                rosmower_msgs::MowMotor mow;
                mow.current = (float)msg.mowCurrent / 100.0;
                mow.power = (float)msg.mowPower / 100.0;
                mow.speed = msg.mowSpeed;
                mow.alarm = msg.mowAlarm;
                mow_pub.publish(mow);
            }
        }
        else
        {
            ROS_WARN("Base Controller checksum mismatch: %d vs %d", msg.checksum, checksum);
        }
        msg_len = 0;
    }
    prev_byte = byte;
}

void HoverMowerBaseController::dyn_callback(ros_hovermower_base_controller::HoverMowerBaseControllerConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: timeout_smag: %i timeout: %i ",
             config.peri_timeout_below_smag,
             config.peri_timeout);

    peri_timeout_smag_ = config.peri_timeout_below_smag;
    peri_timeout_ = config.peri_timeout;
}

bool HoverMowerBaseController::setMowMotorSpeed(rosmower_msgs::setMowMotor::Request &req,
                                                rosmower_msgs::setMowMotor::Response &resp)
{
    if (!MOW || req.Speed > MOW_MAX_SPEED)
    {
        resp.Success = false;
        return false;
    }

    if (req.Speed <= 0)
    {
        mow_target_speed_ = 0;
    }
    else
    {
        mow_target_speed_ = (uint16_t)req.Speed;
    }
    resp.Success = true;
    ROS_INFO("set Speed to %i", mow_target_speed_);
    return true;
}

bool HoverMowerBaseController::RequestCalibration(rosmower_msgs::doCalibration::Request &req,
                                                  rosmower_msgs::doCalibration::Response &resp)
{
    doCalibration = req.calibrate;
    return true;
}

bool HoverMowerBaseController::setSwitch(rosmower_msgs::setSwitch::Request &req,
                                         rosmower_msgs::setSwitch::Response &resp)
{
    switch (req.switch_id)
    {
    case 1:
        switch1 = req.value;
        return true;
        break;

    case 2:
        switch2 = req.value;
        return true;
        break;

    case 3:
        switch3 = req.value;
        return true;
        break;

    default:
        return false;
        break;
    }
    return true;
}

void HoverMowerBaseController::write()
{
    if (port_fd == -1)
    {
        ROS_ERROR("Attempt to write on closed serial");
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
        ROS_ERROR("Error writing to hoverboard serial port");
    }

    if (doCalibration == true)
    {
        doCalibration = false;
    }
}