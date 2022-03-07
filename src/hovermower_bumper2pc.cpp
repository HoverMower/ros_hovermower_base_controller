// code and idea taken from here
// https://github.com/yujinrobot/kobuki/blob/melodic/kobuki_bumper2pc/src/kobuki_bumper2pc.cpp

/**
 * @file /src/kobuki_bumper2pc.cpp
 *
 * @brief Bumper to pointcloud nodelet class implementation.
 *
 * @author Jorge Santos, Yujin Robot
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <pluginlib/class_list_macros.h>

#include "hovermower_bumper2pc.h"

namespace ros_hovermower_base_controller
{

    void Bumper2PcNodelet::bc_bumperCB(const rosmower_msgs::Bumper::ConstPtr &msg)
    {
        if (bumper_pointcloud_pub_.getNumSubscribers() == 0 )
            return;

        // We publish just one "no events" pc (with all three points far away) and stop spamming when bumper/cliff conditions disappear
        if (!msg->left && !msg->right && !prev_bumper_left_ && !prev_bumper_right_)
            return;

        prev_bumper_left_ = msg->left;
        prev_bumper_right_ = msg->right;

        // For any of {left/right} with no bumper event, we publish a faraway point that won't get used
        if (msg->left)
        {
            memcpy(&bumper_left_pc_.data[0 * bumper_left_pc_.point_step + bumper_left_pc_.fields[0].offset], &p_side_x_, sizeof(float));
            memcpy(&bumper_left_pc_.data[0 * bumper_left_pc_.point_step + bumper_left_pc_.fields[1].offset], &p_side_y_, sizeof(float));
        }
        else
        {
            memcpy(&bumper_left_pc_.data[0 * bumper_left_pc_.point_step + bumper_left_pc_.fields[0].offset], &P_INF_X, sizeof(float));
            memcpy(&bumper_left_pc_.data[0 * bumper_left_pc_.point_step + bumper_left_pc_.fields[1].offset], &P_INF_Y, sizeof(float));
        }

        if (msg->right)
        {
            memcpy(&bumper_right_pc_.data[0 * bumper_right_pc_.point_step + bumper_right_pc_.fields[0].offset], &p_side_x_, sizeof(float));
            memcpy(&bumper_right_pc_.data[0 * bumper_right_pc_.point_step + bumper_right_pc_.fields[1].offset], &p_side_y_, sizeof(float));
        }
        else
        {
            memcpy(&bumper_right_pc_.data[0 * bumper_right_pc_.point_step + bumper_right_pc_.fields[0].offset], &P_INF_X, sizeof(float));
            memcpy(&bumper_right_pc_.data[0 * bumper_right_pc_.point_step + bumper_right_pc_.fields[1].offset], &P_INF_Y, sizeof(float));
        }

        bumper_left_pc_.header.stamp = bumper_right_pc_.header.stamp = ros::Time::now();

        bumper_left_pc_pub_.publish(bumper_left_pc_);
        bumper_right_pc_pub_.publish(bumper_right_pc_);
    }

    void Bumper2PcNodelet::onInit()
    {
        ros::NodeHandle nh = this->getPrivateNodeHandle();

        param_reconfig_callback_ = boost::bind(&Bumper2PcNodelet::dyn_callback, this, _1, _2);

        param_reconfig_server_.reset(new DynamicReconfigServer());
        param_reconfig_server_->setCallback(param_reconfig_callback_);

        // Bumper pointcloud distance to base frame; should be something like the robot radius plus
        // costmap resolution plus an extra to cope with robot inertia. This is a bit tricky parameter: if
        // it's too low, costmap will ignore this pointcloud (the robot footprint runs over the hit obstacle),
        // but if it's too big, hit obstacles will be mapped too far from the robot and the navigation around
        // them will probably fail.

        // double r, h, angle;
        // nh.param("pointcloud_radius", r, 0.25);
        // pc_radius_ = r;
        // nh.param("pointcloud_height", h, 0.04);
        // pc_height_ = h;
        // nh.param("side_point_angle", angle, 0.34906585);
        nh.param<std::string>("bumper_left_frame", bumper_left_frame_, "bumper_left");
        nh.param<std::string>("bumper_right_frame", bumper_right_frame_, "bumper_right");
        
        // Lateral points x/y coordinates; we need to store float values to memcopy later
        p_side_x_ = +pc_radius_ * sin(pc_angle_); // angle degrees from vertical
        p_side_y_ = +pc_radius_ * cos(pc_angle_); // angle degrees from vertical
        n_side_y_ = -pc_radius_ * cos(pc_angle_); // angle degrees from vertical

        // Prepare constant parts of the pointcloud message to be  published
        bumper_left_pc_.header.frame_id = bumper_left_frame_;
        bumper_right_pc_.header.frame_id = bumper_right_frame_;
        bumper_left_pc_.width = bumper_right_pc_.width = 3;
        bumper_left_pc_.height = bumper_right_pc_.height = 1;
        bumper_left_pc_.fields.resize(3);
        bumper_right_pc_.fields.resize(3);

        // Set x/y/z as the only fields
        bumper_left_pc_.fields[0].name = bumper_right_pc_.fields[0].name = "x";
        bumper_left_pc_.fields[1].name = bumper_right_pc_.fields[1].name = "y";
        bumper_left_pc_.fields[2].name = bumper_right_pc_.fields[2].name = "z";

        int offset = 0;
        // All offsets are *4, as all field data types are float32
        for (size_t d = 0; d < bumper_left_pc_.fields.size(); ++d, offset += 4)
        {
            bumper_left_pc_.fields[d].count = bumper_right_pc_.fields[d].count = 1;
            bumper_left_pc_.fields[d].offset = bumper_right_pc_.fields[d].offset = offset;
            bumper_left_pc_.fields[d].datatype = bumper_right_pc_.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
        }

        bumper_left_pc_.point_step = bumper_right_pc_.point_step = offset;
        bumper_left_pc_.row_step = bumper_right_pc_.row_step = bumper_left_pc_.point_step * bumper_left_pc_.width;

        bumper_left_pc_.data.resize(3 * bumper_left_pc_.point_step);
        bumper_right_pc_.data.resize(3 * bumper_right_pc_.point_step);
        bumper_left_pc_.is_bigendian = bumper_right_pc_.is_bigendian = false;
        bumper_left_pc_.is_dense = bumper_right_pc_.is_dense = true;

        // Bumper "points" fix coordinates (the others depend on sensor activation/deactivation)

        // z: constant elevation from base frame
        memcpy(&bumper_left_pc_.data[0 * bumper_left_pc_.point_step + bumper_left_pc_.fields[2].offset], &pc_height_, sizeof(float));
        memcpy(&bumper_right_pc_.data[0 * bumper_right_pc_.point_step + bumper_right_pc_.fields[2].offset], &pc_height_, sizeof(float));

        bumper_pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/hovermower/sensors/Bumper_pointcloud", 10);
        base_controller_bumper_sub_ = nh.subscribe("/hovermower/sensors/Bumper", 10, &Bumper2PcNodelet::bc_bumperCB, this);

        ROS_INFO("Bumper pointcloud configured at distance %f and height %f from bumper frame", pc_radius_, pc_height_);
    }

    void Bumper2PcNodelet::dyn_callback(ros_hovermower_base_controller::HoverMowerBaseControllerConfig &config, uint32_t level)
    {

        pc_angle_ = config.bumper_pc_angle;
        pc_height_ = config.bumper_pc_height;
        pc_radius_ = config.bumper_pc_radius;
    }
} // namespace ros_hovermower_base_controller

PLUGINLIB_EXPORT_CLASS(ros_hovermower_base_controller::Bumper2PcNodelet, nodelet::Nodelet);