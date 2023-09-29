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

#include <pluginlib/class_list_macros.hpp>

#include "hovermower_bumper2pc.hpp"

namespace ros_hovermower_base_controller
{

    void Bumper2PcNodelet::bc_bumperCB(const rosmower_msgs::msg::Bumper::ConstPtr &msg)
    {
        if (bumper_pc_pub_.getNumSubscribers() == 0)
            return;

        // We publish just one "no events" pc (with all three points far away) and stop spamming when bumper/cliff conditions disappear
        if (!msg->left && !msg->right && !prev_bumper_left_ && !prev_bumper_right_)
            return;

        prev_bumper_left_ = msg->left;
        prev_bumper_right_ = msg->right;

        // For any of {left/right} with no bumper event, we publish a faraway point that won't get used
        if (msg->left)
        {
            distance_x_ = p_side_x_ + bumper_frame_left_x;
            distance_y_ = bumper_frame_left_y + p_side_y_;
            memcpy(&bumper_pc_.data[0 * bumper_pc_.point_step + bumper_pc_.fields[0].offset], &distance_x_, sizeof(float));
            memcpy(&bumper_pc_.data[0 * bumper_pc_.point_step + bumper_pc_.fields[1].offset], &distance_y_, sizeof(float));
        }
        else
        {
            memcpy(&bumper_pc_.data[0 * bumper_pc_.point_step + bumper_pc_.fields[0].offset], &P_INF_X, sizeof(float));
            memcpy(&bumper_pc_.data[0 * bumper_pc_.point_step + bumper_pc_.fields[1].offset], &P_INF_Y, sizeof(float));
        }

        if (msg->right)
        {
            distance_x_ = p_side_x_ + bumper_frame_right_x;
            distance_y_ = bumper_frame_right_y - p_side_y_ ;

            memcpy(&bumper_pc_.data[1 * bumper_pc_.point_step + bumper_pc_.fields[0].offset], &distance_x_, sizeof(float));
            memcpy(&bumper_pc_.data[1 * bumper_pc_.point_step + bumper_pc_.fields[1].offset], &distance_y_, sizeof(float));
        }
        else
        {
            memcpy(&bumper_pc_.data[1 * bumper_pc_.point_step + bumper_pc_.fields[0].offset], &P_INF_X, sizeof(float));
            memcpy(&bumper_pc_.data[1 * bumper_pc_.point_step + bumper_pc_.fields[1].offset], &P_INF_Y, sizeof(float));
        }

        bumper_pc_.header.stamp = rclcpp::Time::now();

        bumper_pc_pub_.publish(bumper_pc_);
    }

    void Bumper2PcNodelet::onInit()
    {
        rclcpp::Node nh = this->getPrivateNodeHandle();

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
        nh.param<std::string>("base_frame", base_frame_, "base_link");

        // Lateral points x/y coordinates; we need to store float values to memcopy later
        p_side_x_ = +pc_radius_ * sin(pc_angle_); // angle degrees from vertical
        p_side_y_ = +pc_radius_ * cos(pc_angle_); // angle degrees from vertical
        n_side_y_ = -pc_radius_ * cos(pc_angle_); // angle degrees from vertical

        // Prepare constant parts of the pointcloud message to be  published
        bumper_pc_.header.frame_id = base_frame_;

        bumper_pc_.width = 2;
        bumper_pc_.height = 1;
        bumper_pc_.fields.resize(3);

        // Set x/y/z as the only fields
        bumper_pc_.fields[0].name = "x";
        bumper_pc_.fields[1].name = "y";
        bumper_pc_.fields[2].name = "z";

        int offset = 0;
        // All offsets are *4, as all field data types are float32
        for (size_t d = 0; d < bumper_pc_.fields.size(); ++d, offset += 4)
        {
            bumper_pc_.fields[d].count = 1;
            bumper_pc_.fields[d].offset = offset;
            bumper_pc_.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
        }

        bumper_pc_.point_step = offset;
        bumper_pc_.row_step = bumper_pc_.point_step * bumper_pc_.width;

        bumper_pc_.data.resize(2 * bumper_pc_.point_step);
        bumper_pc_.is_bigendian = false;
        bumper_pc_.is_dense = true;

        // Bumper "points" fix coordinates (the others depend on sensor activation/deactivation)
        // z: constant elevation from base frame
        memcpy(&bumper_pc_.data[0 * bumper_pc_.point_step + bumper_pc_.fields[2].offset], &pc_height_, sizeof(float));
        memcpy(&bumper_pc_.data[1 * bumper_pc_.point_step + bumper_pc_.fields[2].offset], &pc_height_, sizeof(float));

        bumper_pc_pub_ = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/hovermower/sensors/Bumper_pointcloud", 10);
        base_controller_bumper_sub_ = nnode->subscriber("/hovermower/sensors/Bumper", 10, &Bumper2PcNodelet::bc_bumperCB, this);

        // get tf between base_frame and bummper_frames
        get_tf_bumper();
        RCLCPP_INFO(node->get_logger(),"Bumper pointcloud configured at distance %f and height %f from bumper frame", pc_radius_, pc_height_);
    }

    // Get transformation between base_link and bumper frames to calculate point location
    void Bumper2PcNodelet::get_tf_bumper()
    {
        
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform(base_frame_, bumper_left_frame_,
                                                        rclcpp::Time(0), rclcpp::Duration(3.0));
            bumper_frame_left_x = transformStamped.transform.translation.x;
            bumper_frame_left_y = transformStamped.transform.translation.y;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node->get_logger(),"%s", ex.what());
        }

        try
        {
            transformStamped = tfBuffer.lookupTransform(base_frame_, bumper_right_frame_,
                                                        rclcpp::Time(0), rclcpp::Duration(3.0));
            bumper_frame_right_x = transformStamped.transform.translation.x;
            bumper_frame_right_y = transformStamped.transform.translation.y;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node->get_logger(),"%s", ex.what());
        } 
       
    }

    void Bumper2PcNodelet::dyn_callback(ros_hovermower_base_controller::HoverMowerBaseControllerConfig &config, uint32_t level)
    {

        pc_angle_ = config.bumper_pc_angle;
        pc_height_ = config.bumper_pc_height;
        pc_radius_ = config.bumper_pc_radius;
    }
} // namespace ros_hovermower_base_controller

PLUGINLIB_EXPORT_CLASS(ros_hovermower_base_controller::Bumper2PcNodelet, nodelet::Nodelet);