#ifndef _HOVERMOWER_BUMPER2PC_H
#define _HOVERMOWER_BUMPER2PC_H

/*
 * Copyright (c) 2013, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /include/kobuki_bumper2pc/kobuki_bumper2pc.hpp
 *
 * @brief Bumper/cliff to pointcloud nodelet class declaration.
 *
 * Publish bumpers and cliff sensors events as points in a pointcloud, so navistack can use them
 * for poor-man navigation. Implemented as a nodelet intended to run together with kobuki_node.
 *
 * @author Jorge Santos, Yujin Robot
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <nodelet/nodelet.hpp>

#include <sensor_msgs/msg/PointCloud2.hpp>
#include <dynamic_reconfigure/server.hpp>
#include <ros_hovermower_base_controller/HoverMowerBaseControllerConfig.hpp>
#include "rosmower_msgs/msg/Bumper.hpp"
#include <tf2_ros/transform_listener.hpp>
#include <geometry_msgs/msg/TransformStamped.hpp>

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace ros_hovermower_base_controller
{

    /**
     * @brief Bumper2PcNodelet class declaration
     */
    class Bumper2PcNodelet : public nodelet::Nodelet
    {
    public:
        Bumper2PcNodelet()
            : P_INF_X(+100 * sin(0.34906585)),
              P_INF_Y(+100 * cos(0.34906585)),
              N_INF_Y(-100 * cos(0.34906585)),
              ZERO(0), prev_bumper_left_(false), prev_bumper_right_(false) {}
        ~Bumper2PcNodelet() {}

        void onInit();
        void dyn_callback(ros_hovermower_base_controller::HoverMowerBaseControllerConfig &config, uint32_t level);

    private:
        const float P_INF_X; // somewhere out of reach from the robot (positive x)
        const float P_INF_Y; // somewhere out of reach from the robot (positive y)
        const float N_INF_Y; // somewhere out of reach from the robot (negative y)
        const float ZERO;

        bool prev_bumper_left_;
        bool prev_bumper_right_;

        // additional bumper attributes
        std::string bumper_left_frame_;
        std::string bumper_right_frame_;
        std::string base_frame_;

        float pc_radius_;
        float pc_height_;
        float pc_angle_;
        float p_side_x_;
        float p_side_y_;
        float n_side_y_;
        float distance_x_;
        float distance_y_;

        // tf transformation between base_frame and bumper frames
        float bumper_frame_left_x = 0.0;
        float bumper_frame_left_y = 0.0;
        float bumper_frame_right_x = 0.0;
        float bumper_frame_right_y = 0.0;
        tf2_ros::Buffer tfBuffer;   

        rclcpp::Publisher bumper_pc_pub_;
        rclcpp::Subscriber base_controller_bumper_sub_;

        sensor_msgs::msg::PointCloud2 bumper_pc_;

        /**
         * @brief Core sensors state structure callback
         * @param msg incoming topic message
         */
        void bc_bumperCB(const rosmower_msgs::msg::Bumper::ConstPtr &msg);
        void get_tf_bumper();

        // dynamic reconfigure
        typedef dynamic_reconfigure::Server<ros_hovermower_base_controller::HoverMowerBaseControllerConfig> DynamicReconfigServer;
        boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
        DynamicReconfigServer::CallbackType param_reconfig_callback_;
    };

} // namespace ros_hovermower_base_controller

#endif