#ifndef AMR_DOCKING_AMR_DOCKER_HPP_
#define AMR_DOCKING_AMR_DOCKER_HPP_

#include "tf/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <std_srvs/Trigger.h>
#include <chrono>
#include <thread>

namespace amr_docker
{
    struct Delta {
        double delta_x{0.0};
        double delta_y{0.0};
        double delta_yaw{0.0};

    };
    class AMRDocker {
    public:
        AMRDocker(ros::NodeHandle& nh);
        void start();

    private:
        ros::NodeHandle nh_p_;
        ros::NodeHandle nh_;

        ros::Subscriber target_pose_input_sub_;
        ros::Publisher twist_output_pub_;
        ros::Timer pub_timer_;

        ros::ServiceServer dock_service_;
        ros::ServiceServer undock_service_;

        geometry_msgs::TransformStamped tfstamped_base_target_;
        geometry_msgs::Twist dock_cmd_vel_;
        std::string target_frame_{"base_link"};

        bool start_dock_{false};

        double dock_linear_gain_{0.3};
        double dock_angular_gain_{2.0};

        // geometry_msgs::PoseStamped dock_pose_;

        void
        loadParams();
        void targetPoseInputCallBack(
        const geometry_msgs::PoseStamped::ConstPtr& msg);
        void timerPubCallBack(const ros::TimerEvent& e);
        Delta blindDock(const geometry_msgs::PoseStamped::ConstPtr& msg);
        bool blindUndock(const geometry_msgs::Twist& command, const int period_in_ms);
        void deltaToTwist(const Delta& delta);
        bool startDock(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool startUnDock(std_srvs::Trigger::Request& req,
                         std_srvs::Trigger::Response& res);
    };
}

#endif