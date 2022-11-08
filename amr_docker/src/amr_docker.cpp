#include "amr_docker/amr_docker.hpp"

namespace amr_docker {
    AMRDocker::AMRDocker(ros::NodeHandle& nh)
        : nh_p_(nh)
        , target_pose_input_sub_(nh_.subscribe<geometry_msgs::PoseStamped>(
          "target_pose_input", 10, &AMRDocker::targetPoseInputCallBack, this))
        , twist_output_pub_(
          nh_p_.advertise<geometry_msgs::Twist>("twist_output", 10))
        , dock_service_(
          nh_p_.advertiseService("start_dock", &AMRDocker::startDock, this))
        , undock_service_(
          nh_p_.advertiseService("start_undock", &AMRDocker::startUnDock, this))
    {
    }

    void AMRDocker::start()
    {
        loadParams();
        while (ros::ok()) {
            ros::Rate(ros::Duration(0.05)).sleep();
            ros::spinOnce();
        }
    }

    void AMRDocker::loadParams()
    {
        if (!nh_p_.param("dock_linear_gain", dock_linear_gain_,
                         dock_linear_gain_)) {
            ROS_WARN_STREAM("dock_linear_gain is not set! Use default "
                            << dock_linear_gain_);
        }
        if (!nh_p_.param("dock_angular_gain", dock_angular_gain_,
                         dock_angular_gain_)) {
            ROS_WARN_STREAM("dock_angular_gain is not set! Use default "
                            << dock_angular_gain_);
        }
    }

    void AMRDocker::targetPoseInputCallBack(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // dock_pose_ = *msg;
        if (start_dock_) {
            deltaToTwist(blindDock(msg));
        }
    }

    void AMRDocker::timerPubCallBack(const ros::TimerEvent& e) {}

    Delta AMRDocker::blindDock(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        Delta delta;
        delta.delta_x = msg->pose.position.x;
        delta.delta_y = msg->pose.position.y;

        double roll, pitch, yaw;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.orientation, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        delta.delta_yaw = yaw;

        return delta;
    }

    void AMRDocker::deltaToTwist(const Delta& delta)
    {
        geometry_msgs::Twist vel_msg;
        const double tolerance_x = 0.05;
        const double tolerance_yaw = 0.05;

        ROS_INFO_STREAM("delta x: " << delta.delta_x);
        ROS_INFO_STREAM("delta y: " << delta.delta_y);
        ROS_INFO_STREAM("delta yaw: " << delta.delta_yaw);
        double twist_linear_x_gain = delta.delta_x;
        double twist_angular_z_gain =
        (abs(delta.delta_y) > abs(delta.delta_yaw)) ? delta.delta_y : delta.delta_yaw;

        if (abs(twist_linear_x_gain) >= tolerance_x ||
            abs(twist_angular_z_gain) >= tolerance_yaw) {
            vel_msg.linear.x = (twist_linear_x_gain)*dock_linear_gain_;
            vel_msg.angular.z = (twist_angular_z_gain)*dock_angular_gain_;
        }
        else {
            start_dock_ = false;
        }

        twist_output_pub_.publish(vel_msg);
    }

    bool AMRDocker::startDock(std_srvs::Trigger::Request& req,
                              std_srvs::Trigger::Response& res)
    {
        start_dock_ = start_dock_ ? false : true;

        std::string start_dock_str = start_dock_ ? "True" : "False";
        res.success = true;
        res.message.append("Dock? ").append(start_dock_str);
        return res.success;
    }

    bool AMRDocker::startUnDock(std_srvs::Trigger::Request& req,
                                std_srvs::Trigger::Response& res)
    {
        geometry_msgs::Twist undock_twist;
        undock_twist.linear.x = -0.5;
        undock_twist.angular.z = 0.0;
        const int undock_period_in_ms = 3000;
        res.success = blindUndock(undock_twist, undock_period_in_ms);
        res.message.append("UnDocked");
        return res.success;
    }

    bool AMRDocker::blindUndock(const geometry_msgs::Twist& command,
                                const int period_in_ms)
    {
        start_dock_ = false;

        auto start_time = std::chrono::steady_clock::now();
        auto end_time = std::chrono::steady_clock::now();

        while (std::chrono::duration_cast<std::chrono::milliseconds>(
               end_time - start_time) <=
               std::chrono::milliseconds(period_in_ms)) {
            twist_output_pub_.publish(command);
            end_time = std::chrono::steady_clock::now();
        }
        geometry_msgs::Twist zero_command;
        twist_output_pub_.publish(zero_command);
        return true;
    }
} // namespace amr_docker