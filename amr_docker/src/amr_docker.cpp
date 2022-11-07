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

    void AMRDocker::loadParams() {}

    void AMRDocker::targetPoseInputCallBack(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
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
        double twist_linear_x_gain = delta.delta_x - tolerance_x;
        double twist_angular_z_gain =
        (delta.delta_yaw - tolerance_yaw) + (delta.delta_y - tolerance_x);

        if (abs(twist_linear_x_gain) >= abs(tolerance_x) ||
            abs(twist_angular_z_gain) >= abs(tolerance_yaw)) {
            vel_msg.linear.x = (twist_linear_x_gain)*0.3;
            vel_msg.angular.z = (twist_angular_z_gain)*0.6;
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
        return true;
    }
} // namespace amr_docker