#include "target_localization/target_localization.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_localization_node");
    ros::NodeHandle nh("~");
    target_localization::TargetLocalization targetLocalizationNode(nh);
    targetLocalizationNode.start();

    ros::spin();
    return 0;
}