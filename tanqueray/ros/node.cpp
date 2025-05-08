/*
*
*
*/

#include "tanqueray/ros/factor_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tanqueray_node");
    ros::NodeHandle nh;

    try
    {
        FactorNode node(nh);
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
    }
    return 0;
}
