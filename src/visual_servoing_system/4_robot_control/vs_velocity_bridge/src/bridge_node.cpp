#include "vs_velocity_bridge/velocity_bridge.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vs_velocity_bridge");
    
    vs_velocity_bridge::VelocityBridge bridge;
    bridge.init();
    
    ros::spin();
    return 0;
}