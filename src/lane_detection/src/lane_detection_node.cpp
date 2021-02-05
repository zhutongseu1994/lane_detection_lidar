#include "lane_detection_manager.h"

using namespace skywell;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_detection_node");
    ros::NodeHandle nh;
    skywell::Param param;

    param.loadcfg();

    skywell::LaneManager lane_manager(nh, &param);

    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ros::spin();
}