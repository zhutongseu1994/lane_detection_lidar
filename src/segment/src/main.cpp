#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <boost/thread.hpp>
#include <ros/console.h>
#include <segment/Heart.h>

#include "manager.h"
#include "config.h"



int main(int argc,char **argv)
{
	ros::init(argc,argv,"segment_node");
	ros::NodeHandle nh("~");
	ros::NodeHandle node;
	skywell::Param param; 
	//param.loadparam(node,nh);
	param.loadcfg();
	skywell::Manager manager(node,&param);
	ros::spin();


	/*
	ros::Publisher heart_pub = nh.advertise<segment::Heart>("heart", 1000);
	ros::Rate loop_rate(10);
	segment::Heart heart;
	//heart.process_id = getpid();
	heart.process_id = 1001;
	heart.process_name = "segment_node";
	int count = 0;
	while (ros::ok)
	{
		count++;
		if ((count%10) == 0)
		{
			//heart_pub.publish(heart);
			count = 0;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	*/
}