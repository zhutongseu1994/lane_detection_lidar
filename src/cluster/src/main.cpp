#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <boost/thread.hpp>
#include <cluster/Heart.h>

#include "param.h"
#include "manager.h"

int main(int argc,char **argv)
{
	ros::init(argc,argv,"cluster_node");
	ros::NodeHandle nh("~");
	ros::NodeHandle node;
	skywell::Param param; 
	param.loadcfg();
	//param.loadparam(node,nh);
	skywell::Manager manager(node,&param);
	ros::spin();


	/*ros::Publisher heart_pub = nh.advertise<cluster::Heart>("heart", 1000);
	ros::Rate loop_rate(10);
	cluster::Heart heart;
	//heart.process_id = getpid();
	heart.process_id = 1002;
	heart.process_name = "cluster_node";
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
	}*/
}