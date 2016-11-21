#include "ros/ros.h"
#include "laser2map/laser2map.h"
#include <cstdio>

#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser2map_node");

	ros::NodeHandle n;

	ros::Publisher movePublisher = n.advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel", 100);
	geometry_msgs::Twist vel;

	vel.linear.x=0;
	vel.linear.y=0;
	vel.linear.z=0;

	vel.angular.x=0;
	vel.angular.y=0;
	vel.angular.z=0;

	ros::Rate r(2);

	laser2map l;

	ros::Time t = ros::Time::now();
	ros::Duration s;

	while(ros::ok())
	{
		
		//l.pointCloudPublisher.publish(l.laserCloud);
		if(l.calculateMatch()<0.85){
			s=ros::Time::now()-t;

			if(s.toSec()>4)
			{

				l.basitAlignment();

				t=ros::Time::now();
												
			}
		}
		l.pub();


		r.sleep();

		ros::spinOnce();


	}
	


}
