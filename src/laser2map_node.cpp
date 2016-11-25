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

	vector<match2> matches;

	float threshold = 0.9;

	while(ros::ok())
	{
		s=ros::Time::now()-t;
		
		//l.compareArtificialAndRealSensorData (-4.69, 10, -0.75);

	

			if(s.toSec()>1)
			{

				matches = l.basitAlignment(threshold);

				int best = 0;
				float best_value = 0;

				for (int i = 0; i<matches.size(); i++)
				{
					float val = l.compareArtificialAndRealSensorData (matches[i].x, matches[i].y, matches[i].yaw);

					ROS_INFO("match: %f %f %f", matches[i].x, matches[i].y, matches[i].yaw);

					if(val > best_value)
					{
						best_value = val;
						best = i;
					} 
						
				}

				if(matches.size() > 0)
				{
					if(matches.size() > 150) threshold += 0.04;

					l.publishPose(matches[best].x, matches[best].y, matches[best].yaw);
					ROS_INFO("pose published: %f %f %f", matches[best].x, matches[best].y, matches[best].yaw);

					

					ROS_INFO("%f %f", matches[best].score, l.compareArtificialAndRealSensorData (matches[best].x, matches[best].y, matches[best].yaw));



				}
				else 
					{
						ROS_INFO("%.2f no match found aybalam", threshold);
						threshold -= 0.02;
					}

			
				t=ros::Time::now();
												
			}
		
		

		l.pub();


		r.sleep();

		ros::spinOnce();


	}
	


}
