
#include "laser2map/laser2map.h"

#include <cstdio>
#include <cmath>



#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>



#define CREATE_LASER_CLOUD
#define CREATE_MAP_CLOUD
#define CREATE_MATCH_CLOUD


	//void laser2map::getPointCloudFromLaser(const sensor_msgs::LaserScan::ConstPtr&);

laser2map::laser2map()
{
	mapPublisher = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ> >("our_map", 1000);
	matchPublisher = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ> >("our_match", 1000);
	laserSubscriber = nodeHandle.subscribe("scan", 1000, &laser2map::getPointCloudFromLaser, this);
	pointCloudPublisher = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ> >("our_points", 1000);
	mapSubscriber = nodeHandle.subscribe("map", 1000, &laser2map::getMapTopic, this);
    posePublisher = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    we_have_map = false;

}

void laser2map::getPointCloudFromLaser(const sensor_msgs::LaserScan::ConstPtr& in)
{
	laser_geometry::LaserProjection projector;

	sensor_msgs::LaserScan scan_in = *in;

    #ifdef CREATE_LASER_CLOUD
    laserCloud.points.clear();
    #endif

    sensor_data.clear();

    tf::StampedTransform laserTransform;

    try
    {
        transformListener.waitForTransform("/map", "/base_laser_link", ros::Time(0), ros::Duration(10.0));
        transformListener.lookupTransform("/map", "/base_laser_link", ros::Time(0), laserTransform);

    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
        

    double r, p;
    tf::Matrix3x3(laserTransform.getRotation()).getRPY(r,p,laser_yaw);
    laser_x = laserTransform.getOrigin().x();
    laser_y = laserTransform.getOrigin().y();



	for(int i=0; i< scan_in.ranges.size(); i++ )
	{

        float angle = scan_in.angle_min + i*(scan_in.angle_increment) + laser_yaw;
        float range = scan_in.ranges[i];


        float x = cos(angle)*range + laser_x;
        float y = sin(angle)*range + laser_y;

        #ifdef CREATE_LASER_CLOUD
        laserCloud.points.push_back(pcl::PointXYZ(x,y,0));
        #endif

        Point2 p;
        p.x=x;
        p.y=y;

        if(i==0 || x < sensor_data[sensor_data.size()-1].x - map_resolution 
            || x > sensor_data[sensor_data.size()-1].x + map_resolution 
            || y < sensor_data[sensor_data.size()-1].y - map_resolution 
            || x > sensor_data[sensor_data.size()-1].x + map_resolution )
            sensor_data.push_back(p);

	}

    #ifdef CREATE_LASER_CLOUD
    laserCloud.header.frame_id="/map";
    #endif

}

void laser2map::getMapTopic(nav_msgs::OccupancyGrid map)
{
	map_width = map.info.width;
	map_height = map.info.height;
	map_resolution = map.info.resolution;
    map_position_x = map.info.origin.position.x;
    map_position_y = map.info.origin.position.y;

    #ifdef CREATE_MAP_CLOUD
    mapPointCloud.points.clear();
    #endif


	for(int i=0; i<map_width*map_height; i++)
	
	{


		float x = (i%map_width)*map.info.resolution + map.info.origin.position.x;
        float y = (i/map_width)*map.info.resolution + map.info.origin.position.y;

			//ROS_INFO("%f   %f  %d",x,y,map.data[i]);		

        #ifdef CREATE_MAP_CLOUD

		if(map.data[i]>0)
		{
            
            mapPointCloud.points.push_back(pcl::PointXYZ( x, y, 0 ));


		}
		mapPointCloud.header.frame_id="/map";

        #endif 


		map_data.push_back(map.data[i]);


	}

    we_have_map = true;

	ROS_INFO("map get");


}

float laser2map::calculateMatch()
{
    if(!we_have_map) return 0;

    #ifdef CREATE_MATCH_CLOUD
    matchCloud.points.clear();
    #endif

	int matched=0;



    for(int i=0; i<sensor_data.size(); i++)
	{

        int pixel_x =int( (sensor_data[i].x - map_position_x) /map_resolution );
        int pixel_y =int( (sensor_data[i].y - map_position_y) /map_resolution );

		int index = pixel_y*map_width + pixel_x;

        int s = 3;

        bool ismatched = false;

        for(int i_x = -s; i_x <= s; i_x++)
        {
            for(int i_y = -s; i_y <= s; i_y++)
            {


                index = pixel_y*map_width + pixel_x + map_width*i_y + i_x;
                             
                if(index >= map_data.size() || index <0) continue;


                if(map_data[index]>0 && !ismatched) {
                    ismatched = true;

                    #ifdef CREATE_MATCH_CLOUD
                    matchCloud.points.push_back(pcl::PointXYZ(sensor_data[i].x, sensor_data[i].y, 0));
                    #endif CREATE_MATCH_CLOUD

                    matched++;


                }
            }
        }

	}

    #ifdef CREATE_MATCH_CLOUD
	matchCloud.header.frame_id="/map";
    #endif

	return matched / float(sensor_data.size());
}

void laser2map::pub()
{
	
    #ifdef CREATE_LASER_CLOUD
	pointCloudPublisher.publish(laserCloud);
    #endif

    #ifdef CREATE_MAP_CLOUD
	mapPublisher.publish(mapPointCloud);
    #endif

    #ifdef CREATE_MATCH_CLOUD
    matchPublisher.publish(matchCloud);
    #endif


}

void laser2map::basitAlignment()
{
    float min_x=-5;
    float min_y=-5;
    float  max_x=5;
    float  max_y=5;
    float x_increment = 0.2;
    float y_increment = 0.2;
    float angle_increment = 0.1;

    vector<Point2> _s_back = sensor_data;

    float max_match = 0;

    float _x, _y, _yaw;



    for (float drift_x = min_x; drift_x<max_x; drift_x+=x_increment)
    {
        for (float drift_y = min_y; drift_y < max_y; drift_y += y_increment)
        {
            for (float drift_angle = 0; drift_angle < 6.28; drift_angle += angle_increment)
            {
                for (int i=0; i<sensor_data.size(); i++)
                {
                    float local_x = _s_back[i].x - laser_x;
                    float local_y = _s_back[i].y - laser_y;

                    sensor_data[i].x = local_x*cos(drift_angle) - local_y*sin(drift_angle) + laser_x + drift_x;
                    sensor_data[i].y = local_x*sin(drift_angle) + local_y*cos(drift_angle) + laser_y + drift_y;
                    

                }

                float m = calculateMatch();
                

                if (m > max_match)
                {
                    max_match = m;

                    #ifdef CREATE_LASER_CLOUD
                    for (int i=0; i<sensor_data.size(); i++)
                    {

                        laserCloud.points[i].x = sensor_data[i].x; 

                        laserCloud.points[i].y = sensor_data[i].y;

                    }
                    pub();
                    #endif

                    _x = drift_x + laser_x;
                    _y = drift_y + laser_y;
                    _yaw = drift_angle + laser_yaw;

                    ROS_INFO("%f match found: %f %f %f",m, drift_x+laser_x, drift_y+laser_y, drift_angle+laser_yaw);

                }
            }
        }
    }

    tf::StampedTransform laser_to_base;
    try
    {
        transformListener.waitForTransform("/base_laser_link", "/base_footprint", ros::Time(0), ros::Duration(10.0));
        transformListener.lookupTransform("/base_laser_link", "/base_footprint", ros::Time(0), laser_to_base);

    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    tf::Quaternion qua;
    qua.setEuler(0, 0, _yaw);

    tf::Transform map_to_base(qua, tf::Vector3(_x, _y, 0));
    map_to_base = map_to_base * laser_to_base ;

    geometry_msgs::PoseWithCovarianceStamped pose;

    pose.pose.pose.position.x = map_to_base.getOrigin().x();
    pose.pose.pose.position.y = map_to_base.getOrigin().y();

    pose.pose.pose.orientation.x = map_to_base.getRotation().getX();
    pose.pose.pose.orientation.y = map_to_base.getRotation().getY();
    pose.pose.pose.orientation.z = map_to_base.getRotation().getZ();
    pose.pose.pose.orientation.w = map_to_base.getRotation().getW();


    pose.pose.covariance[0] = 0.4;
    pose.pose.covariance[7] = 0.4;
    pose.pose.covariance[35] = 0.1;

    posePublisher.publish(pose);


    sensor_data = _s_back;



}

void laser2map::applyICPTransform()
{

#ifdef CREATE_LASER_CLOUD
#ifdef CREATE_MAP_CLOUD

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(laserCloud.makeShared());
    icp.setInputTarget(mapPointCloud.makeShared());
    icp.setMaximumIterations(5000);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);


    ROS_INFO("max. iterations: %d", icp.getMaximumIterations());



    Eigen::Matrix4f transf = icp.getFinalTransformation();
    tf::Matrix3x3 tf_rotation( transf(0,0), transf(0,1), transf(0,2), transf(1,0), transf(1,1), transf(1,2),transf(2,0), transf(2,1), transf(2,2));
    tf::Vector3 tf_translation( transf(0,3), transf(1,3), transf(2,3));
    tf::Transform tf_transform( tf_rotation, tf_translation);

    if(icp.hasConverged())
    {
        ROS_INFO("sending..");
        tf::StampedTransform laserTransform;

        try
        {
            transformListener.waitForTransform("/map", "/base_laser_link", ros::Time(0), ros::Duration(10.0));
            transformListener.lookupTransform("/map", "/base_laser_link", ros::Time(0), laserTransform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());

        }
        tf_transform = tf_transform * laserTransform;
        geometry_msgs::PoseWithCovarianceStamped pose;
        pose.header.frame_id = "map";
        pose.pose.pose.position.x = tf_transform.getOrigin().x();
        pose.pose.pose.position.y = tf_transform.getOrigin().y();
        pose.pose.pose.orientation.x = tf_transform.getRotation().getX();
        pose.pose.pose.orientation.y = tf_transform.getRotation().getY();
        pose.pose.pose.orientation.z = tf_transform.getRotation().getZ();
        pose.pose.pose.orientation.w = tf_transform.getRotation().getW();
        pose.pose.covariance[0] = 0.4;
        pose.pose.covariance[7] = 0.4;
        pose.pose.covariance[35] = 0.1;

        ROS_INFO("%f %f %f %f %f %f %f ", 
            pose.pose.pose.position.x,
            pose.pose.pose.position.y,
            pose.pose.pose.position.z,
            pose.pose.pose.orientation.x,
            pose.pose.pose.orientation.y,
            pose.pose.pose.orientation.z,
            pose.pose.pose.orientation.w);

        pose.header.frame_id = "map";

        posePublisher.publish(pose);

        ROS_INFO("sent");

    }

#endif
#endif
}



