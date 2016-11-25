
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

#include <visualization_msgs/Marker.h>



laser2map::laser2map()
{

	laserSubscriber = nodeHandle.subscribe("scan", 1000, &laser2map::callbackLaser, this);
	mapSubscriber = nodeHandle.subscribe("map", 1000, &laser2map::callbackMap, this);
    posePublisher = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    we_have_map = false;

    #ifdef CREATE_MAP_CLOUD
    mapPublisher = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ> >("our_map", 1000);
    #endif

    #ifdef CREATE_LASER_CLOUD
    pointCloudPublisher = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ> >("our_points", 1000);
    #endif

    #ifdef CREATE_MATCH_CLOUD
    matchPublisher = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ> >("our_match", 1000);
    #endif

    artLaserPublisher = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ> >("artificial_laser", 1000);

    marker_pub = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 10);

}

void laser2map::callbackLaser(const sensor_msgs::LaserScan::ConstPtr& in)
{
    
	scan_in = *in;

    #ifdef CREATE_LASER_CLOUD
    laserCloud.points.clear();
    #endif

    sensor_data.clear();
    sensor_data_reduced.clear();

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

        sensor_data.push_back(p);

        if(i==0 || x < sensor_data_reduced[sensor_data_reduced.size()-1].x - map_resolution 
            || x > sensor_data_reduced[sensor_data_reduced.size()-1].x + map_resolution 
            || y < sensor_data_reduced[sensor_data_reduced.size()-1].y - map_resolution 
            || x > sensor_data_reduced[sensor_data_reduced.size()-1].x + map_resolution )
            sensor_data_reduced.push_back(p);

	}

    #ifdef CREATE_LASER_CLOUD
    laserCloud.header.frame_id="/map";
    #endif

}

void laser2map::callbackMap(nav_msgs::OccupancyGrid _map)
{
    map = _map;
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


//Calculate the match ratio between given points and map
//within a match tolerance
float laser2map::calculateMatch(vector<Point2>& source, float tolerance)
{

    #ifdef CREATE_MATCH_CLOUD
    matchCloud.points.clear();
    #endif

	int matched=0;

    int s = tolerance/map_resolution;



    for(int i=0; i<source.size(); i++)
	{

        int pixel_x =int( (source[i].x - map_position_x) /map_resolution );
        int pixel_y =int( (source[i].y - map_position_y) /map_resolution );

		int index = pixel_y*map_width + pixel_x;

        //tolerance for match acceptance
        //square frame
        
        bool ismatched = false;

        for(int i_x = -s; i_x <= s && !ismatched; i_x++)
        {
            for(int i_y = -s; i_y <= s && !ismatched; i_y++)
            {


                index = pixel_y*map_width + pixel_x + map_width*i_y + i_x;
                             
                if(index < map_data.size() && index >=0) 
                {

                    if(map_data[index]>0 && !ismatched) 
                    {
                        ismatched = true;

                    #ifdef CREATE_MATCH_CLOUD
                        matchCloud.points.push_back(pcl::PointXYZ(source[i].x, source[i].y, 0));
                    #endif 

                        matched++;


                    }
                }
            }
        }
        
	}

    #ifdef CREATE_MATCH_CLOUD
	matchCloud.header.frame_id="/map";
    #endif

	return matched / float(source.size());
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

void laser2map::artMatch()
{

    float min_x=-5;
    float min_y=-5;
    float  max_x=5;
    float  max_y=5;
    float x_increment = 0.2;
    float y_increment = 0.2;
    float angle_increment = 0.1;

    float min_err = 9999;

    float x, y, yaw;

    for (float drift_x = min_x; drift_x<max_x; drift_x+=x_increment)
    {
        for (float drift_y = min_y; drift_y < max_y; drift_y += y_increment)
        {
            for (float drift_angle = 0; drift_angle < 6.28; drift_angle += angle_increment)
            {
                float err = compareArtificialAndRealSensorData(drift_x, drift_y, drift_angle);
                if(err < min_err)
                {
                    min_err = err;
                    ROS_INFO("Match found: %f %f %f", drift_x, drift_y, drift_angle);
                    x = drift_x;
                    y = drift_y;
                    yaw = drift_angle;
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
    qua.setEuler(0, 0, yaw);

    tf::Transform map_to_base(qua, tf::Vector3(x, y, 0));
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


}

//Move the sensor data over the map
//Calculate the match ratio
//Return the list of possible poses whose match ratio is greater than threshold

vector<match2> laser2map::basitAlignment(float threshold)
{
    float min_x = 0 + map.info.origin.position.x;
    float min_y = 0 + map.info.origin.position.y;
    float  max_x = map.info.width * map.info.resolution + map.info.origin.position.x;
    float  max_y = map.info.width * map.info.resolution + map.info.origin.position.y;
    float x_increment = 0.2;
    float y_increment = 0.2;
    float angle_increment = 0.1;

    vector<Point2> s_data;

    vector<match2> matchedPoses;


    for (float drift_x = min_x; drift_x<max_x; drift_x+=x_increment)
    {
        for (float drift_y = min_y; drift_y < max_y; drift_y += y_increment)
        {
            int pixel_x =int( (drift_x - map_position_x) /map_resolution );
            int pixel_y =int( (drift_y - map_position_y) /map_resolution );

            int index = pixel_y*map_width + pixel_x;

            if (index >= 0 && index < map_data.size())
                if (map_data[index] != 0) continue;

            for (float drift_angle = 0; drift_angle < 6.28; drift_angle += angle_increment)
            {
                s_data.clear();

                for (int i=0; i<scan_in.ranges.size(); i++)
                {

                    float angle = scan_in.angle_min + i*(scan_in.angle_increment) + drift_angle;
                    float range = scan_in.ranges[i];

                    Point2 p;
                    p.x = cos(angle)*range + drift_x;
                    p.y = sin(angle)*range + drift_y;

                    if(i==0 || p.x > s_data[s_data.size()-1].x + map.info.resolution || p.y > s_data[s_data.size()-1].y + map.info.resolution
                        || p.x < s_data[s_data.size()-1].x - map.info.resolution || p.y < s_data[s_data.size()-1].y - map.info.resolution)

                        s_data.push_back(p);

                }

                
                //ROS_INFO("calculating: %f %f %f", drift_x, drift_y, drift_angle);
                float m = calculateMatch(s_data, 0.15);

                if (m > threshold)
                {

                    match2 mat;
                    mat.x=drift_x;
                    mat.y=drift_y;
                    mat.yaw=drift_angle;
                    mat.score=m;
                    matchedPoses.push_back(mat);

                    //ROS_INFO("match found %f %f %f %f", mat.score, mat.x, mat.y, mat.yaw);

                }
            }
        }
    }

/*
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

*/

    return matchedPoses;

}



void laser2map::publishPose(float x, float y, float yaw)
{
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
    qua.setEuler(0, 0, yaw);

    tf::Transform map_to_base(qua, tf::Vector3(x, y, 0));
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

vector<Point2> laser2map::artificialSensorData(float x, float y, float yaw)
{
   


    vector<Point2> ld;

    int i=0;

    for(float a = scan_in.angle_min; i<scan_in.ranges.size(); a += scan_in.angle_increment, i++)
    {
        Point2 p;
        for(float r = scan_in.range_min; r <= scan_in.range_max; r += map_resolution/2)
        {
            p.x = x + r*cos(yaw + a);
            p.y = y + r*sin(yaw + a);

            int pixel_x =int( (p.x - map_position_x) /map_resolution );
            int pixel_y =int( (p.y - map_position_y) /map_resolution );

            int index = pixel_y*map_width + pixel_x;

            if(0 <= index && index < map_data.size() )
                if(map_data[index] > 0) break;
        }

        ld.push_back(p);

    }

    
    artLaserCloud.clear();
    for(i = 0; i<ld.size(); i++)
    {
        pcl::PointXYZ pcl_p;
        pcl_p.x = ld[i].x;
        pcl_p.y = ld[i].y;
        pcl_p.z = 0;

        artLaserCloud.points.push_back(pcl_p);
    }
    artLaserCloud.header.frame_id = "/map";

    artLaserPublisher.publish(artLaserCloud);
    
    

    return ld;

}

float laser2map::compareArtificialAndRealSensorData(float x, float y, float yaw)
{
    vector<char> real_data(map_data.size(), 0);



laserCloud.clear();


    for(int i=0; i< scan_in.ranges.size(); i++ )
    {

        float angle = scan_in.angle_min + i*(scan_in.angle_increment) + yaw;
        float range = scan_in.ranges[i];

        float _x = cos(angle)*range + x;
        float _y = sin(angle)*range + y;

        int pixel_x =int( (_x - map_position_x) /map_resolution );
        int pixel_y =int( (_y - map_position_y) /map_resolution );

        int index = pixel_y*map_width + pixel_x;

        if(index >= 0 && index < real_data.size() ) real_data[index] = 100;



        #ifdef CREATE_LASER_CLOUD
        
        laserCloud.points.push_back(pcl::PointXYZ(_x,_y,0));
        
        #endif


        

    }
    pub();

    
    vector<Point2> art = artificialSensorData(x, y, yaw);

    int matched = 0;

    for (int i=0; i < art.size(); i++)
    {
        int pixel_x =int( (art[i].x - map_position_x) /map_resolution );
        int pixel_y =int( (art[i].y - map_position_y) /map_resolution );

        int s = 2;

        bool ismatched = false;

        for(int i_x = -s; i_x <= s; i_x++)
        {
            for(int i_y = -s; i_y <= s; i_y++)
            {


                int index = pixel_y*map_width + pixel_x + map_width*i_y + i_x;
                             
                if(index >= real_data.size() || index <0 || ismatched) continue;


                if(real_data[index]>0 ) {
                    ismatched = true;

                    matched++;

                }
            }
        }

    }


/*

    float sum_err_sq = 0;

    if(art.size() != real_data.size())
        ROS_ERROR("artificial and real sensor datas does not match: %lu %lu", art.size(), real_data.size());


    int nearest_index[real_data.size()];

    for(int i = 0; i < real_data.size(); i++)
    {
        float nearest = 100;
        int ni = 0;
        for(int k = 0; k < art.size(); k++)
        {
            float e = pow(real_data[i].x - art[k].x, 2) + pow(real_data[i].y - art[k].y, 2);
            if (e < nearest) 
                {
                    nearest = e;
                    ni = k;
                }
        }
        sum_err_sq += nearest;

        nearest_index[i] = ni;
        
    }




    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/map";
    line_list.id = 150;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.color.r = 1;
    line_list.color.a = 1;
    line_list.header.stamp = ros::Time::now();
    line_list.pose.orientation.w = 1.0;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.ns = "points_and_lines";


    for(int i=0; i<real_data.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = real_data[i].x;
        p.y = real_data[i].y;
        p.z = 0;
        line_list.points.push_back(p);
        p.x = real_data[nearest_index[i]].x;
        p.y = real_data[nearest_index[i]].y;
        line_list.points.push_back(p);
    }

    marker_pub.publish(line_list);
    */

    return matched/float(art.size());

}