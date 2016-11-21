#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/registration/icp.h"

#include "ros/ros.h"

#define CREATE_LASER_CLOUD
#define CREATE_MAP_CLOUD
#define CREATE_MATCH_CLOUD

using namespace std;

struct Point2
{
    float x;
    float y;

};

class laser2map
{
public:
    ros::NodeHandle nodeHandle;
    ros::Publisher mapPublisher;
    ros::Subscriber laserSubscriber;
    ros::Subscriber mapSubscriber;
    ros::Publisher pointCloudPublisher;
    ros::Publisher matchPublisher;
    ros::Publisher posePublisher;
    tf::TransformListener transformListener;


    nav_msgs::OccupancyGrid our_map;

    #ifdef CREATE_LASER_CLOUD
    pcl::PointCloud<pcl::PointXYZ> laserCloud;
    #endif

    #ifdef CREATE_MAP_CLOUD
    pcl::PointCloud<pcl::PointXYZ> mapPointCloud;
    #endif

    #ifdef CREATE_MATCH_CLOUD
    pcl::PointCloud<pcl::PointXYZ> matchCloud;
    #endif

    vector<char> map_data;
    int map_width, map_height;
    float map_resolution, map_position_x, map_position_y;
    double laser_x, laser_y, laser_yaw;

    bool we_have_map;

    vector<Point2> sensor_data;

    laser2map();

    void getPointCloudFromLaser(const sensor_msgs::LaserScan::ConstPtr& in);
    void getMapTopic(nav_msgs::OccupancyGrid map);

    void pub();
    float calculateMatch();

    void applyICPTransform();

    bool robustAlignment();

    void basitAlignment();



};
