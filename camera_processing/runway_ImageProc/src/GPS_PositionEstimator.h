//
// Created by silverback on 11/12/20.
//

#ifndef RUNWAY_IMAGEPROC_POSITIONESTIMATOR_H
#define RUNWAY_IMAGEPROC_POSITIONESTIMATOR_H
#include <ros/ros.h>
#include <runway_ImageProc/MetersCoordinates.h>
#include <runway_ImageProc/MetersPointsArrays.h>
#include <runway_ImageProc/LightCoordinates.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <mutex>
#include <utility>

#define EARTHS_RADIUS 6378137
#define PI 3.14159265359
std::mutex mtx;

class SubscribeAndPublish
{
public:
    SubscribeAndPublish();
    void metersCallback(const runway_ImageProc::MetersPointsArrays::ConstPtr& msg);
    void gnssCallback(const nav_msgs::Odometry::ConstPtr& msg);
    visualization_msgs::Marker DeleteMarker();
    visualization_msgs::Marker ArrowMarker(float x, float y, float orientation);
    visualization_msgs::Marker SphereMarker(float x, float y);
    visualization_msgs::Marker EllipseMarker(float x, float y, float minor, float major, float orientation);
private:
    std::pair<float,float> globalFrame_transform(float x, float y, float orientation);
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Publisher vis_pub;
    ros::Subscriber meters_sub;
    ros::Subscriber gnss_sub;
    int marker_counter = 0;
    float gnss_x, gnss_y, gnss_rot;
    bool gnss_init = false;
};//End of class SubscribeAndPublish

#endif //RUNWAY_IMAGEPROC_POSITIONESTIMATOR_H
