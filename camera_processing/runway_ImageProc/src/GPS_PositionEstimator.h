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
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
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
    SubscribeAndPublish() :  tf_listener(tf_buffer)
    {
        //Points publishing topic
        pub = nh.advertise<runway_ImageProc::LightCoordinates>("/image_processing/gps_coordinates", 1);
        vis_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
        //Image subscribing topic
        meters_sub = nh.subscribe("/image_processing/relative_meters", 1, &SubscribeAndPublish::metersCallback, this);
        gnss_sub = nh.subscribe("/odometry/filtered_map", 1, &SubscribeAndPublish::gnssCallback, this);
    }
    void metersCallback(const runway_ImageProc::MetersPointsArrays::ConstPtr& msg);
    void gnssCallback(const nav_msgs::Odometry::ConstPtr& msg);
    visualization_msgs::Marker DeleteMarker();
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
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
};//End of class SubscribeAndPublish

#endif //RUNWAY_IMAGEPROC_POSITIONESTIMATOR_H
