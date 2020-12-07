#ifndef RUNWAY_IMAGEPROC_POSITIONESTIMATOR_H
#define RUNWAY_IMAGEPROC_POSITIONESTIMATOR_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <runway_ImageProc/MetersCoordinates.h>
#include <runway_ImageProc/MetersPointsArrays.h>

cv::RNG rng(12345);
bool show_images;
int crop_top, crop_bottom, uncertaintyPixels, thresholdValue;
cv::Mat transformM, cameraMatrix, distCoeff;
std::vector <cv::Point2f> pts_dest, pts_src;
float transformGNSS_x = 0, transformGNSS_y = 0; //Set these parameters to reflect the Camera to GNSS transformation

float map(float x, float in_min, float in_max, float out_min, float out_max);

void initializeTransformMatrix();

void initializeUndistortMatrixes();
//void initializeTransformMatrix2();

std::vector <cv::Point2f> getRelativePositions(cv::Mat image);

class SubscribeAndPublish
{
public:
    SubscribeAndPublish();

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
};//End of class SubscribeAndPublish

#endif //RUNWAY_IMAGEPROC_POSITIONESTIMATOR_H
