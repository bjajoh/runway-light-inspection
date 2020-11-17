//
// Created by silverback on 11/12/20.
//

#ifndef RUNWAY_IMAGEPROC_POSITIONESTIMATOR_H
#define RUNWAY_IMAGEPROC_POSITIONESTIMATOR_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <runway_ImageProc/MetersCoordinates.h>
#include <runway_ImageProc/MetersPointsArrays.h>
cv::RNG rng(12345);
bool show_images;
int crop_top, crop_bottom, maxWidth, maxHeight, uncertaintyPixels;
cv::Point2f src_tl(496, 936), src_tr(1531, 935), src_br(1974, 1053), src_bl(38,1053); //NEEDS TO BE CALIBRATED
cv::Mat transformM, cameraMatrix, distCoeff;
float transformGNSS_x = 0, transformGNSS_y = 0; //Set these parameters to reflect the Camera to GNSS transformation

float map(float x, float in_min, float in_max, float out_min, float out_max);
void initializeCameraCalibration();
void initializeTransformMatrix();
void initializeUndistortMatrixes();

std::vector<cv::Point2f> getRelativePositions(cv::Mat image);
class SubscribeAndPublish
{
public:
    SubscribeAndPublish();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
};//End of class SubscribeAndPublish

#endif //RUNWAY_IMAGEPROC_POSITIONESTIMATOR_H
