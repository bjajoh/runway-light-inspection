#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/features2d.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

cv::RNG rng(12345);
#define showImage 0
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

std::vector<cv::Point2f> getRelativePositions(cv::Mat image)
{
    //Crop the image to a particular band in the image
    //cv::Rect cropROI(cv::Point(0, 480), cv::Point(2056, 640));
    //image = image(cropROI);2
    cv::Mat initialImage, unidstortedImage;
    image.copyTo(initialImage);
    //Undistort image
    float cameraMatrixData[] = { 1.6945136795020819e+03, 0., 1.0513426826746233e+03, 0., 1.6911721955170856e+03, 8.5302576653616484e+02, 0., 0., 1. };
    cv::Mat cameraMatrix(3, 3, CV_32FC1, cameraMatrixData);
    float distCoeffData[] = { -2.6990671556165891e-01, -4.1277164997035813e-02, -2.2322061311698415e-03, -4.9975847204175605e-04, 3.8123598487614557e-01 };
    cv::Mat distCoeff(1, 5, CV_32FC1, distCoeffData);
    cv::undistort(initialImage, unidstortedImage, cameraMatrix, distCoeff);
    cv::undistort(initialImage, image, cameraMatrix, distCoeff);
    cv::imwrite("C:/Users/rrung/source/repos/Masters/IDS Camera Image Collection/IDS Camera Image Collection/Pictures/Square_0_undistorted.JPG", unidstortedImage);
    //Initialize initial image points
    std::vector<cv::Point2f> srcPoints;
    cv::Point2f src_tl(409, 299), src_tr(1486, 285), src_br(1804, 360), src_bl(102, 380); //NEEDS TO BE CALIBRATED
    srcPoints.push_back(src_tl); srcPoints.push_back(src_tr); srcPoints.push_back(src_br); srcPoints.push_back(src_bl);
    //Compute width and height of new image and initalize points
    float widthA = sqrt(pow(src_br.x - src_bl.x, 2) + pow(src_br.y - src_bl.y, 2));
    float widthB = sqrt(pow(src_tr.x - src_tl.x, 2) + pow(src_tr.y - src_tl.y, 2));
    int maxWidth = std::max((int)widthA, (int)widthB);
    float heightA = sqrt(pow(src_tr.x - src_br.x, 2) + pow(src_tr.y - src_br.y, 2));
    float heightB = sqrt(pow(src_tl.x - src_bl.x, 2) + pow(src_tl.y - src_bl.y, 2));
    int maxHeight = std::max((int)heightA, (int)heightB);
    std::vector<cv::Point2f> dstPoints;
    cv::Point2f dst_tl(0, 0), dst_tr(maxWidth - 1, 0), dst_br(maxWidth - 1, maxHeight - 1), dst_bl(0, maxHeight - 1);
    dstPoints.push_back(dst_tl); dstPoints.push_back(dst_tr); dstPoints.push_back(dst_br); dstPoints.push_back(dst_bl);
    //Compute perspective transform matrix and apply it
    cv::Mat transformM = cv::getPerspectiveTransform(srcPoints, dstPoints);
    cv::Mat warpedImage, warpedInitialImage;
    cv::warpPerspective(image, warpedImage, transformM, cv::Size(maxWidth, maxHeight));
    cv::warpPerspective(unidstortedImage, warpedInitialImage, transformM, cv::Size(maxWidth, maxHeight));
    //Threshold
    cv::threshold(warpedImage, warpedImage, 130, 255, 0);
    //Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(warpedImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat drawing = cv::Mat::zeros(warpedImage.size(), CV_8UC3);
    //Calculate moments
    std::vector<cv::Moments> mu(contours.size());
    for (size_t i = 0; i < contours.size(); i++)
    {
        mu[i] = moments(contours[i]);
    }
    std::vector<cv::Point2f> lightCenter(contours.size());
    for (size_t i = 0; i < contours.size(); i++)
    {
        //add 1e-5 to avoid division by zero
        lightCenter[i] = cv::Point2f(static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5)),
                                     static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5)));
    }
    //Map light centers to approximate position in meters relative to camera
    std::vector<cv::Point2f> lightPositions;
    for (size_t i = 0; i < contours.size(); i++)
    {
        float lightX = map(lightCenter[i].x, 0, warpedImage.cols - 1, -7.02, 7.02); //NEEDS TO BE CALIBRATED
        float lightY = map(warpedImage.rows - 1 - lightCenter[i].y, 0, warpedImage.rows - 1, 7.43, 12.885); //NEEDS TO BE CALIBRATED
        std::cout << "Light " << i << ": " << lightX << " " << lightY << std::endl;
        lightPositions.push_back(cv::Point2f(lightX, lightY));
    }
    //Show image
    if (showImage)
    {
        //Draw ligth centers
        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            //drawContours(drawing, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
            cv::circle(drawing, lightCenter[i], 4, color, -1);
        }
        cv::imshow("Final image", drawing);
        cv::imshow("Warped image", warpedImage);
        cv::imshow("Initial image", initialImage);
        cv::imshow("Initial undistorted image", unidstortedImage);
        cv::imshow("Initial warped image", warpedInitialImage);
        cv::waitKey();
    }
    return lightPositions;
}

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        //Points publishing topic
        pub = nh.advertise<sensor_msgs::PointCloud2>("/published_topic", 1);

        //Image subscribing topic
        sub = nh.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::imageCallback, this);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            std::vector<cv::Point2f> lightPositions = getRelativePositions(image);
            PointCloud::Ptr msg (new PointCloud);
            msg->header.frame_id = "front_cam";
            msg->height = lightPositions.size();
            msg->width = 1;
            for(int i = 0; i < lightPositions.size(); i++)
            {
                msg->points.push_back (pcl::PointXYZ(lightPositions[i].x, lightPositions[i].y, 0));
            }
            //pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
            ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("camera_frame_positions", 1);
            pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
            pub.publish(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Image_Processing");
    SubscribeAndPublish SAPObject;
    ros::spin();
}