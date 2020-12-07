#include "PositionEstimator.h"

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void initializeTransformMatrix()
{
    cv::Matx33f homography;
    pts_dest = {
            {-1.2975, 5.505}, //Top Left
            {1.2975, 5.505},  //Top Right
            {1.2975, 2.91}, //Bottom Right
            {-1.2975,2.91}, //Bottom Left
    };
    pts_src = {
            {613, 953}, //Top Left
            {1442,951}, //Top Right
            {1814, 1015}, //Bottom Right
            {237,1016},  //Bottom Left
    };
    cv::Mat test = cv::getPerspectiveTransform(pts_src, pts_dest);
    test.copyTo(transformM);
}

void initializeUndistortMatrixes()
{
    float cameraMatrixData[] = {1.6945139839119236e+03, 0., 1.0513426976359647e+03, 0., 1.6911722973100891e+03,
                                8.5302616014514581e+02, 0., 0., 1.};
    cv::Mat cameraMatrix_c(3, 3, CV_32FC1, cameraMatrixData);
    cameraMatrix_c.copyTo(cameraMatrix);
    float distCoeffData[] = {-2.6990623906843647e-01, -4.1278176195138259e-02, -2.2322574218248707e-03,
                             -4.9966928708844395e-04, 3.8123651292776789e-01};
    cv::Mat distCoeff_c(1, 5, CV_32FC1, distCoeffData);
    distCoeff_c.copyTo(distCoeff);
}

std::vector <cv::Point2f> getRelativePositions(cv::Mat image)
{
    //Undistort image
    cv::Mat undistorted_image;
    cv::undistort(image, undistorted_image, cameraMatrix, distCoeff);
    //Crop the image to a particular band in the image
    cv::Rect cropROI(cv::Point(0, crop_top), cv::Point(2056, crop_bottom));
    undistorted_image = undistorted_image(cropROI);
    //Threshold
    cv::threshold(undistorted_image, undistorted_image, thresholdValue, 255, 0);
    //Find contours
    std::vector <std::vector<cv::Point>> contours;
    std::vector <cv::Vec4i> hierarchy;
    cv::findContours(undistorted_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    //Calculate moments and light center pixels
    std::vector <cv::Moments> mu(contours.size());
    std::vector <cv::Point2f> lightCenter;
    for (size_t i = 0; i < contours.size(); i++)
    {
        mu[i] = moments(contours[i]);
        float lightCenter_y = static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5));
            float lightCenter_x = static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5));
            lightCenter.push_back(cv::Point2f(lightCenter_x, lightCenter_y + crop_top));
            lightCenter.push_back(cv::Point2f(lightCenter_x, lightCenter_y - uncertaintyPixels + crop_top));
            lightCenter.push_back(cv::Point2f(lightCenter_x + uncertaintyPixels, lightCenter_y + crop_top));
            lightCenter.push_back(cv::Point2f(lightCenter_x, lightCenter_y + uncertaintyPixels+ crop_top));
            lightCenter.push_back(cv::Point2f(lightCenter_x - uncertaintyPixels, lightCenter_y + crop_top));
    }
    std::vector <cv::Point2f> lightPositions;
    if (!lightCenter.empty())
    {
        cv::perspectiveTransform(lightCenter, lightPositions, transformM);
        for(int i = 0; i < lightPositions.size(); i+=5)
            std::cout << "Light " << i/5 << ": " << lightPositions[i].x << " " << lightPositions[i].y << std::endl;
    }
    //Show image
    if (show_images)
    {
        cv::Mat drawing = cv::Mat::zeros(image.size(), CV_8UC3);
        //Draw ligth centers
        for (size_t i = 0; i < lightCenter.size() / 5; i++)
        {
            cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            cv::circle(drawing, lightCenter[i * 5], 4, color, -1);
            cv::circle(drawing, lightCenter[i * 5 + 1], 2, color, -1);
            cv::circle(drawing, lightCenter[i * 5 + 2], 2, color, -1);
            cv::circle(drawing, lightCenter[i * 5 + 3], 2, color, -1);
            cv::circle(drawing, lightCenter[i * 5 + 4], 2, color, -1);
        }
        cv::resize(drawing, drawing, cv::Size(), 0.5, 0.5);
        cv::resize(undistorted_image, undistorted_image, cv::Size(), 0.5, 0.5);
        cv::resize(image, image, cv::Size(), 0.5, 0.5);
        cv::imshow("Final image drawing", drawing);
        cv::imshow("Final image", undistorted_image);
        cv::imshow("Initial image", image);
        cv::waitKey(1);
    }
    return lightPositions;
}


void SubscribeAndPublish::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    std::cout<<"Callback called\n";
    try
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
        std::vector <cv::Point2f> lightPositions = getRelativePositions(image);
        runway_ImageProc::MetersPointsArrays msg_pub;
        runway_ImageProc::MetersCoordinates coords;
        msg_pub.header.frame_id = "front_cam";
        msg_pub.header.stamp = ros::Time::now();
        msg_pub.lights_nr = lightPositions.size() / 5;
        for (int i = 0; i < lightPositions.size() / 5; i++)
        {
            coords.x = lightPositions[i*5].x;
            coords.y = lightPositions[i*5].y;
            msg_pub.light.push_back(coords);
            coords.x = lightPositions[i*5+1].x;
            coords.y = lightPositions[i*5+1].y;
            msg_pub.uncertainty_up.push_back(coords);
            coords.x = lightPositions[i*5+2].x;
            coords.y = lightPositions[i*5+2].y;
            msg_pub.uncertainty_right.push_back(coords);
            coords.x = lightPositions[i*5+3].x;
            coords.y = lightPositions[i*5+3].y;
            msg_pub.uncertainty_down.push_back(coords);
            coords.x = lightPositions[i*5+4].x;
            coords.y = lightPositions[i*5+4].y;
            msg_pub.uncertainty_left.push_back(coords);
        }
        pub.publish(msg_pub);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

SubscribeAndPublish::SubscribeAndPublish()
{
    //Param reading
    /*
    nh.getParam("front_showImages",show_images);
    nh.getParam("front_uncertaintyPixels",uncertaintyPixels);
    nh.getParam("front_thresholdValue",thresholdValue);
    nh.getParam("front_cropTop",crop_top);
    nh.getParam("front_cropBottom",crop_bottom);
     */
    show_images = true;
    uncertaintyPixels = 5;
    thresholdValue = 210;
    crop_top = 924;
    crop_bottom = 1432;
    //Initialize matrices
    initializeTransformMatrix();
    initializeUndistortMatrixes();

    //Points publishing topic
    pub = nh.advertise<runway_ImageProc::MetersPointsArrays>("/image_processing/relative_meters", 1);

    //Image subscribing topic
    sub = nh.subscribe("/left/image_raw", 1, &SubscribeAndPublish::imageCallback, this);
}

int main(int argc, char **argv)
{
    std::cout<<"Node initialization initiated\n";
    ros::init(argc, argv, "Image_Processing_Front");
    SubscribeAndPublish SAPObject;
    ros::spin();
}