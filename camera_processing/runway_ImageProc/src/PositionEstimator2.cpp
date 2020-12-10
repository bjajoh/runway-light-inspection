#include "PositionEstimator2.h"

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void initializeTransformMatrix()
{
    cv::Matx33f homography;
    pts_dest = {
            {-129.75, 550.5}, //Top Left
            {129.75, 550.5},  //Top Right
            {129.75, 291}, //Bottom Right
            {-129.75,291}, //Bottom Left
    };
    pts_src = {
            {335, 540}, //Top Left
            {950,543}, //Top Right
            {1221, 589}, //Bottom Right
            {61,582},  //Bottom Left
    };
    cv::Mat test = cv::getPerspectiveTransform(pts_src, pts_dest);
    std::cout<<"Init2: "<<std::endl<<test<<std::endl;
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
    std::vector <cv::Point2f> lightCenter, lightCenter_transformed;
    for (size_t i = 0; i < contours.size(); i++)
    {
        mu[i] = moments(contours[i]);
        float lightCenter_y = static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5));
        float lightCenter_x = static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5));
        lightCenter.push_back(cv::Point2f(lightCenter_x, lightCenter_y + crop_top));
        if (lightCenter_y - uncertaintyPixels < 0)
            lightCenter.push_back(cv::Point2f(lightCenter_x, 0 + crop_top));
        else
            lightCenter.push_back(cv::Point2f(lightCenter_x, lightCenter_y - uncertaintyPixels + crop_top));
        if (lightCenter_x + uncertaintyPixels > undistorted_image.cols - 1)
            lightCenter.push_back(cv::Point2f(undistorted_image.cols - 1, lightCenter_y + crop_top));
        else
            lightCenter.push_back(cv::Point2f(lightCenter_x + uncertaintyPixels, lightCenter_y + crop_top));
        if (lightCenter_y + uncertaintyPixels > undistorted_image.rows - 1)
            lightCenter.push_back(cv::Point2f(lightCenter_x, undistorted_image.rows - 1 + crop_top));
        else
            lightCenter.push_back(cv::Point2f(lightCenter_x, lightCenter_y + uncertaintyPixels+ crop_top));
        if (lightCenter_x - uncertaintyPixels < 0)
            lightCenter.push_back(cv::Point2f(0, lightCenter_y + crop_top));
        else
            lightCenter.push_back(cv::Point2f(lightCenter_x - uncertaintyPixels, lightCenter_y + crop_top));
    }
    std::vector <cv::Point2f> lightPositions;
    if (!lightCenter.empty())
    {
        cv::perspectiveTransform(lightCenter, lightPositions, transformM);
        for(int i = 0; i < lightPositions.size(); i++)
            std::cout << "Light " << i << ": " << lightPositions[i].x << " " << lightPositions[i].y << std::endl;
    }
    //Show image
    if (show_images)
    {
        cv::Mat drawing = cv::Mat::zeros(undistorted_image.size(), CV_8UC3);
        cv::warpPerspective(undistorted_image, drawing, transformM, cv::Size(undistorted_image.rows,undistorted_image.cols));
        cv::cvtColor(drawing, drawing, cv::COLOR_GRAY2BGR);
        //Draw ligth centers
        for (size_t i = 0; i < lightCenter.size() / 5; i++)
        {
            cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            cv::circle(drawing, lightCenter_transformed[i * 5], 4, color, -1);
            cv::circle(drawing, lightCenter_transformed[i * 5 + 1], 2, color, -1);
            cv::circle(drawing, lightCenter_transformed[i * 5 + 2], 2, color, -1);
            cv::circle(drawing, lightCenter_transformed[i * 5 + 3], 2, color, -1);
            cv::circle(drawing, lightCenter_transformed[i * 5 + 4], 2, color, -1);
        }
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
    nh.getParam("back_showImages",show_images);
    nh.getParam("back_uncertaintyPixels",uncertaintyPixels);
    nh.getParam("back_thresholdValue",thresholdValue);
    nh.getParam("back_cropTop",crop_top);
    nh.getParam("back_cropBottom",crop_bottom);
     */
    show_images = true;
    uncertaintyPixels = 10;
    thresholdValue = 100;
    crop_top = 200;
    crop_bottom = 1000;
    //Initialize matrices
    initializeTransformMatrix();
    initializeUndistortMatrixes();

    //Points publishing topic
    pub = nh.advertise<runway_ImageProc::MetersPointsArrays>("/image_processing/relative_meters", 1);

    //Image subscribing topic
    sub = nh.subscribe("/camera/image_raw", 1, &SubscribeAndPublish::imageCallback, this);
}

int main(int argc, char **argv)
{
    std::cout<<"Node initialization initiated B-)\n";
    ros::init(argc, argv, "Image_Processing_Front");
    SubscribeAndPublish SAPObject;
    ros::spin();
}