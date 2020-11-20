#include "PositionEstimator.h"

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void initializeTransformMatrix()
{
    /*
     *Old version
    //Initialize initial im age points
    std::vector <cv::Point2f> srcPoints;
    srcPoints.push_back(src_tl);
    srcPoints.push_back(src_tr);
    srcPoints.push_back(src_br);
    srcPoints.push_back(src_bl);
    //Compute width and height of new image and initalize points
    float widthA = sqrt(pow(src_br.x - src_bl.x, 2) + pow(src_br.y - src_bl.y, 2));
    float widthB = sqrt(pow(src_tr.x - src_tl.x, 2) + pow(src_tr.y - src_tl.y, 2));
    maxWidth = std::max((int) widthA, (int) widthB);
    float heightA = sqrt(pow(src_tr.x - src_br.x, 2) + pow(src_tr.y - src_br.y, 2));
    float heightB = sqrt(pow(src_tl.x - src_bl.x, 2) + pow(src_tl.y - src_bl.y, 2));
    maxHeight = std::max((int) heightA, (int) heightB);
    std::vector <cv::Point2f> dstPoints;
    cv::Point2f dst_tl(0, 0), dst_tr(maxWidth - 1, 0), dst_br(maxWidth - 1, maxHeight - 1), dst_bl(0, maxHeight - 1);
    dstPoints.push_back(dst_tl);
    dstPoints.push_back(dst_tr);
    dstPoints.push_back(dst_br);
    dstPoints.push_back(dst_bl);
    //Compute perspective transform matrix
    transformM = cv::getPerspectiveTransform(srcPoints, dstPoints);
     */
    //Initialize initial image points
    std::vector <cv::Point2f> srcPoints;
    srcPoints.push_back(src_tl);
    srcPoints.push_back(src_tr);
    srcPoints.push_back(src_br);
    srcPoints.push_back(src_bl);
    //Initialize real life points
    std::vector <cv::Point2f> dstPoints;
    dstPoints.push_back(dst_tl);
    dstPoints.push_back(dst_tr);
    dstPoints.push_back(dst_br);
    dstPoints.push_back(dst_bl);
    //Compute perspective transform matrix
    cv::Mat test = cv::getPerspectiveTransform(srcPoints, dstPoints);
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

void initializeCameraCalibration()
{
    crop_top = std::min(src_tl.y,src_tr.y) - crop_topMargin;
    crop_bottom = std::max(src_bl.y,src_br.y) + crop_bottomMargin;
    src_tl.y -= crop_top; src_br.y-=crop_top; src_bl.y-=crop_top; src_tr.y-=crop_top;
}

std::vector <cv::Point2f> getRelativePositions(cv::Mat image)
{
    //Undistort image
    cv::Mat undistorted_image;
    cv::undistort(image, undistorted_image, cameraMatrix, distCoeff);
    //Crop the image to a particular band in the image
    //cv::Rect cropROI(cv::Point(0, crop_top), cv::Point(2056, crop_bottom));
    //undistorted_image = undistorted_image(cropROI);
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
            lightCenter.push_back(cv::Point2f(lightCenter_x, lightCenter_y));
            /*
            if (lightCenter_y - uncertaintyPixels < 0)
                lightCenter.push_back(cv::Point2f(lightCenter_x, 0));
            else
                lightCenter.push_back(cv::Point2f(lightCenter_x, lightCenter_y - uncertaintyPixels));
            if (lightCenter_x + uncertaintyPixels > undistorted_image.cols - 1)
                lightCenter.push_back(cv::Point2f(undistorted_image.cols - 1, lightCenter_y));
            else
                lightCenter.push_back(cv::Point2f(lightCenter_x + uncertaintyPixels, lightCenter_y));
            if (lightCenter_y + uncertaintyPixels > undistorted_image.rows - 1)
                lightCenter.push_back(cv::Point2f(lightCenter_x, undistorted_image.rows - 1));
            else
                lightCenter.push_back(cv::Point2f(lightCenter_x, lightCenter_y + uncertaintyPixels));
            if (lightCenter_x - uncertaintyPixels < 0)
                lightCenter.push_back(cv::Point2f(0, lightCenter_y));
            else
                lightCenter.push_back(cv::Point2f(lightCenter_x - uncertaintyPixels, lightCenter_y));
                */
        //}
    }
    std::vector <cv::Point2f> lightPositions;
    if (!lightCenter.empty())
    {
        cv::perspectiveTransform(lightCenter, lightPositions, transformM);
        /* Old method
        //Map light centers to approximate position in meters relative to camera
        for (size_t i = 0; i < lightCenter_transformed.size(); i++)
        {
            float lightX = map(lightCenter_transformed[i].x, 0, maxWidth - 1, -2.86, 2.86) + transformGNSS_x; //NEEDS TO BE CALIBRATED
            float lightY = map(maxHeight - lightCenter_transformed[i].y, 0, maxHeight - 1, 5.52,
                               10.14) + transformGNSS_y; //NEEDS TO BE CALIBRATED
            std::cout << lightCenter_transformed[i].x << " " << maxHeight - lightCenter_transformed[i].y << std::endl;
            std::cout << "Light " << i << ": " << lightX << " " << lightY << std::endl;
            lightPositions.push_back(cv::Point2f(lightX, lightY));
        }
         */
        for(int i = 0; i < lightPositions.size(); i++)
            std::cout << "Light " << i << ": " << lightPositions[i].x << " " << lightPositions[i].y << std::endl;
    }
    //Show image
    if (show_images)
    {
        cv::Mat drawing = cv::Mat::zeros(undistorted_image.size(), CV_8UC3);
        cv::warpPerspective(undistorted_image, drawing, transformM, cv::Size(maxWidth, maxHeight));
        cv::cvtColor(drawing, drawing, cv::COLOR_GRAY2BGR);
        //Draw ligth centers
        for (size_t i = 0; i < lightCenter.size() / 5; i++)
        {
            cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            //drawContours(drawing, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
            /*
            cv::circle(drawing, lightCenter_transformed[i * 5], 4, color, -1);
            cv::circle(drawing, lightCenter_transformed[i * 5 + 1], 2, color, -1);
            cv::circle(drawing, lightCenter_transformed[i * 5 + 2], 2, color, -1);
            cv::circle(drawing, lightCenter_transformed[i * 5 + 3], 2, color, -1);
            cv::circle(drawing, lightCenter_transformed[i * 5 + 4], 2, color, -1);
             */
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
    nh.getParam("front_showImages",show_images);
    nh.getParam("front_uncertaintyPixels",uncertaintyPixels);
    nh.getParam("front_thresholdValue",thresholdValue);
    nh.getParam("front_cropTopMargin",crop_topMargin);
    nh.getParam("front_cropBottomMargin",crop_bottomMargin);
     */
    show_images = true;
    uncertaintyPixels = 10;
    thresholdValue = 150;
    crop_topMargin = 40;
    crop_bottomMargin = 40;
    //Initialize matrices
    initializeCameraCalibration();
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