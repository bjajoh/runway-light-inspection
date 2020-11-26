#include "GPS_PositionEstimator.h"

void SubscribeAndPublish::metersCallback(const runway_ImageProc::MetersPointsArrays::ConstPtr& msg)
{
    std::cout<<"Meters callback called\n";
    runway_ImageProc::LightCoordinates msg_pub;
    for(int i = 0; i < msg->lights_nr; i++)
    {
        mtx.lock();
        std::pair<float,float> point_coordinates = globalFrame_transform(msg->light[i].x,msg->light[i].y);
        msg_pub.point_x = point_coordinates.first;
        msg_pub.point_y = point_coordinates.second;
        msg_pub.uncertainty_major = std::sqrt(std::pow(msg->uncertainty_up[i].x - msg->uncertainty_down[i].x, 2) + std::pow(msg->uncertainty_up[i].y - msg->uncertainty_down[i].y, 2) * 1.0);
        msg_pub.uncertainty_minor = std::sqrt(std::pow(msg->uncertainty_left[i].x - msg->uncertainty_right[i].x, 2) + std::pow(msg->uncertainty_left[i].y - msg->uncertainty_right[i].y, 2) * 1.0);
        //See if you need to switch x and y or not
        float orientation = std::atan2(msg->uncertainty_up[i].x - msg->uncertainty_down[i].x, msg->uncertainty_up[i].y - msg->uncertainty_down[i].y) + gnss_rot;
        if(orientation > 2 * PI)
            orientation -= 2 * PI;
        msg_pub.uncertainty_orientation = orientation;
        mtx.unlock();
        pub.publish(msg_pub);
    }

}

void SubscribeAndPublish::gnssCallback(const runway_ImageProc::MetersPointsArrays::ConstPtr& msg)
{
    std::cout<<"GNSS callback called\n";
    mtx.lock();
    //gnss_x = ;
    //gnss_y = ;
    //nss_rot = ;
    mtx.unlock();
    std::cout<<"GNSS coordinates updated. X: "<<gnss_x<<" Y: "<<gnss_y<<" Rot: "<< gnss_rot<<std::endl;
}

SubscribeAndPublish::SubscribeAndPublish()
{
    //REMOVE THIS ONCE GNSS CALLBACK IS FINALIZED
    gnss_x = 50;
    gnss_y = 50;
    gnss_rot = 0.785398;
    //Points publishing topic
    pub = nh.advertise<runway_ImageProc::MetersPointsArrays>("/image_processing/gps_coordinates", 1);

    //Image subscribing topic
    meters_sub = nh.subscribe("/image_processing/relative_meters", 1, &SubscribeAndPublish::metersCallback, this);
    gnss_sub = nh.subscribe("/gnss", 1, &SubscribeAndPublish::gnssCallback, this);
}

std::pair<float,float> SubscribeAndPublish::globalFrame_transform (float x, float y)
{
    std::pair<float,float> globalFrame_loc;
    float distance = std::sqrt(std::pow(x - 0, 2) + std::pow(y - 0, 2) * 1.0);
    globalFrame_loc.first = gnss_x + distance * std::cos(gnss_rot); //Check if you need to swap x and y based on robot frame
    globalFrame_loc.second = gnss_y + distance * std::sin(gnss_rot); //Check if you need to swap x and y based on robot frame
    //(x2,y2)=(x1+l⋅cos(a),y1+l⋅sin(a))
    return globalFrame_loc;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS_Position_Estimator");
    SubscribeAndPublish SAPObject;
    ros::spin();
}
