#include "GPS_PositionEstimator.h"

void SubscribeAndPublish::metersCallback(const runway_ImageProc::MetersPointsArrays::ConstPtr& msg)
{
    std::cout<<"Meters callback called\n";
    runway_ImageProc::UtmCoordinates msg_pub;
    for(int i = 0; i < msg->lights_nr; i++)
    {
        gps_transform(msg->light[i].x,msg->light[i].y);
        msg->uncertainty_up[i];
        msg->uncertainty_right[i];
        msg->uncertainty_down[i];
        msg->uncertainty_left[i];

        msg_pub.point_easting = 0;
        msg_pub.point_northing = 0;
        msg_pub.uncertainty_major = 0;
        msg_pub.uncertainty_minor = 0;
        msg_pub.uncertainty_orientation = 0;

        //pub.publish(msg_pub);
    }

}

void SubscribeAndPublish::gnssCallback(const runway_ImageProc::MetersPointsArrays::ConstPtr& msg)
{
    std::cout<<"GNSS callback called\n";
    mtx.lock();
    gnss_lat = 57.033435;
    gnss_long = 9.922650;
    mtx.unlock();
    std::cout<<"GNSS coordinates updated. Lat: "<<gnss_lat<<" Long: "<<gnss_long<<std::endl;
}

SubscribeAndPublish::SubscribeAndPublish()
{
    //REMOVE THIS ONCE GNSS CALLBACK IS FINALIZED
    gnss_lat = 57.033435;
    gnss_long = 9.922650;
    //Points publishing topic
    //pub = nh.advertise<runway_ImageProc::MetersPointsArrays>("/image_processing/gps_coordinates", 1);

    //Image subscribing topic
    meters_sub = nh.subscribe("/image_processing/relative_meters", 1, &SubscribeAndPublish::metersCallback, this);
    gnss_sub = nh.subscribe("/gnss", 1, &SubscribeAndPublish::gnssCallback, this);
}

std::pair<float,float> SubscribeAndPublish::gps_transform(float x, float y)
{
    std::pair<float,float> gps_loc;
    mtx.lock();
    gps_loc.first = gnss_lat + (180/PI)*(y/EARTHS_RADIUS);
    gps_loc.second = gnss_long + (180/PI)*(x/EARTHS_RADIUS)/std::cos(PI/180.0*gnss_lat);
    mtx.unlock();
    //(x2,y2)=(x1+l⋅cos(a),y1+l⋅sin(a))
    return gps_loc;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS_Position_Estimator");
    SubscribeAndPublish SAPObject;
    ros::MultiThreadedSpinner spinner(2); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
}
