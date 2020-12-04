#include "GPS_PositionEstimator.h"
struct Quaternion
{
    double w, x, y, z;
};

Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}
visualization_msgs::Marker SubscribeAndPublish::DeleteMarker(){
    visualization_msgs::Marker deleteMarker;
    if(marker_counter >= 0){
        deleteMarker.header.stamp = ros::Time();
        deleteMarker.header.frame_id = "/my_frame";
        deleteMarker.ns = "gps_estimation";
        deleteMarker.id = marker_counter;
        deleteMarker.type = visualization_msgs::Marker::CYLINDER;
        deleteMarker.action = visualization_msgs::Marker::DELETE;
        deleteMarker.color.a = 0.0;
        marker_counter--;
    }
    return deleteMarker;
}

visualization_msgs::Marker SubscribeAndPublish::SphereMarker(float x, float y){
    visualization_msgs::Marker sphereMarker;
        sphereMarker.header.stamp = ros::Time();
        sphereMarker.header.frame_id = "/my_frame";
        sphereMarker.ns = "gps_estimation";
        sphereMarker.id = marker_counter;
        sphereMarker.type = visualization_msgs::Marker::SPHERE;
        sphereMarker.action = visualization_msgs::Marker::ADD;
        sphereMarker.pose.position.x = x;
        sphereMarker.pose.position.y = y;
        sphereMarker.pose.position.z = 0;
        sphereMarker.pose.orientation.x = 0.0;
        sphereMarker.pose.orientation.y = 0.0;
        sphereMarker.pose.orientation.z = 0.0;
        sphereMarker.pose.orientation.w = 1.0;
        sphereMarker.scale.x = 0.2;
        sphereMarker.scale.y = 0.2;
        sphereMarker.scale.z = 0.2;
        sphereMarker.color.a = 1.0;
        sphereMarker.color.r = 0;
        sphereMarker.color.g = 1.0;
        sphereMarker.color.b = 0;
        marker_counter++;
    return sphereMarker;
}

visualization_msgs::Marker SubscribeAndPublish::EllipseMarker(float x, float y, float minor, float major, float orientation){
    visualization_msgs::Marker ellipseMarker;
    ellipseMarker.header.stamp = ros::Time();
    ellipseMarker.header.frame_id = "/my_frame";
    ellipseMarker.ns = "gps_estimation";
    ellipseMarker.id = marker_counter;
    ellipseMarker.type = visualization_msgs::Marker::CYLINDER;
    ellipseMarker.action = visualization_msgs::Marker::ADD;
    ellipseMarker.pose.position.x = x;
    ellipseMarker.pose.position.y = y;
    ellipseMarker.pose.position.z = 0;
    Quaternion orientationQ =  ToQuaternion(orientation,0,0);
    ellipseMarker.pose.orientation.x = orientationQ.x;
    ellipseMarker.pose.orientation.y = orientationQ.y;
    ellipseMarker.pose.orientation.z = orientationQ.z;
    ellipseMarker.pose.orientation.w = orientationQ.w;
    ellipseMarker.scale.x = minor;
    ellipseMarker.scale.y = major;
    ellipseMarker.scale.z = 0.05;
    ellipseMarker.color.a = 1.0;
    ellipseMarker.color.r = 1;
    ellipseMarker.color.g = 0;
    ellipseMarker.color.b = 0;
    marker_counter++;
    return ellipseMarker;
}

void SubscribeAndPublish::metersCallback(const runway_ImageProc::MetersPointsArrays::ConstPtr& msg)
{
    if(gnss_init)
    {
        std::cout << "Meters callback called\n";
        //First we delete all existing markers
        visualization_msgs::MarkerArray vis_msg;
        while (marker_counter != 0)
        {
            vis_msg.markers.push_back(DeleteMarker());
        }
        vis_pub.publish(vis_msg);
        vis_msg.markers.clear();
        //Now we calculate gps points for all light points and uncertainty ellipsoid dimensions
        runway_ImageProc::LightCoordinates msg_pub;
        for (int i = 0; i < msg->lights_nr; i++)
        {
            mtx.lock();
            msg_pub.uncertainty_major = std::sqrt(std::pow(msg->uncertainty_up[i].x - msg->uncertainty_down[i].x, 2) +
                                                  std::pow(msg->uncertainty_up[i].y - msg->uncertainty_down[i].y, 2) *
                                                  1.0);
            msg_pub.uncertainty_minor = std::sqrt(
                    std::pow(msg->uncertainty_left[i].x - msg->uncertainty_right[i].x, 2) +
                    std::pow(msg->uncertainty_left[i].y - msg->uncertainty_right[i].y, 2) * 1.0);
            float orientation = std::atan2(msg->uncertainty_up[i].x - msg->uncertainty_down[i].x,
                                           msg->uncertainty_up[i].y - msg->uncertainty_down[i].y) + gnss_rot;
            if (orientation > 2 * PI)
                orientation -= 2 * PI;
            std::pair<float, float> point_coordinates = globalFrame_transform(msg->light[i].x, msg->light[i].y,
                                                                              orientation);
            msg_pub.point_x = point_coordinates.first;
            msg_pub.point_y = point_coordinates.second;
            msg_pub.uncertainty_orientation = orientation;
            mtx.unlock();
            pub.publish(msg_pub);
            //Create markers and add to array
            vis_msg.markers.push_back(SphereMarker(point_coordinates.first,point_coordinates.second));
            vis_msg.markers.push_back(EllipseMarker(point_coordinates.first,point_coordinates.second,msg_pub.uncertainty_minor,msg_pub.uncertainty_major,orientation));
        }
        //Publish markers array
        vis_pub.publish(vis_msg);
    }
}

void SubscribeAndPublish::gnssCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::cout<<"GNSS callback called\n";
    double roll, pitch, yaw;
    tf::Quaternion quat;
    std::cout<<"X: "<<msg->pose.pose.orientation.x<<" Y: "<<msg->pose.pose.orientation.y<<" Z: "<<msg->pose.pose.orientation.z<<" W: "<<msg->pose.pose.orientation.w<<std::endl;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    mtx.lock();
    gnss_x = msg->pose.pose.position.x;
    gnss_y = msg->pose.pose.position.y;
    //gnss_rot = std::atan2(2.0*(msg->pose.pose.orientation.y*msg->pose.pose.orientation.z + msg->pose.pose.orientation.w*msg->pose.pose.orientation.x), 1 - 2 * (msg->pose.pose.orientation.x * msg->pose.pose.orientation.x + msg->pose.pose.orientation.y * msg->pose.pose.orientation.y));
    //gnss_rot = std::atan2(2.0*(q.y*q.z + q.w*q.x), 1 - 2 * (q.x * q.x + q.y * q.y));
    gnss_rot = yaw;
    gnss_init = true;
    mtx.unlock();
    std::cout<<"GNSS coordinates updated. X: "<<gnss_x<<" Y: "<<gnss_y<<" Rot: "<< gnss_rot * 180 / PI<< std::endl;
}

SubscribeAndPublish::SubscribeAndPublish()
{
    //Points publishing topic
    pub = nh.advertise<runway_ImageProc::LightCoordinates>("/image_processing/gps_coordinates", 1);
    vis_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
    //Image subscribing topic
    meters_sub = nh.subscribe("/image_processing/relative_meters", 1, &SubscribeAndPublish::metersCallback, this);
    gnss_sub = nh.subscribe("/gnss", 1, &SubscribeAndPublish::gnssCallback, this);
}

std::pair<float,float> SubscribeAndPublish::globalFrame_transform (float x, float y, float orientation)
{
    std::pair<float,float> globalFrame_loc;
    float distance = std::sqrt(std::pow(x - 0, 2) + std::pow(y - 0, 2) * 1.0);
    globalFrame_loc.first = gnss_x + distance * std::cos(orientation); //Check if you need to swap x and y based on robot frame
    globalFrame_loc.second = gnss_y + distance * std::sin(orientation); //Check if you need to swap x and y based on robot frame
    //(x2,y2)=(x1+l⋅cos(a),y1+l⋅sin(a))
    return globalFrame_loc;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS_Position_Estimator");
    SubscribeAndPublish SAPObject;
    ros::spin();
}
