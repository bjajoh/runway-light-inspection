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

visualization_msgs::Marker SubscribeAndPublish::DeleteMarker()
{
    visualization_msgs::Marker deleteMarker;
    if (marker_counter > 0)
    {
        deleteMarker.header.stamp = ros::Time();
        deleteMarker.header.frame_id = "/map";
        deleteMarker.ns = "gps_estimation";
        deleteMarker.id = marker_counter;
        deleteMarker.type = visualization_msgs::Marker::CYLINDER;
        deleteMarker.action = visualization_msgs::Marker::DELETE;
        deleteMarker.color.a = 0.0;
        marker_counter--;
    }
    return deleteMarker;
}

visualization_msgs::Marker SubscribeAndPublish::SphereMarker(float x, float y)
{
    visualization_msgs::Marker sphereMarker;
    sphereMarker.header.stamp = ros::Time();
    sphereMarker.header.frame_id = "/map";
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
    sphereMarker.scale.x = 0.1;
    sphereMarker.scale.y = 0.1;
    sphereMarker.scale.z = 0.1;
    sphereMarker.color.a = 1.0;
    sphereMarker.color.r = 0;
    sphereMarker.color.g = 1.0;
    sphereMarker.color.b = 0;
    marker_counter++;
    return sphereMarker;
}

visualization_msgs::Marker SubscribeAndPublish::EllipseMarker(float x, float y, float minor, float major, float orientation)
{
    visualization_msgs::Marker ellipseMarker;
    ellipseMarker.header.stamp = ros::Time();
    ellipseMarker.header.frame_id = "/map";
    ellipseMarker.ns = "gps_estimation";
    ellipseMarker.id = marker_counter;
    ellipseMarker.type = visualization_msgs::Marker::CYLINDER;
    ellipseMarker.action = visualization_msgs::Marker::ADD;
    ellipseMarker.pose.position.x = x;
    ellipseMarker.pose.position.y = y;
    ellipseMarker.pose.position.z = 0;
    Quaternion orientationQ = ToQuaternion(orientation, 0, 0);
    ellipseMarker.pose.orientation.x = orientationQ.x;
    ellipseMarker.pose.orientation.y = orientationQ.y;
    ellipseMarker.pose.orientation.z = orientationQ.z;
    ellipseMarker.pose.orientation.w = orientationQ.w;
    ellipseMarker.scale.x = major;
    ellipseMarker.scale.y = minor;
    ellipseMarker.scale.z = 0.05;
    ellipseMarker.color.a = 1.0;
    ellipseMarker.color.r = 1;
    ellipseMarker.color.g = 0;
    ellipseMarker.color.b = 0;
    marker_counter++;
    return ellipseMarker;
}

void SubscribeAndPublish::metersCallback(const runway_ImageProc::MetersPointsArrays::ConstPtr &msg)
{
        std::cout << "Meters callback called\n";
        //First we delete all existing markers
        visualization_msgs::MarkerArray vis_msg;
        while (marker_counter > 0)
        {
            vis_msg.markers.push_back(DeleteMarker());
        }
        vis_pub.publish(vis_msg);
        vis_msg.markers.clear();
        //Now we calculate gps points for all light points and uncertainty ellipsoid dimensions
        runway_ImageProc::LightCoordinates msg_pub;
        for (int i = 0; i < msg->lights_nr; i++)
        {
            msg_pub.uncertainty_major = std::sqrt(std::pow(msg->uncertainty_up[i].x - msg->uncertainty_down[i].x, 2) +
                                                  std::pow(msg->uncertainty_up[i].y - msg->uncertainty_down[i].y, 2));
            msg_pub.uncertainty_minor = std::sqrt(
                    std::pow(msg->uncertainty_left[i].x - msg->uncertainty_right[i].x, 2) +
                    std::pow(msg->uncertainty_left[i].y - msg->uncertainty_right[i].y, 2) );
            geometry_msgs::PoseStamped camera_pose;
            //First up, we transform the light point
            camera_pose.header = msg->header;
            camera_pose.pose.position.x = msg->light[i].x;
            camera_pose.pose.position.y = msg->light[i].y;
            while(!tf_buffer.canTransform("base_link/camera_front", msg->header.stamp, "map", ros::Time::now(), "base_link", ros::Duration(1.0)));
            geometry_msgs::PoseStamped map_pose = tf_buffer.transform(camera_pose, "map");
            msg_pub.point_x = map_pose.pose.position.x;
            msg_pub.point_y = map_pose.pose.position.y;
            //Now we get the 2 major uncertainty points in order to calculate the ellipse's orientation
            camera_pose.pose.position.x = msg->uncertainty_up[i].x;
            camera_pose.pose.position.y = msg->uncertainty_up[i].y;
            while(!tf_buffer.canTransform("base_link/camera_front", msg->header.stamp, "map", ros::Time::now(), "base_link", ros::Duration(1.0)));
            map_pose = tf_buffer.transform(camera_pose, "map");
            camera_pose.pose.position.x = msg->uncertainty_down[i].x;
            camera_pose.pose.position.y = msg->uncertainty_down[i].y;
            while(!tf_buffer.canTransform("base_link/camera_front", msg->header.stamp, "map", ros::Time::now(), "base_link", ros::Duration(1.0)));
            geometry_msgs::PoseStamped map_pose2 = tf_buffer.transform(camera_pose, "map");
            msg_pub.uncertainty_orientation = std::atan2(map_pose.pose.position.x - map_pose2.pose.position.x,map_pose.pose.position.y - map_pose2.pose.position.y);
            pub.publish(msg_pub);
            //Create markers and add to array
            vis_msg.markers.push_back(SphereMarker(msg_pub.point_x, msg_pub.point_y));
            vis_msg.markers.push_back(
                    EllipseMarker(msg_pub.point_x, msg_pub.point_y, msg_pub.uncertainty_minor,
                                  msg_pub.uncertainty_major, msg_pub.uncertainty_orientation));
        }
        //Publish markers array
        vis_pub.publish(vis_msg);
}

void SubscribeAndPublish::gnssCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //std::cout << "GNSS callback called\n";
    double roll, pitch, yaw;
    tf::Quaternion quat;
    //std::cout << "X: " << msg->pose.pose.orientation.x << " Y: " << msg->pose.pose.orientation.y << " Z: "
    //          << msg->pose.pose.orientation.z << " W: " << msg->pose.pose.orientation.w << std::endl;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    mtx.lock();
    gnss_x = msg->pose.pose.position.x;
    gnss_y = msg->pose.pose.position.y;
    //gnss_rot = std::atan2(2.0*(msg->pose.pose.orientation.y*msg->pose.pose.orientation.z + msg->pose.pose.orientation.w*msg->pose.pose.orientation.x), 1 - 2 * (msg->pose.pose.orientation.x * msg->pose.pose.orientation.x + msg->pose.pose.orientation.y * msg->pose.pose.orientation.y));
    //gnss_rot = std::atan2(2.0*(q.y*q.z + q.w*q.x), 1 - 2 * (q.x * q.x + q.y * q.y));
    if(yaw < 0)
        gnss_rot = yaw + 2*PI;
    else
        gnss_rot = yaw;
    if(gnss_rot > 2*PI)
        gnss_rot -= 2*PI;
    //std::cout << "GNSS coordinates updated. X: " << gnss_x << " Y: " << gnss_y << " Rot: " << gnss_rot *180 / PI
     //         << std::endl;
    mtx.unlock();
    gnss_init = true;
}

std::pair<float, float> SubscribeAndPublish::globalFrame_transform(float x, float y, float orientation)
{
    std::pair<float, float> globalFrame_loc;
    float distance = std::sqrt(std::pow(x, 0) + std::pow(y, 0));
    globalFrame_loc.first =
            gnss_x + distance * std::cos(orientation); //Check if you need to swap x and y based on robot frame
    globalFrame_loc.second =
            gnss_y + distance * std::sin(orientation); //Check if you need to swap x and y based on robot frame
    //(x2,y2)=(x1+l⋅cos(a),y1+l⋅sin(a))
    return globalFrame_loc;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS_Position_Estimator");
    SubscribeAndPublish SAPObject;
    ros::AsyncSpinner spinner(2); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
    //ros::spin();
}
