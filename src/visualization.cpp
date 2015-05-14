#include "exploration_baxter/visualization.h"

void visualization::dispMkarr(octomap::OcTree *octree, visualization_msgs::MarkerArray &marker_arr, std_msgs::ColorRGBA &color, const ros::Time &rostime, std::string frame)
{
    /* Node size in certain depth */
    double depth_size;
    /* Visualization configuration */
    for (int i = 0; i < marker_arr.markers.size(); i++) {
        depth_size = octree->getNodeSize(i);
        marker_arr.markers[i].header.frame_id = frame;
        marker_arr.markers[i].header.stamp = rostime;
        marker_arr.markers[i].ns = "map";
        marker_arr.markers[i].id = i;
        marker_arr.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        marker_arr.markers[i].scale.x = depth_size;
        marker_arr.markers[i].scale.y = depth_size;
        marker_arr.markers[i].scale.z = depth_size;
        marker_arr.markers[i].color = color;
        marker_arr.markers[i].lifetime = ros::Duration();
        if (marker_arr.markers[i].points.size() > 0) {
            marker_arr.markers[i].action = visualization_msgs::Marker::ADD;
        }
        else {
            marker_arr.markers[i].action = visualization_msgs::Marker::DELETE;
        }
    }
}
void visualization::dispMkarr(octomap::OcTree octree, visualization_msgs::MarkerArray &marker_arr, std_msgs::ColorRGBA &color, const ros::Time &rostime, std::string frame)
{
    /* Node size in certain depth */
    double depth_size;
    /* Visualization configuration */
    for (int i = 0; i < marker_arr.markers.size(); i++) {
        depth_size = octree.getNodeSize(i);
        marker_arr.markers[i].header.frame_id = frame;
        marker_arr.markers[i].header.stamp = rostime;
        marker_arr.markers[i].ns = "map";
        marker_arr.markers[i].id = i;
        marker_arr.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        marker_arr.markers[i].scale.x = depth_size;
        marker_arr.markers[i].scale.y = depth_size;
        marker_arr.markers[i].scale.z = depth_size;
        marker_arr.markers[i].color = color;
        marker_arr.markers[i].lifetime = ros::Duration();
        if (marker_arr.markers[i].points.size() > 0) {
            marker_arr.markers[i].action = visualization_msgs::Marker::ADD;
        }
        else {
            marker_arr.markers[i].action = visualization_msgs::Marker::DELETE;
        }
    }
}

void visualization::dispMkarr(octomap::OcTree *octree, visualization_msgs::MarkerArray &marker_arr,std_msgs::ColorRGBA &color, const ros::Time &rostime, std::string frame, unsigned int depth)
{
    double depth_size = octree->getNodeSize(depth);
    marker_arr.markers[0].header.frame_id = frame;
    marker_arr.markers[0].header.stamp = rostime;
    marker_arr.markers[0].ns = "map";
    marker_arr.markers[0].id = 0;
    marker_arr.markers[0].type = visualization_msgs::Marker::CUBE_LIST;
    marker_arr.markers[0].scale.x = depth_size;
    marker_arr.markers[0].scale.y = depth_size;
    marker_arr.markers[0].scale.z = depth_size;
    marker_arr.markers[0].color = color;
    marker_arr.markers[0].lifetime = ros::Duration();
    if (marker_arr.markers[0].points.size() > 0) {
        marker_arr.markers[0].action = visualization_msgs::Marker::ADD;
    }
    else {
        marker_arr.markers[0].action = visualization_msgs::Marker::DELETE;
    }
}

void visualization::dispMkarr(visualization_msgs::MarkerArray &marker_arr, uint8_t &shape, std_msgs::ColorRGBA &color, geometry_msgs::Vector3 &scale, std::string &frame)
{
    for (int i = 0; i < marker_arr.markers.size(); i++) {
        marker_arr.markers[i].header.frame_id = frame;
        marker_arr.markers[i].header.stamp = ros::Time::now();
        marker_arr.markers[i].ns = "map";
        marker_arr.markers[i].id = i;
        marker_arr.markers[i].type = shape;
        marker_arr.markers[i].scale = scale;
        marker_arr.markers[i].color = color;
        marker_arr.markers[i].lifetime = ros::Duration();
        marker_arr.markers[i].action = visualization_msgs::Marker::ADD;
    }
}

void visualization::dispPoint(visualization_msgs::Marker &marker, std_msgs::ColorRGBA &color, geometry_msgs::Vector3 &scale, std::string &frame)
{
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "map";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale = scale;
    marker.color = color;
}

