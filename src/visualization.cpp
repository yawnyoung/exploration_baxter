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
