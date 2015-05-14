/****************************************************
 * Introduction: Visualization functions for octomap
 ***************************************************/

#include "ros/ros.h"
#include <octomap/octomap.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

namespace visualization {
    /**
     * @brief Visualize markerarray
     * @input octree to hold the markers
     *        markerarray
     *        color of markerarray
     *        time stamp
     *        frame stamp
     */
    void dispMkarr(octomap::OcTree *octree, visualization_msgs::MarkerArray &marker_arr, std_msgs::ColorRGBA &color, const ros::Time &rostime, std::string frame);
    void dispMkarr(octomap::OcTree octree, visualization_msgs::MarkerArray &marker_arr,std_msgs::ColorRGBA &color, const ros::Time &rostime, std::string frame);
    /**
     * @brief Visualize markerarray of one certain depth
     * @input octree to hold the markers, markerarray, color of markerarray, time stamp, frame stamp, depth
     */
    void dispMkarr(octomap::OcTree *octree, visualization_msgs::MarkerArray &marker_arr,std_msgs::ColorRGBA &color, const ros::Time &rostime, std::string frame, unsigned int depth);
    /**
     * @brief Visualize markers with 1 size
     */
    void dispMkarr(visualization_msgs::MarkerArray &marker_arr, uint8_t &shape, std_msgs::ColorRGBA &color, geometry_msgs::Vector3 &scale, std::string &frame);

    /**
     * @brief Visualize points
     */
    void dispPoint(visualization_msgs::Marker &marker, std_msgs::ColorRGBA &color, geometry_msgs::Vector3 &scale, std::string &frame);
} // namespace visualization
