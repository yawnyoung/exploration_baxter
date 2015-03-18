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
} // namespace visualization
