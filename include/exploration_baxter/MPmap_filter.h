/************************************************************
 * Introduction: Octomap filter for motion planning
 ***********************************************************/

#include "ros/ros.h"

#include <octomap/octomap.h>
#include <octomap/OcTreeLUT.h>
#include <octomap_ros/conversions.h>

#include "exploration_baxter/visualization.h"

using namespace octomap;

class MPmapFilter {
    /* NodeHandle */
    ros::NodeHandle mp_nh;
    /* Depth of octree */
    unsigned mp_treeDepth;
    /* Neighbor directions to detect */
    std::vector<OcTreeLUT::NeighborDirection> nb_dir;
    /* Visualization markers for frontier cells */
    visualization_msgs::MarkerArray frtNodesVis;
    /* Publisher of frontier cell visualization */
    ros::Publisher frt_pub;
    /* Frontier cell color */
    std_msgs::ColorRGBA frt_color;
    /* Flag to indicate whether publish frontier or not */
    bool isPublishFrt;
    /* World frame */
    std::string mp_WorldFrameId;

    void mpPublishAll(const ros::Time &rostime);
    public:
    /* Octree */
    OcTree *mp_octree;
    /* Cells labeled by frontier including occupied cells and free cells */
    std::vector<OcTreeKey> frontier_cells;

    MPmapFilter(ros::NodeHandle nh);
    /**
     * @brief Extract frontier of octree for each call
     * @input octree to be operated
     * @output octree operated, cell set labeled by frontier
     */
    void frontierExtraction(OcTree *octree);

};
