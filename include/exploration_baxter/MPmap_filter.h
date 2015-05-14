/************************************************************
 * Introduction: Octomap filter for motion planning
 *               Extract frontier
 ***********************************************************/

#include "ros/ros.h"

#include <algorithm>

#include <octomap/octomap.h>
#include <octomap/OcTreeLUT.h>
#include <octomap_ros/conversions.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "exploration_baxter/visualization.h"

using namespace octomap;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class MPmapFilter {
    /* NodeHandle */
    ros::NodeHandle mp_nh;
    /* Depth of octree */
    unsigned mp_treeDepth;
    /* Neighbor directions to detect */
    std::vector<OcTreeLUT::NeighborDirection> nb_dir;
    /* Visualization markers for frontier cells and modified frontier cells */
    visualization_msgs::MarkerArray frtNodesVis;
    /* Publisher of frontier cell visualization */
    ros::Publisher frt_pub;
    /* Frontier cell color */
    std_msgs::ColorRGBA frt_color;
    /* Flag to indicate whether publish frontier or not */
    bool isPublishFrt;
    /* World frame */
    std::string mp_WorldFrameId;
    /* Octree key of bounding box for the whole octomap */
    OcTreeKey nbv_bbxminkey, nbv_bbxmaxkey;
    /* Publisher of frontier pointcloud */
    ros::Publisher frtPc_pub;
    /* Publisher of real occupied pointcloud */
    ros::Publisher occPc_pub;
    /* Publisher of void-frontier pointcloud */
    ros::Publisher voidfrtPc_pub;
    /* Radius for void-frontier search */
    double voidfrt_radius;
    /* S1 position */
    point3d s1_pos;
    /* Move range 1 */
    double range_one;
    /* Move range 2 */
    double range_two;
   /* A functor of comparator */
    struct comp_crit {
        const MPmapFilter& m_mapFilter;
        comp_crit( const MPmapFilter& mapFilter) : m_mapFilter(mapFilter) { }
        bool operator()(std::pair<point3d, double> left, std::pair<point3d, double> right)
        {
            return (left.second < right.second);
        }
    };
    /**
     * @brief Publish visualization markers
     * @input time stamp
     */
    void mpPublishAll(const ros::Time &rostime);
    /**
     * @brief Sort
     */
    void doSort();

    /**
     * @brief Extract void-frontier
     */
    void voidfrtExtraction();

    public:
    OcTree *mp_octree;
    /* Cells labeled by frontier including occupied cells and free cells */
    std::vector<OcTreeKey> frontier_cells;
    /* Candidate positions */
    std::vector<geometry_msgs::Point> cand_positions;
    /* A set of pairs including frontier point and its corresponding criterion function value */
    std::vector<std::pair<point3d, double> > cand_pair;
    /* A set of pairs including frontier point and its unknown neighbor */
    std::vector<std::pair<point3d, point3d> > frt_unknown;

    /* Node number in three areas */
    unsigned int num_areaone;
    unsigned int num_areatwo;
    unsigned int num_areathree;

    /* Frontier properties struct */
    struct frt_prp {
        point3d frt_node;           // frontier
        point3d unk_nb;             // unknown neighbor to frontier
        double sensor_frt;          // distance from sensor origin to frontier
        bool isvoid;                // flag indicates whether this frontier point is void frontier point
        double avg_score;             // average score
    };

    /* A set of frontier properties structs */
    std::vector<frt_prp> frt_areaone, frt_areatwo, frt_areathree, frt_group;
    /* Frontier pointcloud */
    PointCloud frt_pc;
    /* Real occupied pointcloud */
    PointCloud occ_pc;
    /* Void-frontier pointcloud */
    PointCloud voidfrt_pc;
    /* Proportion of void-frontier points to frontier points */
    double voidfrt_frt;
    /* Flag indicates whether the last frontier was observed */
    bool frt_observed;

    MPmapFilter(ros::NodeHandle nh, OcTreeKey bbxminkey, OcTreeKey bbxmaxkey);
    /**
     * @brief Extract frontier of octree for each call
     * @input octree to be operated
     * @output octree operated, cell set labeled by frontier
     */
    void frontierExtraction(OcTree *octree);
    /**
     * @brief Calculate NBV candidates based on frontier cells considering the lower limit of sensor
     * @input NBV octree, origin of sensor, last frontier
     * @output A set of candidates
     */
    void FrtNbvCandidates(OcTree *octree, point3d &origin, point3d &last_frontier);
    /**
     * @brief One of criterion functions: Calculate the distance between frontier point and sensor origin
     * @input frontier point, sensor origin
     * @output distance between frontier point and sensor origin
     */
    double Dist_FrtOrg(point3d &frt, point3d &org);

    /**
     * @brief Extract void frontier for the whole frontier properties group
     */
    void VoidfrtgroupExtract();

};
