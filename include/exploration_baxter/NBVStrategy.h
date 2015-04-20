/***********************************************************
 * Introduction: Calculate next best view through different
 *               strategy
 **********************************************************/

#include "ros/ros.h"

#include <octomap/octomap.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <octomap_ros/conversions.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <tf/transform_listener.h>

#include "exploration_baxter/visualization.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef std::vector<std::pair<octomap::point3d, double> > DistPoints;
typedef std::vector<geometry_msgs::Pose> views;
typedef std::vector<geometry_msgs::PoseStamped> viewsStamped;

class NBVStrategy {

    /* ros nodehandle */
    ros::NodeHandle m_nh;
    /* Lower limit of sensor in meter */
    double lower_limit;
    /* World reference frame */
    std::string m_WorldFrame;
    /* Visualization makers for normals */
    visualization_msgs::MarkerArray normalVis;
    /* Publisher of visual normals */
    ros::Publisher normal_pub;
    /* Transformation of sensor frame relative to right hand camera frame */
    Eigen::Affine3d TsensorTorhc;

    /**
     * @brief Sort frontier points by distance to sensor origin from minimum to maximum
     * @input Frontier points with distance (pairs)
     */
    void doSort(DistPoints &distpoints);

    /**
     * @brief Calculate desired pose for right hand camera based on desired pose for sensor
     */
    void RHCPose();

    /* A functor of comparator for sort function */
    struct comp_dist {
        const NBVStrategy& m_strategy;
        comp_dist(const NBVStrategy& strategy) : m_strategy(strategy) { }
        bool operator()(std::pair<octomap::point3d, double> left, std::pair<octomap::point3d, double> right)
        {
            return (left.second < right.second);
        }
    };

    public:

        /* Next best view candidates including position and orientation */
        //views nbvCands;
        /* Modify type of nbvCands */
        viewsStamped nbvCands;
        /* Next best view candidates including position and orientation for right hand camera */
        viewsStamped RHCnbvCands;

        NBVStrategy(ros::NodeHandle nh);

        /**
         * @brief Strategy to track the nearest frontier point
         * @input Frontier pointcloud; frontier points with distance to sensor origin; number of candidates; sensor origin; Top portion of frontier points considered to be candidates
         * @output Next view candidates including position and orientation
         */
        void FrtNearTracker(PointCloud &cloud, DistPoints &distpoints, int &cand_num, octomap::point3d origin, geometry_msgs::Pose last_view, float portion);
};
