/***********************************************************
 * Introduction: Calculate next best view through different
 *               strategy
 **********************************************************/

#include "ros/ros.h"

#include "exploration_baxter/MPmap_filter.h"

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
typedef std::vector<std::pair<octomap::point3d, octomap::point3d> > FrtNb;
typedef std::vector<MPmapFilter::frt_prp> FrtPrp;
typedef std::vector<geometry_msgs::Pose> views;
typedef std::vector<std::pair<octomap::point3d, geometry_msgs::PoseStamped> > viewsStamped;

class NBVStrategy {

    /* ros nodehandle */
    ros::NodeHandle m_nh;

    /* World reference frame */
    std::string m_WorldFrame;
    /* Visualization makers for normals */
    visualization_msgs::MarkerArray normalVis;
    /* Publisher of visual normals */
    ros::Publisher normal_pub;
    /* End effector of plan instead of camera_rgb_optical_frame*/
    std::string plan_eef;
    /* Transformation of sensor frame relative to right hand camera frame */
    Eigen::Affine3d TsensorTorhc;
    /* Information gain score for each area */
    double score_one, score_two, score_three;
    /* Information gain weight */
    double weight_ig;
    /* Continuity weight */
    double weight_cont;
    /* S1 position */
    octomap::point3d s1_pos;
    /* Move ranges of areaone and areatwo */
    double range_one, range_two;
    /* For random */
    unsigned next1, next2;
    /* Publisher of frontier point to be observed */
    ros::Publisher frtpt_pub;

    /**
     * @brief Sort frontier points by distance to sensor origin from minimum to maximum
     * @input Frontier points with distance (pairs)
     */
    void doSort(DistPoints &distpoints);

    /**
     * @brief Sort frontier properties struct by distance to sensor origin
     * @input Frontier properties struct
     */
    void sortFrtprp(FrtPrp &frtprp);

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
        bool operator()(MPmapFilter::frt_prp left, MPmapFilter::frt_prp right)
        {
            return (left.sensor_frt < right.sensor_frt);
        }
    };
    /* A functor of comparator for select maximum distance */
    struct maxmin_dist {
        const NBVStrategy& m_strategy;
        maxmin_dist(const NBVStrategy& strategy) : m_strategy(strategy) { }
        bool operator()(MPmapFilter::frt_prp &left, MPmapFilter::frt_prp right)
        {
            return left.sensor_frt < right.sensor_frt;
        }
    };
    /* A functor of comparator for sorting average score */
    struct comp_score {
        const NBVStrategy& m_strategy;
        comp_score(const NBVStrategy& strategy) : m_strategy(strategy) { }
        bool operator()(MPmapFilter::frt_prp left, MPmapFilter::frt_prp right)
        {
            return (left.avg_score > right.avg_score);
        }
    };

    /* Find neighbor function */
    struct find_nb{
        const NBVStrategy& m_strategy;
        octomap::point3d frontier_;
        find_nb(const NBVStrategy& strategy, const octomap::point3d& frontier):
            m_strategy(strategy),
            frontier_(frontier)
        { }
        bool operator()(std::pair<octomap::point3d, octomap::point3d> const& p)
        {
            return (p.first == frontier_);
        }
    };

    /*
     * @brief Assign average score to each frontier
     * @input frontier properties group
     * @output
     */
    void ScoreAssign(FrtPrp &frtprps);

    /*
     * @brief Pick point on a unit sphere randomly
     */
    Eigen::Vector3f RandomPointSphere();

    Eigen::Vector3f RandomPointHemisphere(Eigen::Vector3f const &v);

    unsigned my_rand(void);

    float U_m1_p1();

    public:

    /* Next best view candidates including position and orientation */
    //views nbvCands;
    /* Modify type of nbvCands */
    viewsStamped nbvCands;
    /* Next best view candidates including position and orientation for right hand camera */
    viewsStamped RHCnbvCands;

    NBVStrategy(ros::NodeHandle nh);

    /* Lower limit of sensor in meter */
    double lower_limit;

    /**
     * @brief Strategy to track the nearest frontier point
     * @input Frontier pointcloud; frontier points with distance to sensor origin; number of candidates; sensor origin; Top portion of frontier points considered to be candidates
     * @output Next view candidates including position and orientation
     */
    void FrtNearTracker(PointCloud &cloud, DistPoints &distpoints, int &cand_num, octomap::point3d origin, geometry_msgs::Pose last_view, float portion, FrtNb frt_nb);

    /**
     * @brief Strategy to track the frontier considering distance from frontier to sensor origin and distance from frontier to base
     * @input Frontier pointcloud to extract normal
     *        Frontier properties struct in three areas
     *
     * @output
     */
    void ManeuverContinue(PointCloud &cloud, FrtPrp &areaone, FrtPrp &areatwo, FrtPrp &areathree, int &cand_num, octomap::point3d origin, float portion);

    /**
     * @brief Exploration strategy based on weighted function of continuity and information gain in C space
     * @input
     * @output
     */
    void ContInfogain(PointCloud &cloud, FrtPrp &frtprps, int &cand_num, octomap::point3d origin);

    /**
     * @brief Exploration strategy based on weighted function of continuity and information gain in C space
     *        For each candidate frontier, this function generates K candidate poses to observe it
     * @input
     * @output
     */
    void ContInfogainMulCands(PointCloud &cloud, FrtPrp &frtprps, int &cand_num, octomap::point3d origin);
};
