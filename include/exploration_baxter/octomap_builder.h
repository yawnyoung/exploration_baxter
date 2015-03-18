/*****************************************************************
 * Introduction: Build octomap from pointcloud plus pre-defined
 *               free zone.
 ***************************************************************/

#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/OcTreeLUT.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include "exploration_baxter/visualization.h"

class OctomapBuilder {
    public:
        typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
        /* Octree */
        octomap::OcTree *m_octree;
    private:
    ros::NodeHandle m_nh;
    /* Octree resolution */
    double m_res;
    /* Pre-defined free zone bounding box axis-aligned coordinates*/
    octomap::point3d m_freebbxmin, m_freebbxmax;
    /* Pre-defined free zone bounding box keys */
    octomap::OcTreeKey m_freebbxminkey, m_freebbxmaxkey;
    /* Minimum and maximum values along each axis */
    double m_pointcloudMinX, m_pointcloudMaxX, m_pointcloudMinY, m_pointcloudMaxY, m_pointcloudMinZ, m_pointcloudMaxZ;
    /* The maximum range for each ray */
    double m_maxRange;
    /* Temp storage for ray casting */
    octomap::KeyRay m_keyRay;
    /* Filter out speckles */
    bool m_filterSpeckles;
    /* Octree depth */
    unsigned m_treeDepth;
    /* Hit probability */
    double m_probHit;
    /* Miss probability */
    double m_probMiss;
    /* Threshold for minimum probability */
    double m_thresMin;
    /* Threshold for maximum probability */
    double m_thresMax;

    /* Message filter subscriber subscribing to pointcloud2 */
    message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub;
    /* Subscribed pointcloud2 topic name */
    std::string m_pointCloudTopic;
    /* TF message filter to process subscribed pointcloud2 */
    tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub;
    /* The map frame */
    std::string m_worldFrameId;
    /* TF listener */
    tf::TransformListener m_tfListener;

    /* Neighbor directions for checking speckles */
    std::vector<octomap::OcTreeLUT::NeighborDirection> nb_dir;

    /* Publisher of free and occupied markers */
    ros::Publisher free_pub, occ_pub;
    /* Colors for free and occupied markers */
    std_msgs::ColorRGBA free_color, occ_color;

    virtual void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    virtual void specklesfilter();

    /**
     * @brief Update occupancy map with pointcloud scan.
     * The scans should be in the global map frame.
     *
     * @param sensorOrigin origin of the measurements for raytracing
     * @param input_pc the pointcloud as input
     */
    virtual void insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& input_pc);
    void publishAll(const ros::Time& rostime);

    public:
        OctomapBuilder(ros::NodeHandle nh, octomap::point3d freebbxmin, octomap::point3d freebbxmax);
};