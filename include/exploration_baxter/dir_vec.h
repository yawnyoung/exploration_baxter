
/**************************************************************************************************
 * Functions: To calculate the direction vector based on a octomap.
 *
 * Definition:
 *      Direction vector: This vector directs to area with most unknown nodes in a certain
 *                        bounding box.
 *                        This vector is used to indicate the direction the sensor should
 *                        go along. (It's different from the orientation of the sensor at next
 *                        best view)
**************************************************************************************************/
#include "ros/ros.h"

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>

#include <tf/transform_listener.h>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <numeric>

using namespace octomap;

class CalDir {

    ros::NodeHandle nh_;
    ros::Publisher occ_pub, free_pub;
    ros::Publisher org_pub;
    // bounding box
    point3d bbxmin_;
    point3d bbxmax_;

    // vector of direction from base to each unknown node.
    //std::vector<point3d> vects_;
    // average direction vector
    point3d dir_vec;

    OcTree *ot_map;
    int m_treeDepth;
    // markers for visualization
    visualization_msgs::MarkerArray occNodesVis, freeNodesVis;

    // listen the transformation of /camera_rgb_optical_frame to /base
    tf::TransformListener listener;
    tf::StampedTransform transform;

    public:
        CalDir(point3d bbxmin, point3d bbxmax, ros::NodeHandle n);

        /**********************************************************************
         * Function: Calculate the direction vector based on the given octomap
         * Input: octomap
         * Output: direction vector (the vector is relative to base frame)
         *********************************************************************/
        point3d cal_dirvec_base(OcTree *ot);

        // The same function as "cal_dirvec_base" except that the vector is relative to sensor frame
        point3d cal_dirvec_sensor(OcTree *ot);

        // function for visualization
        void cell_vis();

        tf::Vector3 sensor_origin;
};

CalDir::CalDir(point3d bbxmin, point3d bbxmax, ros::NodeHandle n):
    bbxmin_(bbxmin),
    bbxmax_(bbxmax),
    nh_(n)
{
    ros::Time begin = ros::Time::now();
    ROS_INFO("ENTER CONSTRUCTION");
    occ_pub = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells", 1);
    free_pub = nh_.advertise<visualization_msgs::MarkerArray>("free_cells", 1);
    org_pub = nh_.advertise<visualization_msgs::Marker>("origin", 1);
    listener.waitForTransform("/base", "/camera_rgb_optical_frame", ros::Time::now(),ros::Duration(0.5));
    // check whether frame exists
    std::cout << listener.frameExists("base") << std::endl;
    ros::Duration elapsed_time = ros::Time::now() - begin;
    ROS_INFO_STREAM("Time elapsed: " << elapsed_time << " sec.");
    ROS_INFO("EXIT CONSTRUCTION");
}

point3d CalDir::cal_dirvec_base(OcTree *ot)
{
    ot_map = ot;
    m_treeDepth = ot_map->getTreeDepth();
    // calculate the range for each axis direction
    OcTreeKey bbxminkey = ot_map->coordToKey(bbxmin_);
    OcTreeKey bbxmaxkey = ot_map->coordToKey(bbxmax_);
    int sizeX = bbxmaxkey[0] - bbxminkey[0];
    int sizeY = bbxmaxkey[1] - bbxminkey[1];
    int sizeZ = bbxmaxkey[2] - bbxminkey[2];
    OcTreeKey bbxkey;

    std::vector<point3d> vects_;
    // record every unknown node
    for (int dx = 0; dx < sizeX; ++dx)
    {
        bbxkey[0] = bbxminkey[0] + dx;
        for (int dy = 0; dy < sizeY; ++dy)
        {
            bbxkey[1] = bbxminkey[1] + dy;
            for (int dz = 0; dz < sizeZ; ++dz)
            {
                bbxkey[2] = bbxminkey[2] + dz;
                OcTreeNode *node = ot_map->search(bbxkey);
                if (!node) {
                    vects_.push_back(ot_map->keyToCoord(bbxkey));

                }
            }
        }
    }
    // sum of direction vectors
    point3d init(0, 0, 0);
    point3d sum_vec(0, 0, 0);
    //point3d sum_vec = std::accumulate(vects_.begin(), vects_.end(), init);
    for (std::vector<point3d>::iterator iter = vects_.begin(); iter != vects_.end(); ++iter)
    {
        sum_vec.x() += (*iter).x();
        sum_vec.y() += (*iter).y();
        sum_vec.z() += (*iter).z();
    }

    // average of sum
    unsigned int v_size = vects_.size();
    std::cout << "unknown nodes: " << v_size << std::endl;
    point3d avg_vec(sum_vec.x()/v_size, sum_vec.y()/v_size, sum_vec.z()/v_size);
    point3d norm_vec = avg_vec.normalize();

    return norm_vec;
}

point3d CalDir::cal_dirvec_sensor(OcTree *ot)
{
    try {
        ROS_INFO("here");
        listener.lookupTransform("/base", "/camera_rgb_optical_frame", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
    sensor_origin = transform.getOrigin();
    visualization_msgs::Marker origin_m;
    origin_m.header.frame_id = "/base";
    origin_m.header.stamp = ros::Time::now();
    origin_m.ns = "origin";
    origin_m.id = 0;
    origin_m.type = visualization_msgs::Marker::ARROW;
    origin_m.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point start,end;
    start.x = 0;
    start.y = 0;
    start.z = 0;
    end.x = sensor_origin.x();
    end.y = sensor_origin.y();
    end.z = sensor_origin.z();
    origin_m.points.push_back(start);
    origin_m.points.push_back(end);
    origin_m.scale.x = 0.005;
    origin_m.scale.y = 0.005;
    origin_m.scale.z = 0.005;
    origin_m.color.r = 1;
    origin_m.color.g = 0;
    origin_m.color.b = 0;
    origin_m.color.a = 1;
    origin_m.lifetime = ros::Duration();
    org_pub.publish(origin_m);
    ot_map = ot;
    m_treeDepth = ot_map->getTreeDepth();

    // calculate the range for each axis direction
    OcTreeKey bbxminkey = ot_map->coordToKey(bbxmin_);
    OcTreeKey bbxmaxkey = ot_map->coordToKey(bbxmax_);
    int sizeX = bbxmaxkey[0] - bbxminkey[0];
    int sizeY = bbxmaxkey[1] - bbxminkey[1];
    int sizeZ = bbxmaxkey[2] - bbxminkey[2];
    OcTreeKey bbxkey;

    std::vector<point3d> vects_;
    // record every unknown node
    for (int dx = 0; dx < sizeX; ++dx)
    {
        bbxkey[0] = bbxminkey[0] + dx;
        for (int dy = 0; dy < sizeY; ++dy)
        {
            bbxkey[1] = bbxminkey[1] + dy;
            for (int dz = 0; dz < sizeZ; ++dz)
            {
                bbxkey[2] = bbxminkey[2] + dz;
                OcTreeNode *node = ot_map->search(bbxkey);
                if (!node) {
                    point3d vec_base = ot_map->keyToCoord(bbxkey);

                    /*point3d vec_sensor;
                    vec_sensor.x() = vec_base.x() - sensor_origin.x();
                    vec_sensor.y() = vec_base.y() - sensor_origin.y();
                    vec_sensor.z() = vec_base.z() - sensor_origin.z();*/

                    // transform vector from base frame to sensor origin frame
                    tf::Vector3 pt_tf = transform.inverse()*octomap::pointOctomapToTf((ot_map->keyToCoord(bbxkey)));
                    vects_.push_back(octomap::pointTfToOctomap(pt_tf));
                }
            }
        }
    }

    // sum of direction vectors
    point3d init(0, 0, 0);
    //point3d sum_vec(0, 0, 0);
    point3d sum_vec = std::accumulate(vects_.begin(), vects_.end(), init);
    /*
    for (std::vector<point3d>::iterator iter = vects_.begin(); iter != vects_.end(); ++iter)
    {
        sum_vec.x() += (*iter).x();
        sum_vec.y() += (*iter).y();
        sum_vec.z() += (*iter).z();
    }*/

    // average of sum
    unsigned int v_size = vects_.size();
    std::cout << "unknown nodes: " << v_size << std::endl;
    point3d avg_vec(sum_vec.x()/v_size, sum_vec.y()/v_size, sum_vec.z()/v_size);
    /*
    point3d vec_sensor;
    vec_sensor.x() = avg_vec.x() - sensor_origin.x();
    vec_sensor.y() = avg_vec.y() - sensor_origin.y();
    vec_sensor.z() = avg_vec.z() - sensor_origin.z();*/
    //point3d norm_vec = avg_vec.normalize();

    return avg_vec;

}
void CalDir::cell_vis()
{
    occNodesVis.markers.resize(m_treeDepth+1);
    freeNodesVis.markers.resize(m_treeDepth+1);
    for (OcTree::leaf_bbx_iterator it = ot_map->begin_leafs_bbx(bbxmin_, bbxmax_); it != ot_map->end_leafs_bbx(); ++it)
    {
        unsigned int idx = it.getDepth();
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        geometry_msgs::Point cubeCenter;
        cubeCenter.x = x;
        cubeCenter.y = y;
        cubeCenter.z = z;

        if (ot_map->isNodeOccupied(*it)) {
            occNodesVis.markers[idx].points.push_back(cubeCenter);
        }
        else {
            freeNodesVis.markers[idx].points.push_back(cubeCenter);
        }
    }
        for (unsigned i = 0; i < occNodesVis.markers.size(); ++i)
        {
            double size = ot_map->getNodeSize(i);
            occNodesVis.markers[i].header.frame_id = "/base";
            occNodesVis.markers[i].header.stamp = ros::Time::now();
            occNodesVis.markers[i].ns = "occ_map";
            occNodesVis.markers[i].id = i;
            occNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            occNodesVis.markers[i].scale.x = size;
            occNodesVis.markers[i].scale.y = size;
            occNodesVis.markers[i].scale.z = size;
            occNodesVis.markers[i].color.r = 1;
            occNodesVis.markers[i].color.g = 0;
            occNodesVis.markers[i].color.b = 0;
            occNodesVis.markers[i].color.a = 1;
            occNodesVis.markers[i].lifetime = ros::Duration();
            if (occNodesVis.markers[i].points.size() > 0) {
                occNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
            }
            else {
                occNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
            }
        }
        for (unsigned i = 0; i < freeNodesVis.markers.size(); ++i)
        {
            double size = ot_map->getNodeSize(i);
            freeNodesVis.markers[i].header.frame_id = "/base";
            freeNodesVis.markers[i].header.stamp = ros::Time::now();
            freeNodesVis.markers[i].ns = "free_map";
            freeNodesVis.markers[i].id = i;
            freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            freeNodesVis.markers[i].scale.x = size;
            freeNodesVis.markers[i].scale.y = size;
            freeNodesVis.markers[i].scale.z = size;
            freeNodesVis.markers[i].color.r = 0;
            freeNodesVis.markers[i].color.g = 1;
            freeNodesVis.markers[i].color.b = 0;
            freeNodesVis.markers[i].color.a = 0.2;
            freeNodesVis.markers[i].lifetime = ros::Duration();
            if (freeNodesVis.markers[i].points.size() > 0) {
                freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
            }
            else {
                freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
            }
        }

    occ_pub.publish(occNodesVis);
    free_pub.publish(freeNodesVis);
}

