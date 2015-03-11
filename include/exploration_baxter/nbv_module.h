
/***********************************************************************************
 * Introduction: This module provides next best view candidates based on an octomap
 *               in certain range.
 *
 * Method:
 *
 * Functions: Calculate direction vectors;
 *            Calculate frontiers;
 *            Calculate intersection point;
 *            Calculate the nearest frontier to sensor;
 *            Calculate next best view candidates;
 *
 *
 * ********************************************************************************/

#include "ros/ros.h"

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap/OcTreeLUT.h>

#include <tf/transform_listener.h>

#include <numeric>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

using namespace octomap;

class NBVModule {

    // Ros NodeHandle
    ros::NodeHandle nh_;
    // Octree
    OcTree *ot_nbv;
    // Tree max depth
    int m_treeDepth;

    /**
     * Calculate boundary keys and sizes along each axis for a bounding box.
     */
    bool bbx_func();
    point3d bbxmin_, bbxmax_, freebbxmin_, freebbxmax_, obsrvbbxmin_, obsrvbbxmax_, dispmin_, dispmax_;       // min and max coordinates for a bounding box including free_bxx and obsrv_bbx
    OcTreeKey bbxminkey, bbxmaxkey, freebbxminkey, freebbxmaxkey, obsrvbbxminkey, obsrvbbxmaxkey;     // min and max keys for a bounding box
    int sizeX, sizeY, sizeZ;        // key sizes along each axis for a bounding box

    // Data members for updating nbv module
    tf::TransformListener bs_listener;      // tf listener
    tf::StampedTransform bs_transform;      // transform of sensor frame relative to base frame

    // Data members for calculating direction vectors
    point3d dir_vec;                    // direction vector in camera_rgb_optical_frame
    point3d dir_vec_base;               // direction vector in base frame
    std::vector<point3d> vects_;        // vector of points directing to unknown nodes
    geometry_msgs::Point start_dirvec;  // start point of display direction vector
    ros::Publisher dispdir_pub;         // publisher of display direction vector

    // Data members for extracting frontiers
    std::vector<point3d> points_frt;
    visualization_msgs::MarkerArray frtNodesVis;        // frontier markers for visualization
    ros::Publisher frt_pub;     // publisher of display frontier markers
    visualization_msgs::MarkerArray occNodesVis;      // markers of occupied nodes for visualization
    ros::Publisher occ_pub;       // publisher of display occupied markers
    visualization_msgs::MarkerArray freeNodesVis;       // markers of free nodes for visualization
    ros::Publisher free_pub;        // publisher of display free markers
    visualization_msgs::MarkerArray unknownNodesVis;    // markers of unknown nodes for visualization
    ros::Publisher unknown_pub;     // publisher of display unknown markers

    // Data members for calculating intersection of direction vector and frontier
    double dists;               // distance from frontier point to direction vector
    point3d pt_intsct;          // intersection point
    visualization_msgs::Marker intersect;       // markers of intersection point
    ros::Publisher intersect_pub;                // publisher of intersection marker

    // Data members for calculating the nearest frontier point to sensor origin
    point3d near_frt;

    // Data members for calculating next best view candidates
    std::vector<point3d> candidates;

    public:
        // Number of unknown nodes
        unsigned int num_unknown;

        /**
         * Constructor function
         * @param input boundary coordinates, octree
         */
        NBVModule(point3d obsrvbbxmin, point3d obsrvbbxmax, point3d freebbxmin, point3d freebbxmax, OcTree *ot, ros::NodeHandle n);
        /**
         * Update NBV octomap and transformation between 'camera_rgb_optical_frame' and 'base'
         */
        bool update_nbv(OcTree *ot);

        /**
         * Traverse octree
         * @param input octree
         */
        bool trvs_tree();

        /**
         * Calculate direction vector
         * @param boolean parameter to indicate whether or not display direction vector
         */
        void cal_dirvec(bool &disp_vec);

        /**
         * Extract frontier
         */
        bool extr_frontier();

        /**
         * Explore boundary along each axis
         */
        //void explr_bd(OcTreeNode *frt_node, std::vector<int> no_child, std::vector<int> noch_rev);
        //void explr_bd(OcTree::tree_iterator tree_it, std::vector<int> no_child, std::vector<int> noch_rev);
        void explr_bd();

        /**
         * Set configuration of markerarray to be displayed
         * @param input marker_array to be displayed, color set of this marker array
         */
        void disp_mkarr(visualization_msgs::MarkerArray &marker_arr, std_msgs::ColorRGBA &color);

        /**
         * Expand leaf node in bounding box recursively
         * @param leaf iterator whose node is going to be expanded
         */
        void expandbbxRecurs();

        /**
         * Calculate the intersection of direction vector and frontier
         */
        void intersection();

        /**
         * Calculate the nearest frontier points to sensor
         */
        void nearestfrt();

        /**
         * Calculate the next best view candidates on spherical surface
         */
        void nbvcandidates();
};

NBVModule::NBVModule(point3d obsrvbbxmin, point3d obsrvbbxmax, point3d freebbxmin, point3d freebbxmax, OcTree *ot, ros::NodeHandle n):
    nh_(n),
    //bbxmin_(bbxmin),
    //bbxmax_(bbxmax),
    freebbxmin_(freebbxmin),
    freebbxmax_(freebbxmax),
    obsrvbbxmin_(obsrvbbxmin),
    obsrvbbxmax_(obsrvbbxmax),
    ot_nbv(ot)
{
    // Calculate the whole bounding box
    bbxmin_.x() = std::min(freebbxmin_.x(), obsrvbbxmin_.x());
    bbxmin_.y() = std::min(freebbxmin_.y(), obsrvbbxmin_.y());
    bbxmin_.z() = std::min(freebbxmin_.z(), obsrvbbxmin_.z());

    bbxmax_.x() = std::max(freebbxmax_.x(), obsrvbbxmax_.x());
    bbxmax_.y() = std::max(freebbxmax_.y(), obsrvbbxmax_.y());
    bbxmax_.z() = std::max(freebbxmax_.z(), obsrvbbxmax_.z());

    dispmin_ = bbxmin_;
    dispmax_ = bbxmax_;
    dispmax_.y() -= 0.5;

    // Get tree depth
    m_treeDepth = ot_nbv->getTreeDepth();

    // Display configuration for direction vector
    start_dirvec.x = 0;
    start_dirvec.y = 0;
    start_dirvec.z = 0;
    dispdir_pub = nh_.advertise<visualization_msgs::Marker>("marker_dirvec", 1);

    // Display configuration for frontier nodes
    frt_pub = nh_.advertise<visualization_msgs::MarkerArray>("marker_frontier", 1);
    frtNodesVis.markers.resize(m_treeDepth+1);

    // Display configuration for occupied nodes
    occ_pub = nh_.advertise<visualization_msgs::MarkerArray>("marker_occ", 1);
    occNodesVis.markers.resize(m_treeDepth+1);

    // Display configuration for free nodes
    free_pub = nh_.advertise<visualization_msgs::MarkerArray>("marker_free", 1);
    freeNodesVis.markers.resize(m_treeDepth+1);

    // Display configuration for unknown nodes
    unknown_pub = nh_.advertise<visualization_msgs::MarkerArray>("marker_unknown", 1);
    unknownNodesVis.markers.resize(1);

    // Display configuration for intersection point
    intersect_pub = nh_.advertise<visualization_msgs::Marker>("intersection", 1);


    if (m_treeDepth == 16) {
        if (!bbx_func()) {
            ROS_ERROR("Fail to construct NBV Module!");
        }
        else {
            ROS_INFO("Construct NBV Module!");
        }
    }

}

bool NBVModule::bbx_func()
{
    bool min = ot_nbv->coordToKeyChecked(bbxmin_, bbxminkey);
    bool max = ot_nbv->coordToKeyChecked(bbxmax_, bbxmaxkey);
    ROS_INFO_STREAM("Min point is in octree: " << min);
    ROS_INFO_STREAM("Max Point is in octree: " << max);
    sizeX = bbxmaxkey[0] - bbxminkey[0] + 1;
    sizeY = bbxmaxkey[1] - bbxminkey[1] + 1;
    sizeZ = bbxmaxkey[2] - bbxminkey[2] + 1;

    if (sizeX < 0 || sizeY < 0 || sizeZ < 0) {
        ROS_DEBUG("Size should be larger than 0. Please check the input bounding box coordinates");
        return false;
    }
    else {
        return true;
    }
}

bool NBVModule::update_nbv(OcTree *ot)
{
    // Update nbv octomap
    ot_nbv = ot;
    // set free zone


    // Update transform
    try {
        bs_listener.waitForTransform("/base", "/camera_rgb_optical_frame", ros::Time::now(), ros::Duration(3.0));
        bs_listener.lookupTransform("/base", "/camera_rgb_optical_frame", ros::Time(0), bs_transform);
        return true;
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

//TODO Another method to calculate number of unknown nodes and direction vector
//total node size - known node size
bool NBVModule::trvs_tree()
{
    // Traverse octree and record unknown nodes as well as frontier nodes
    OcTreeKey bbxkey;       // create key to traverse tree
    OcTreeNode *srch_node;      // searched node
    tf::Vector3 pt_tf;      // vector in 'camera_rgb_optical_frame'

    // set free bounding box to be the box we want to set free nodes in it
    ot_nbv->setBBXMin(freebbxmin_);
    ot_nbv->setBBXMax(freebbxmax_);

    // visualize unknown nodes
    geometry_msgs::Point cubeCenter_unknown;

    for (int dx = 0; dx < sizeX; dx++) {
        bbxkey[0] = bbxminkey[0] + dx;
        for (int dy = 0; dy < sizeY; dy++) {
            bbxkey[1] = bbxminkey[1] + dy;
            for (int dz = 0; dz < sizeZ; dz++) {
                bbxkey[2] = bbxminkey[2] + dz;
                if (ot_nbv->inBBX(bbxkey)) {
                    // update node to be free in free bounding box
                    ot_nbv->updateNode(bbxkey, false, false);
                }
                srch_node = ot_nbv->search(bbxkey);

                if (!srch_node) {
                    // store vectors from sensor origin to unknown nodes in 'camera_rgb_optical_frame'
                    pt_tf = bs_transform.inverse()*pointOctomapToTf(ot_nbv->keyToCoord(bbxkey));
                    vects_.push_back(pointTfToOctomap(pt_tf));

                    // calculate the center of unknown nodes
                    cubeCenter_unknown = pointOctomapToMsg(ot_nbv->keyToCoord(bbxkey));
                    unknownNodesVis.markers[0].points.push_back(cubeCenter_unknown);
                }
            }
        }
    }

    std_msgs::ColorRGBA unknown_color;
    unknown_color.r = 0;
    unknown_color.g = 1;
    unknown_color.b = 1;
    unknown_color.a = 1;
    double size = ot_nbv->getNodeSize(16);
    unknownNodesVis.markers[0].header.frame_id = "/base";
    unknownNodesVis.markers[0].header.stamp = ros::Time::now();
    unknownNodesVis.markers[0].ns = "map";
    unknownNodesVis.markers[0].id = 0;
    unknownNodesVis.markers[0].type = visualization_msgs::Marker::CUBE_LIST;
    unknownNodesVis.markers[0].scale.x = size;
    unknownNodesVis.markers[0].scale.y = size;
    unknownNodesVis.markers[0].scale.z = size;
    unknownNodesVis.markers[0].color = unknown_color;
    unknownNodesVis.markers[0].lifetime = ros::Duration();
    unknownNodesVis.markers[0].action = visualization_msgs::Marker::ADD;

    unknown_pub.publish(unknownNodesVis);
    unknownNodesVis.markers[0].points.clear();

    num_unknown = vects_.size();

    if (num_unknown != 0) {
        ROS_INFO("Unknown nodes still exist.");
        return true;
    }
    else {
        ROS_INFO("No unknown node exists.");
        return false;
    }
}

void NBVModule::cal_dirvec(bool &disp_vec)
{
    point3d init_vec(0, 0, 0);
    point3d sum_vec = std::accumulate(vects_.begin(), vects_.end(), init_vec);
    dir_vec.x() = sum_vec.x() / num_unknown;
    dir_vec.y() = sum_vec.y() / num_unknown;
    dir_vec.z() = sum_vec.z() / num_unknown;
    tf::Vector3 vec_base = bs_transform * pointOctomapToTf(dir_vec);
    dir_vec_base = pointTfToOctomap(vec_base);
    ROS_INFO("Number of unknown nodes: %d", num_unknown);
    vects_.clear();

    if (disp_vec) {
        // Display direction vector
        visualization_msgs::Marker marker_dirvec;
        marker_dirvec.header.frame_id = "/camera_rgb_optical_frame";
        marker_dirvec.header.stamp = ros::Time::now();
        marker_dirvec.ns = "dir_vec";
        marker_dirvec.id = 0;
        marker_dirvec.type = visualization_msgs::Marker::ARROW;
        marker_dirvec.action = visualization_msgs::Marker::ADD;
        marker_dirvec.points.push_back(start_dirvec);
        marker_dirvec.points.push_back(pointOctomapToMsg(dir_vec));
        marker_dirvec.scale.x = 0.005;
        marker_dirvec.scale.y = 0.005;
        marker_dirvec.scale.z = 0.005;
        marker_dirvec.color.r = 0;
        marker_dirvec.color.g = 0;
        marker_dirvec.color.b = 1;
        marker_dirvec.color.a = 1;
        marker_dirvec.lifetime = ros::Duration();
        dispdir_pub.publish(marker_dirvec);
    }
}

/*
void NBVModule::explr_bd(OcTreeNode *frt_node, std::vector<int> no_child, std::vector<int> noch_rev)
{
    std::vector<int> inner_no_child(no_child);
    std::vector<int> inner_noch_rev(noch_rev);
    if (frt_node->hasChildren()) {
        for (int i = 0; i < inner_no_child.size(); i++) {
            if (!frt_node->childExists(inner_no_child[i])) {
                if (frt_node->childExists(inner_noch_rev[i])) {
                    OcTreeNode *frt_node_ch = frt_node->getChild(inner_noch_rev[i]);
                    explr_bd(frt_node_ch, inner_no_child, inner_noch_rev);
                }
            }
            else {
                OcTreeNode *frt_node_ch = frt_node->getChild(inner_no_child[i]);
                explr_bd(frt_node_ch, inner_no_child, inner_noch_rev);
            }
        }
    }
    else {

    }
}*/

/*
void NBVModule::explr_bd(OcTree::tree_iterator tree_it, std::vector<int> no_child, std::vector<int> noch_rev)
{
    std::vector<int> inner_no_child(no_child);
    std::vector<int> inner_noch_rev(noch_rev);
    if (tree_it->hasChildren()) {
        for (int i = 0; i < inner_no_child.size(); i++) {

        }
    }
}*/

void NBVModule::explr_bd()
{
    OcTreeLUT lut(16);      // lookup table for finding neighbors
    OcTreeKey nb_key;       // key of neighbor
    unsigned int idx;       // depth of certain node which leaf iterator refers to
    geometry_msgs::Point cubeCenter_frt;        // center of frontier node
    geometry_msgs::Point cubeCenter_known;      // center of known node

    std::vector<OcTreeLUT::NeighborDirection> nb_dir;       // vector of 18 face and edge neighbor directions
    // face directions
    nb_dir.push_back(OcTreeLUT::W);
    nb_dir.push_back(OcTreeLUT::E);
    nb_dir.push_back(OcTreeLUT::N);
    nb_dir.push_back(OcTreeLUT::S);
    nb_dir.push_back(OcTreeLUT::T);
    nb_dir.push_back(OcTreeLUT::B);
    // edge directions
    /*
    nb_dir.push_back(OcTreeLUT::SW);
    nb_dir.push_back(OcTreeLUT::NW);
    nb_dir.push_back(OcTreeLUT::SE);
    nb_dir.push_back(OcTreeLUT::NE);
    nb_dir.push_back(OcTreeLUT::TW);
    nb_dir.push_back(OcTreeLUT::BW);
    nb_dir.push_back(OcTreeLUT::TE);
    nb_dir.push_back(OcTreeLUT::BE);
    nb_dir.push_back(OcTreeLUT::TN);
    nb_dir.push_back(OcTreeLUT::TS);
    nb_dir.push_back(OcTreeLUT::BN);
    nb_dir.push_back(OcTreeLUT::BS);*/

    int num_frt = 0;
    int num_occ = 0;
    int num_free = 0;
    bool is_frt = false;

    ROS_INFO("Pre Number of all known nodes: %d", ot_nbv->getNumLeafNodes());
    expandbbxRecurs();
    ROS_INFO("Pro Number of all known nodes: %d", ot_nbv->getNumLeafNodes());

    OcTreeKey check_key;

    for (OcTree::leaf_bbx_iterator leaf_it = ot_nbv->begin_leafs_bbx(bbxmin_, bbxmax_); leaf_it != ot_nbv->end_leafs_bbx(); ++leaf_it){
        const OcTreeKey leaf_key = leaf_it.getKey();        // get key of node which leaf iterator refers to
        for (std::vector<OcTreeLUT::NeighborDirection>::iterator nb_dir_it = nb_dir.begin(); nb_dir_it != nb_dir.end(); nb_dir_it++)
        {
            lut.genNeighborKey(leaf_key, *nb_dir_it, nb_key);
            if (!ot_nbv->search(nb_key)) {
                check_key = ot_nbv->coordToKey(leaf_it.getCoordinate());
                if (check_key[0] > bbxminkey[0] && check_key[0] < bbxmaxkey[0]
                 && check_key[1] > bbxminkey[1] && check_key[1] < bbxmaxkey[1]
                 && check_key[2] > bbxminkey[2] && check_key[2] < bbxmaxkey[2] ) {
                    // record frontier coordinates
                    points_frt.push_back(leaf_it.getCoordinate());
                    // display these pseudo-frontier node
                    idx = leaf_it.getDepth();
                    cubeCenter_frt = pointOctomapToMsg(leaf_it.getCoordinate());
                    frtNodesVis.markers[idx].points.push_back(cubeCenter_frt);
                }
                num_frt++;
                is_frt = true;
                break;
            }
        }
        idx = leaf_it.getDepth();
        cubeCenter_known = pointOctomapToMsg(leaf_it.getCoordinate());
        if (ot_nbv->isNodeOccupied(*leaf_it)) {
            occNodesVis.markers[idx].points.push_back(cubeCenter_known);
            num_occ++;
        }
        else {
            freeNodesVis.markers[idx].points.push_back(cubeCenter_known);
            num_free++;
        }
        is_frt = false;
    }
    std_msgs::ColorRGBA frt_color;
    frt_color.r = 1;
    frt_color.g = 1;
    frt_color.b = 0;
    frt_color.a = 0.5;
    disp_mkarr(frtNodesVis, frt_color);
    frt_pub.publish(frtNodesVis);
    for (int i = 0; i < frtNodesVis.markers.size(); i++) {
        frtNodesVis.markers[i].points.clear();
    }

    std_msgs::ColorRGBA occ_color;
    occ_color.r = 1;
    occ_color.g = 0;
    occ_color.b = 0;
    occ_color.a = 1;
    disp_mkarr(occNodesVis, occ_color);
    occ_pub.publish(occNodesVis);
    for (int i = 0; i < occNodesVis.markers.size(); i++) {
        occNodesVis.markers[i].points.clear();
    }

    std_msgs::ColorRGBA free_color;
    free_color.r = 0;
    free_color.g = 1;
    free_color.b = 0;
    free_color.a = 0.3;
    disp_mkarr(freeNodesVis, free_color);
    free_pub.publish(freeNodesVis);
    for (int i = 0; i < freeNodesVis.markers.size(); i++) {
        freeNodesVis.markers[i].points.clear();
    }

    ROS_INFO("Number of occupied nodes: %d", num_occ);
}


void NBVModule::disp_mkarr(visualization_msgs::MarkerArray &marker_arr, std_msgs::ColorRGBA &color)
{
    double depth_size;      // node size in certain depth
    for (int i = 0; i < marker_arr.markers.size(); i++) {
        depth_size = ot_nbv->getNodeSize(i);
        marker_arr.markers[i].header.frame_id = "/base";
        marker_arr.markers[i].header.stamp = ros::Time::now();
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

void NBVModule::expandbbxRecurs()
{
    ros::Time begin = ros::Time::now();
    bool rep = true;
    while (rep) {
        rep = false;
        for (OcTree::leaf_bbx_iterator leaf_it = ot_nbv->begin_leafs_bbx(bbxmin_, bbxmax_); leaf_it != ot_nbv->end_leafs_bbx(); ++leaf_it)
        {
            if (leaf_it.getDepth()!= m_treeDepth) {
                leaf_it->expandNode();
                unsigned int it_depth = leaf_it.getDepth();
                if ((it_depth++) != m_treeDepth)
                    rep = true;
            }
        }
    }
    ros::Duration elapsed_time = ros::Time::now() - begin;
    ROS_INFO_STREAM("Expand Time elapsed: " << elapsed_time << " sec.");
    return;
}

void NBVModule::intersection()
{
    KeyRay keys;
    point3d origin = pointTfToOctomap(bs_transform.getOrigin());
    point3d end = dir_vec_base;

    // Calculate the distance from frontier point to direction line and then calculate the intersection whose distance to direction vector is minial
    point3d org_frt;        // vector from sensor origin to frontier point
    point3d org_end = dir_vec_base - origin;        // vector from sensor origin to direction vector end point
    double c1;              // dot between org_frt and dir_vec_base
    double c2 = org_end.dot(org_end);     // dot between dir_vec_base and itself
    point3d pt_hit;         // the point nearest to the frontier point
    double lamda;           // ratio of distance from origin to point nearest to the frontier point to the length of direction vector
    double comp = 100;
    for (std::vector<point3d>::iterator frt_it = points_frt.begin(); frt_it != points_frt.end(); frt_it++)
    {
        org_frt = *frt_it - origin;
        c1 = org_frt.dot(org_end);
        if (c1 <= 0) {
            dists = frt_it->distance(origin);
        }
        else if (c2 <= c1) {
            dists = frt_it->distance(end);
        }
        else {
            lamda = c1 / c2;
            pt_hit = origin + org_end * lamda;
            dists = frt_it->distance(pt_hit);
        }
        if (dists < comp) {
            pt_intsct = *frt_it;
            comp = dists;
        }
    }
    points_frt.clear();
    ROS_INFO_STREAM("X " << pt_intsct.x() << " Y " << pt_intsct.y() << " Z " << pt_intsct.z());

    // Display this intersection point
    intersect.header.frame_id = "/base";
    intersect.header.stamp = ros::Time::now();
    intersect.ns = "map";
    intersect.type = visualization_msgs::Marker::POINTS;
    intersect.action = visualization_msgs::Marker::ADD;
    intersect.points.push_back(pointOctomapToMsg(pt_intsct));
    intersect.scale.x = 0.05;
    intersect.scale.y = 0.05;
    intersect.color.r = 1;
    intersect.color.g = 1;
    intersect.color.b = 0.5;
    intersect.color.a = 1;
    intersect.lifetime = ros::Duration();
    intersect_pub.publish(intersect);
    intersect.points.clear();
}

void NBVModule::nearestfrt()
{
    double comp = 100;
    double dist = 0;
    point3d origin = pointTfToOctomap(bs_transform.getOrigin());

    for (std::vector<point3d>::iterator frt_it = points_frt.begin(); frt_it != points_frt.end(); frt_it++)
    {

        dist = frt_it->distance(origin);
        if (dist < comp) {
            near_frt = *frt_it;
            comp = dist;
        }
    }
    points_frt.clear();

    // Display this nearest intersection point
    intersect.header.frame_id = "/base";
    intersect.header.stamp = ros::Time::now();
    intersect.ns = "map";
    intersect.type = visualization_msgs::Marker::POINTS;
    intersect.action = visualization_msgs::Marker::ADD;
    intersect.points.push_back(pointOctomapToMsg(near_frt));
    intersect.scale.x = 0.05;
    intersect.scale.y = 0.05;
    intersect.color.r = 1;
    intersect.color.g = 0.5;
    intersect.color.b = 0.5;
    intersect.color.a = 1;
    intersect.lifetime = ros::Duration();
    intersect_pub.publish(intersect);
    intersect.points.clear();

}

void NBVModule::nbvcandidates()
{

}
