#include "exploration_baxter/MPmap_filter.h"

MPmapFilter::MPmapFilter(ros::NodeHandle nh, OcTreeKey bbxminkey, OcTreeKey bbxmaxkey):
    mp_nh(nh),
    mp_WorldFrameId("/base"),
    mp_octree(NULL),
    nbv_bbxminkey(bbxminkey),
    nbv_bbxmaxkey(bbxmaxkey),
    voidfrt_radius(0.1),   // Set default to be 10 cm
    isPublishFrt(true)      // Set default value to be true in order to publish frontier visualization cells by default
{
    /* Get parameter */
    mp_nh.param("publish_frontier", isPublishFrt, isPublishFrt);
    mp_nh.param("voidfrontier_radius", voidfrt_radius, voidfrt_radius);
    /* face directions */
    nb_dir.push_back(OcTreeLUT::W);
    nb_dir.push_back(OcTreeLUT::E);
    nb_dir.push_back(OcTreeLUT::N);
    nb_dir.push_back(OcTreeLUT::S);
    nb_dir.push_back(OcTreeLUT::T);
    nb_dir.push_back(OcTreeLUT::B);

    /* Set publisher of frontier cell visualization */
    frt_pub = nh.advertise<visualization_msgs::MarkerArray>("marker_frt", 1);
    /* Set publisher of frontier pointcloud */
    frtPc_pub = nh.advertise<PointCloud>("frontier_pointcloud", 1);
    /* Set publisher of real occupied pointcloud */
    occPc_pub = nh.advertise<PointCloud>("occupied_pointcloud", 1);
    /* Set frontier cell color */
    frt_color.r = 1;
    frt_color.g = 1;
    frt_color.b = 0;
    frt_color.a = 0.5;

}

void MPmapFilter::frontierExtraction(OcTree *octree)
{
    mp_octree = new OcTree(*octree);
    mp_treeDepth = mp_octree->getTreeDepth();
    /* Expand octree before extracting frontier */
    ros::WallTime begin_expd = ros::WallTime::now();
    //mp_octree->expand();
    //double expd_elapsed = (ros::WallTime::now() - begin_expd).toSec();
    //ROS_INFO("Expand octree spent %f sec.", expd_elapsed);
    /* Lookup table for neighbors */
    OcTreeLUT lut(mp_treeDepth);
    /* Key of leaf cells and neighbors of leaf cells */
    OcTreeKey leaf_key, nb_key;
    /* Set size of frontier marker array */
    frtNodesVis.markers.resize(mp_treeDepth + 1);
    /* Clear the markers of last call */
    for (int i = 0; i < mp_treeDepth + 1; i++) {
        frtNodesVis.markers[i].points.clear();
    }
    /* Centers of frontier cubes */
    geometry_msgs::Point cubeCenter_frt;
    /* Depth of leaf node */
    unsigned idx;
    /* Clear the frontier cell for each loop */
    frontier_cells.clear();
    /* Threshold of occupancy */
    float occ_log = 0.0;

    for (OcTree::leaf_iterator leaf_it = mp_octree->begin_leafs(); leaf_it != mp_octree->end_leafs(); leaf_it++)
    {
        /* Modification: frontier cells just include free cells */
        if (leaf_it->getLogOdds() < occ_log) {
            leaf_key = leaf_it.getKey();
            idx = leaf_it.getDepth();
            for (std::vector<OcTreeLUT::NeighborDirection>::iterator nb_dir_it = nb_dir.begin(); nb_dir_it != nb_dir.end(); nb_dir_it++)
            {
                lut.genNeighborKey(leaf_key, *nb_dir_it, nb_key);
                if (!mp_octree->search(nb_key)) {
                    mp_octree->updateNode(leaf_key, true);
                    frontier_cells.push_back(leaf_key);
                    cubeCenter_frt = pointOctomapToMsg(mp_octree->keyToCoord(leaf_key));
                    frtNodesVis.markers[idx].points.push_back(cubeCenter_frt);
                    break;
                }
            }
        }
    }
    ROS_INFO("Number of frontier cells is: %lu", frontier_cells.size());
    const ros::Time current = ros::Time::now();
    mpPublishAll(current);
    double expd_elapsed = (ros::WallTime::now() - begin_expd).toSec();
    ROS_INFO("Extracting frontier spents %f sec.", expd_elapsed);
    delete mp_octree;
    mp_octree = NULL;
}

void MPmapFilter::FrtNbvCandidates(OcTree *octree, point3d &origin)
{
    /* Expand whole tree for extracting frontier */
    ros::WallTime begin_expd = ros::WallTime::now();
    octree->expand();
    double expd_elapsed = (ros::WallTime::now() - begin_expd).toSec();
    ROS_INFO("Expand octree spent %f sec.", expd_elapsed);
    /* Leaf node depth */
    unsigned idx;
    /* Lookup table for neighbors */
    OcTreeLUT lut(octree->getTreeDepth());
    /* Key of leaf nodes and their neighbors */
    OcTreeKey leaf_key, nb_key;
    /* Point of frontier */
    point3d frt_point;
    /* Return value of criterion function */
    double crit;
    /* Clear candidate pairs */
    cand_pair.clear();
    /* Clear PCL frontier pointcloud */
    frt_pc.clear();
    /* Clear PCL occupied pointcloud */
    occ_pc.clear();

    ROS_INFO_STREAM("X: " << origin.x() <<  " Y: " << origin.y() << " Z: " << origin.z());
    //ros::WallTime begin_extract = ros::WallTime::now();
    for (OcTree::leaf_iterator leaf_it = octree->begin_leafs(); leaf_it != octree->end_leafs(); ++leaf_it)
    {
        /* For free cells extract frontier */
        if (!octree->isNodeOccupied(*leaf_it)) {
        leaf_key = leaf_it.getKey();
        if (leaf_key[0] > nbv_bbxminkey[0] && leaf_key[1] > nbv_bbxminkey[1] && leaf_key[2] < nbv_bbxmaxkey[2]
         && leaf_key[0] < nbv_bbxmaxkey[0] && leaf_key[1] < nbv_bbxmaxkey[1] && leaf_key[2] > nbv_bbxminkey[2])
        {
            for (std::vector<OcTreeLUT::NeighborDirection>::iterator nb_dir_it = nb_dir.begin(); nb_dir_it != nb_dir.end(); ++nb_dir_it)
            {
                lut.genNeighborKey(leaf_key, *nb_dir_it, nb_key);
                if (!octree->search(nb_key)) {
                    frt_point = octree->keyToCoord(leaf_key);
                    /* Calculate the distance between frontier point and sensor origin */
                    crit = Dist_FrtOrg(frt_point, origin);
                    std::pair<point3d, double> m_pair = std::make_pair(frt_point, crit);
                    cand_pair.push_back(m_pair);
                    /* Convert octomap points into pointcloud2 for normal estimation */
                    frt_pc.push_back(pcl::PointXYZ(frt_point.x(), frt_point.y(), frt_point.z()));
                    break;
                }
            }
        }
        }
        /* Convert occupied cells to pointcloud */
        else {
            point3d occ_point = octree->keyToCoord(leaf_it.getKey());
            occ_pc.push_back(pcl::PointXYZ(occ_point.x(), occ_point.y(), occ_point.z()));
        }
    }
    /* Test for publish frontier pointcloud */
    PointCloud::Ptr msg_pc(new PointCloud);
    sensor_msgs::PointCloud2 temp_pc;
    temp_pc.header.frame_id = mp_WorldFrameId;
    temp_pc.header.stamp = ros::Time::now();
    frt_pc.header = pcl_conversions::toPCL(temp_pc.header);
    frtPc_pub.publish(frt_pc);
    occ_pc.header = pcl_conversions::toPCL(temp_pc.header);
    occPc_pub.publish(occ_pc);
    voidfrtExtraction();
}

void MPmapFilter::mpPublishAll(const ros::Time &rostime)
{
    if (isPublishFrt) {
        visualization::dispMkarr(mp_octree, frtNodesVis, frt_color, rostime, mp_WorldFrameId);
        frt_pub.publish(frtNodesVis);
    }
}

double MPmapFilter::Dist_FrtOrg(point3d &frt, point3d &org)
{
    return org.distance(frt);
}

void MPmapFilter::doSort()
{
    std::sort(cand_pair.begin(), cand_pair.end(), comp_crit(*this));
}

void MPmapFilter::voidfrtExtraction()
{
    /* Pointer to occupied pointcloud */
    PointCloud::Ptr occPc_ptr(new PointCloud(occ_pc));
    /* Build KdTree for occupied pointcloud */
    pcl::KdTreeFLANN<pcl::PointXYZ> occ_kdtree;
    occ_kdtree.setInputCloud(occPc_ptr);
    /* Number of void-frontier points */
    unsigned int num_voidfrt = 0;
    /* Extract void-frontier if it has occupied neighbors in r m */
    for (PointCloud::iterator pc_it = frt_pc.begin(); pc_it != frt_pc.end(); ++pc_it)
    {
        std::vector<int> indices;
        std::vector<float> sqr_dists;
        /* Search occupied neighbors within radius search */
        if (occ_kdtree.radiusSearch(*pc_it, voidfrt_radius, indices, sqr_dists) > 5) {
            ++num_voidfrt;
        }
    }
    /* Calculate the proportion of void-frontier points to frontier points */
    voidfrt_frt = double(num_voidfrt) / double(frt_pc.size());

}
