#include "exploration_baxter/MPmap_filter.h"

MPmapFilter::MPmapFilter(ros::NodeHandle nh):
    mp_nh(nh),
    mp_octree(NULL),
    mp_WorldFrameId("/base"),
    isPublishFrt(true)      // Set default value to be true in order to publish frontier visualization cells by default
{
    /* Get parameter */
    mp_nh.param("publish_frontier", isPublishFrt, isPublishFrt);
    /* face directions */
    nb_dir.push_back(OcTreeLUT::W);
    nb_dir.push_back(OcTreeLUT::E);
    nb_dir.push_back(OcTreeLUT::N);
    nb_dir.push_back(OcTreeLUT::S);
    nb_dir.push_back(OcTreeLUT::T);
    nb_dir.push_back(OcTreeLUT::B);

    /* Set publisher of frontier cell visualization */
    frt_pub = nh.advertise<visualization_msgs::MarkerArray>("marker_frt", 1);
    /* Set fronter cell color */
    frt_color.r = 1;
    frt_color.g = 1;
    frt_color.b = 0;
    frt_color.a = 0.5;
}

void MPmapFilter::frontierExtraction(OcTree *octree)
{
    mp_octree = octree;
    mp_treeDepth = mp_octree->getTreeDepth();
    /* Expand octree before extracting frontier */
    //mp_octree->expand();
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

    for (OcTree::leaf_iterator leaf_it = mp_octree->begin_leafs(); leaf_it != mp_octree->end_leafs(); leaf_it++)
    {
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
            }
        }
    }
    const ros::Time current = ros::Time::now();
    mpPublishAll(current);
}

void MPmapFilter::mpPublishAll(const ros::Time &rostime)
{
    if (isPublishFrt) {
        visualization::dispMkarr(mp_octree, frtNodesVis, frt_color, rostime, mp_WorldFrameId);
        frt_pub.publish(frtNodesVis);
    }
}
