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

    /* Set ranges */
    s1_pos.x() = 0.0639686;
    s1_pos.y() = -0.259257;
    s1_pos.z() = 0.390049;
    range_one = 0.489;
    range_two = 0.969;

    /* Set publisher of frontier cell visualization */
    frt_pub = nh.advertise<visualization_msgs::MarkerArray>("marker_frt", 1);
    /* Set publisher of frontier pointcloud */
    frtPc_pub = nh.advertise<PointCloud>("frontier_pointcloud", 1);
    /* Set publisher of real occupied pointcloud */
    occPc_pub = nh.advertise<PointCloud>("occupied_pointcloud", 1);
    /* Set publisher of void-frontier pointcloud */
    voidfrtPc_pub = nh.advertise<PointCloud>("voidfrontier_pointcloud", 1);
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

void MPmapFilter::FrtNbvCandidates(OcTree *octree, point3d &origin, point3d &last_frontier)
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
    /* Clear frontier_unknown neighbor pairs */
    frt_unknown.clear();
    /* Clear frontier properties vector */
    frt_areaone.clear();
    frt_areatwo.clear();
    frt_areathree.clear();
    /* Clear frontier properties group */
    frt_group.clear();
    /* Clear PCL frontier pointcloud */
    frt_pc.clear();
    /* Clear PCL occupied pointcloud */
    occ_pc.clear();
    /* Clear volume for each area */
    vol_areaone_nbv = 0;
    vol_areatwo_nbv = 0;
    vol_areathree_nbv = 0;
    vol_workspace_nbv = 0;
    /* Clear number of nodes in each area */
    num_frtone = 0;
    num_frttwo = 0;
    num_frtthree = 0;
    /* Initialize the flag of frontier observation */
    frt_observed = true;

    ROS_INFO_STREAM("X: " << origin.x() <<  " Y: " << origin.y() << " Z: " << origin.z());
    //ros::WallTime begin_extract = ros::WallTime::now();
    for (OcTree::leaf_iterator leaf_it = octree->begin_leafs(); leaf_it != octree->end_leafs(); ++leaf_it)
    {
        /* For free cells extract frontier */
        leaf_key = leaf_it.getKey();
        if (leaf_key[0] > nbv_bbxminkey[0] && leaf_key[1] > nbv_bbxminkey[1] && leaf_key[2] < nbv_bbxmaxkey[2]
                && leaf_key[0] < nbv_bbxmaxkey[0] && leaf_key[1] < nbv_bbxmaxkey[1] && leaf_key[2] > nbv_bbxminkey[2])
        {
            vol_workspace_nbv += pow(leaf_it.getSize(), 3);
            point3d leaf_pt = leaf_it.getCoordinate();
            /* Record the node number in each area for nbv_octree */
            if (leaf_pt.y() < -0.22 && leaf_pt.distance(s1_pos) < range_one) {
                vol_areaone_nbv += pow(leaf_it.getSize(), 3);
            }
            else if (leaf_pt.y() >= -0.22 && leaf_pt.distance(s1_pos) < range_one || leaf_pt.distance(s1_pos) < range_two) {
                vol_areatwo_nbv += pow(leaf_it.getSize(), 3);
            }
            else {
                vol_areathree_nbv += pow(leaf_it.getSize(), 3);
            }
            if (!octree->isNodeOccupied(*leaf_it)) {
                for (std::vector<OcTreeLUT::NeighborDirection>::iterator nb_dir_it = nb_dir.begin(); nb_dir_it != nb_dir.end(); ++nb_dir_it)
                {
                    lut.genNeighborKey(leaf_key, *nb_dir_it, nb_key);
                    if (!octree->search(nb_key)) {
                        frt_point = octree->keyToCoord(leaf_key);
                        point3d nb_pt = octree->keyToCoord(nb_key);
                        /* Calculate the distance between frontier point and sensor origin */
                        crit = Dist_FrtOrg(frt_point, origin);
                        std::pair<point3d, double> m_pair = std::make_pair(frt_point, crit);
                        cand_pair.push_back(m_pair);
                        /* Insert the frontier_unknown pairs */
                        frt_unknown.push_back(std::make_pair(frt_point, nb_pt));

                        /* Insert the frontier properties */
                        frt_prp m_frtprp;
                        m_frtprp.frt_node = frt_point;
                        m_frtprp.unk_nb = nb_pt;
                        m_frtprp.sensor_frt = crit;
                        m_frtprp.isvoid = false;
                        /* Distinguish points according to the area they belong to */
                        // Area one
                        if (frt_point.y() < -0.22 && frt_point.distance(s1_pos) < range_one) {
                            frt_areaone.push_back(m_frtprp);
                            num_frtone ++;
                        }
                        // Area two
                        else if (frt_point.y() >= -0.22 && frt_point.distance(s1_pos) < range_one) {
                            frt_areatwo.push_back(m_frtprp);
                            num_frttwo ++;
                        }
                        else if (frt_point.distance(s1_pos) < range_two) {
                            frt_areatwo.push_back(m_frtprp);
                            num_frttwo ++;
                        }
                        // Area three
                        else {
                            frt_areathree.push_back(m_frtprp);
                            num_frtthree ++;
                        }

                        /* Frontier group for all frontier cells */
                        frt_group.push_back(m_frtprp);

                        /* Convert octomap points into pointcloud2 for normal estimation */
                        pcl::PointXYZ temp_frt(frt_point.x(), frt_point.y(), frt_point.z());
                        frt_pc.push_back(temp_frt);

                        break;
                    }
                }
            }
            /* Convert occupied cells to pointcloud */
            else {
                point3d occ_point = octree->keyToCoord(leaf_it.getKey());
                occ_pc.push_back(pcl::PointXYZ(occ_point.x(), occ_point.y(), occ_point.z()));
            }
        }
    }
    /* Check whether the last frontier was observed */
    if (frt_pc.size() > 0) {
        PointCloud::Ptr frtPc_ptr(new PointCloud(frt_pc));
        pcl::KdTreeFLANN<pcl::PointXYZ> frt_kdtree;
        frt_kdtree.setInputCloud(frtPc_ptr);
        std::vector<int> indices;
        std::vector<float> sqr_dists;
        /* Search occupied neighbors within radius search */
        pcl::PointXYZ frtpt(last_frontier.x(), last_frontier.y(), last_frontier.z());
        if (frt_kdtree.radiusSearch(frtpt, 0.005, indices, sqr_dists) > 0) {
            frt_observed = false;
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
    ROS_INFO_STREAM("The size of frontier pointcloud is: " << frt_pc.size());
    ROS_INFO_STREAM("The size of occupied pointcloud is: " << occ_pc.size());
    if (occ_pc.size() != 0) {
        //voidfrtExtraction();
        VoidfrtgroupExtract();
    }
    voidfrt_pc.header = pcl_conversions::toPCL(temp_pc.header);
    voidfrtPc_pub.publish(voidfrt_pc);
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
    /* Clear void-frontier pointcloud */
    voidfrt_pc.clear();
    /* Extract void-frontier if it has occupied neighbors in r m */
    for (std::vector<frt_prp>::iterator m_it = frt_areaone.begin(); m_it != frt_areaone.end(); ++m_it)
    {
        std::vector<int> indices;
        std::vector<float> sqr_dists;
        /* Search occupied neighbors within radius search */
        pcl::PointXYZ itpt(m_it->frt_node.x(), m_it->frt_node.y(), m_it->frt_node.z());
        if (occ_kdtree.radiusSearch(itpt, voidfrt_radius, indices, sqr_dists) > 5) {
            pcl::PointXYZ occpt(occPc_ptr->points[indices[0] ]);
            Eigen::Vector3f occpt_eig(occpt.x, occpt.y, occpt.z);
            Eigen::Vector3f itnb_eig(m_it->unk_nb.x(), m_it->unk_nb.y(), m_it->unk_nb.z());
            Eigen::Vector3f itpt_eig(itpt.x, itpt.y, itpt.z);
            Eigen::Vector3f frt_occ(occpt_eig - itpt_eig);
            Eigen::Vector3f frt_unk(itnb_eig - itpt_eig);
            if (frt_unk.dot(frt_occ) > 0) {
                ++num_voidfrt;
                voidfrt_pc.push_back(itpt);
                m_it->isvoid = true;
            }
        }
    }
    for (std::vector<frt_prp>::iterator m_it = frt_areatwo.begin(); m_it != frt_areatwo.end(); ++m_it)
    {
        std::vector<int> indices;
        std::vector<float> sqr_dists;
        /* Search occupied neighbors within radius search */
        pcl::PointXYZ itpt(m_it->frt_node.x(), m_it->frt_node.y(), m_it->frt_node.z());
        if (occ_kdtree.radiusSearch(itpt, voidfrt_radius, indices, sqr_dists) > 5) {
            pcl::PointXYZ occpt(occPc_ptr->points[indices[0] ]);
            Eigen::Vector3f occpt_eig(occpt.x, occpt.y, occpt.z);
            Eigen::Vector3f itnb_eig(m_it->unk_nb.x(), m_it->unk_nb.y(), m_it->unk_nb.z());
            Eigen::Vector3f itpt_eig(itpt.x, itpt.y, itpt.z);
            Eigen::Vector3f frt_occ(occpt_eig - itpt_eig);
            Eigen::Vector3f frt_unk(itnb_eig - itpt_eig);
            if (frt_unk.dot(frt_occ) > 0) {
                ++num_voidfrt;
                voidfrt_pc.push_back(itpt);
                m_it->isvoid = true;
            }
        }
    }
    for (std::vector<frt_prp>::iterator m_it = frt_areathree.begin(); m_it != frt_areathree.end(); ++m_it)
    {
        std::vector<int> indices;
        std::vector<float> sqr_dists;
        /* Search occupied neighbors within radius search */
        pcl::PointXYZ itpt(m_it->frt_node.x(), m_it->frt_node.y(), m_it->frt_node.z());
        if (occ_kdtree.radiusSearch(itpt, voidfrt_radius, indices, sqr_dists) > 5) {
            pcl::PointXYZ occpt(occPc_ptr->points[indices[0] ]);
            Eigen::Vector3f occpt_eig(occpt.x, occpt.y, occpt.z);
            Eigen::Vector3f itnb_eig(m_it->unk_nb.x(), m_it->unk_nb.y(), m_it->unk_nb.z());
            Eigen::Vector3f itpt_eig(itpt.x, itpt.y, itpt.z);
            Eigen::Vector3f frt_occ(occpt_eig - itpt_eig);
            Eigen::Vector3f frt_unk(itnb_eig - itpt_eig);
            if (frt_unk.dot(frt_occ) > 0) {
                ++num_voidfrt;
                voidfrt_pc.push_back(itpt);
                m_it->isvoid = true;
            }
        }
    }
    /* Calculate the proportion of void-frontier points to frontier points */
    voidfrt_frt = double(num_voidfrt) / double(frt_pc.size());
}

void MPmapFilter::VoidfrtgroupExtract()
{
    /* Pointer to occupied pointcloud */
    PointCloud::Ptr occPc_ptr(new PointCloud(occ_pc));
    /* Build KdTree for occupied pointcloud */
    pcl::KdTreeFLANN<pcl::PointXYZ> occ_kdtree;
    occ_kdtree.setInputCloud(occPc_ptr);
    /* Number of void-frontier points */
    unsigned int num_voidfrtone, num_voidfrttwo, num_voidfrtthree;
    num_voidfrtone = 0;
    num_voidfrttwo = 0;
    num_voidfrtthree = 0;
    /* Clear void-frontier pointcloud */
    voidfrt_pc.clear();
    for (std::vector<frt_prp>::iterator m_it = frt_group.begin(); m_it != frt_group.end(); m_it++)
    {
        std::vector<int> indices;
        std::vector<float> sqr_dists;
        /* Search occupied neighbors within radius search */
        pcl::PointXYZ itpt(m_it->frt_node.x(), m_it->frt_node.y(), m_it->frt_node.z());
        if (occ_kdtree.radiusSearch(itpt, voidfrt_radius, indices, sqr_dists) > 5) {
            /*pcl::PointXYZ occpt(occPc_ptr->points[indices[0] ]);
            Eigen::Vector3f occpt_eig(occpt.x, occpt.y, occpt.z);
            Eigen::Vector3f itnb_eig(m_it->unk_nb.x(), m_it->unk_nb.y(), m_it->unk_nb.z());
            Eigen::Vector3f itpt_eig(itpt.x, itpt.y, itpt.z);
            Eigen::Vector3f frt_occ(occpt_eig - itpt_eig);
            Eigen::Vector3f frt_unk(itnb_eig - itpt_eig);
            if (frt_unk.dot(frt_occ) > 0) {
                ++num_voidfrt;
                voidfrt_pc.push_back(itpt);
                m_it->isvoid = true;
            }*/

            // TODO Distinguish void_frontier in different areas
            if (m_it->frt_node.y() < -0.22 && m_it->frt_node.distance(s1_pos) < range_one) {
                num_voidfrtone++;
            }
            else if (m_it->frt_node.y() >= 0.22 && m_it->frt_node.distance(s1_pos) < range_one || m_it->frt_node.distance(s1_pos) < range_two) {
                num_voidfrttwo++;
            }
            else {
                num_voidfrtthree++;
            }
            voidfrt_pc.push_back(itpt);
            m_it->isvoid = true;
        }
    }
    voidfrt_frtone = double(num_voidfrtone) / double(num_frtone);
    voidfrt_frttwo = double(num_voidfrttwo) / double(num_frttwo);
    voidfrt_frtthree = double(num_voidfrtthree) / double(num_frtthree);
}
