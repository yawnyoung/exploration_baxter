#include <exploration_baxter/octomap_builder.h>

OctomapBuilder::OctomapBuilder(ros::NodeHandle nh):
    m_nh(nh),
    m_res(0.01),            // the initial value of octree resolution is 0.05m
    m_treeDepth(0),
    nbv_probHit(0.7),
    nbv_probMiss(0.4),
    mp_probHit(0.7),
    mp_probMiss(0.4),
    m_thresMin(0.12),
    m_thresMax(0.97),
    m_maxRange(1.5),       // the default negative value indicates that no maximum range is considered
    m_pointcloudMinX(-std::numeric_limits<double>::max()),
    m_pointcloudMaxX(std::numeric_limits<double>::max()),
    m_pointcloudMinY(-std::numeric_limits<double>::max()),
    m_pointcloudMaxY(std::numeric_limits<double>::max()),
    m_pointcloudMinZ(-std::numeric_limits<double>::max()),
    m_pointcloudMaxZ(std::numeric_limits<double>::max()),
    m_filterSpeckles(false),     // set default value as true in order to filter out single speckles by default
    m_pointCloudTopic("/camera/depth/points"),
    m_worldFrameId("/base"),
    m_pointCloudSub(NULL),
    m_tfPointCloudSub(NULL)
{
    /* Minimum and maximum values along each axis for pre-defined free zone */
    double m_freebbxMinX, m_freebbxMinY, m_freebbxMinZ, m_freebbxMaxX, m_freebbxMaxY, m_freebbxMaxZ, torso_freebbxMinX, torso_freebbxMaxX, torso_freebbxMinY;
    /* Initialize octomap object & params */
    m_nh.param("resolution", m_res, m_res);
    m_nh.param("sensor_model/nbv_hit", nbv_probHit, nbv_probHit);
    m_nh.param("sensor_model/nbv_miss", nbv_probMiss, nbv_probMiss);
    m_nh.param("sensor_model/mp_hit", mp_probHit, mp_probHit);
    m_nh.param("sensor_model/mp_miss", mp_probMiss, mp_probMiss);
    m_nh.param("sensor_model/min", m_thresMin, m_thresMin);
    m_nh.param("sensor_model/max", m_thresMax, m_thresMax);
    m_nh.param("pointcloud_min_x", m_pointcloudMinX, m_pointcloudMinX);
    m_nh.param("pointcloud_max_x", m_pointcloudMaxX, m_pointcloudMaxX);
    m_nh.param("pointcloud_min_y", m_pointcloudMinY, m_pointcloudMinY);
    m_nh.param("pointcloud_max_y", m_pointcloudMaxY, m_pointcloudMaxY);
    m_nh.param("pointcloud_min_z", m_pointcloudMinZ, m_pointcloudMinZ);
    m_nh.param("pointcloud_max_z", m_pointcloudMaxZ, m_pointcloudMaxZ);
    m_nh.param("freezone_min_x", m_freebbxMinX, m_freebbxMinX);
    m_nh.param("freezone_min_y", m_freebbxMinY, m_freebbxMinY);
    m_nh.param("freezone_min_z", m_freebbxMinZ, m_freebbxMinZ);
    m_nh.param("freezone_max_x", m_freebbxMaxX, m_freebbxMaxX);
    m_nh.param("freezone_max_y", m_freebbxMaxY, m_freebbxMaxY);
    m_nh.param("freezone_max_z", m_freebbxMaxZ, m_freebbxMaxZ);
    m_nh.param("torsofree_min_x", torso_freebbxMinX, torso_freebbxMinX);
    m_nh.param("torsofree_max_x", torso_freebbxMaxX, torso_freebbxMaxX);
    m_nh.param("torsofree_min_y", torso_freebbxMinY, torso_freebbxMinY);
    m_nh.param("sensor_model/max_range", m_maxRange, m_maxRange);
    m_nh.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
    m_nh.param("pointcloud_topic", m_pointCloudTopic, m_pointCloudTopic);
    m_nh.param("frame_id", m_worldFrameId, m_worldFrameId);

    /* NBV Octree configuration */
    nbv_octree = new octomap::OcTree(m_res);
    nbv_octree->setProbHit(nbv_probHit);
    nbv_octree->setProbMiss(nbv_probMiss);
    nbv_octree->setClampingThresMin(m_thresMin);
    nbv_octree->setClampingThresMax(m_thresMax);
    m_treeDepth = nbv_octree->getTreeDepth();
    /* Variables needed to update node in certain bounding box */
    unsigned int sizeX, sizeY, sizeZ;
    octomap::OcTreeKey bbxkey;
    /* MP Octree configuration */
    ROS_INFO("Start to build octomap for motion planning");
    mp_octree = new octomap::OcTree(m_res);
    mp_octree->setProbHit(mp_probHit);
    mp_octree->setProbMiss(mp_probMiss);
    mp_octree->setClampingThresMin(m_thresMin);
    mp_octree->setClampingThresMax(m_thresMax);
    /* Set pre-defined occupied zone */
    octomap::point3d m_allbbxmin(m_pointcloudMinX, m_pointcloudMinY, m_pointcloudMinZ);
    octomap::point3d m_allbbxmax(m_pointcloudMaxX, m_pointcloudMaxY, m_pointcloudMaxZ);
    if (mp_octree->coordToKeyChecked(m_allbbxmin, m_allbbxminkey) && mp_octree->coordToKeyChecked(m_allbbxmax, m_allbbxmaxkey)) {
        m_allbbxminkey = mp_octree->coordToKey(m_allbbxmin);
        m_allbbxmaxkey = mp_octree->coordToKey(m_allbbxmax);
        sizeX = m_allbbxmaxkey[0] - m_allbbxminkey[0] + 1;
        sizeY = m_allbbxmaxkey[1] - m_allbbxminkey[1] + 1;
        sizeZ = m_allbbxmaxkey[2] - m_allbbxminkey[2] + 1;
        if (sizeX > 0 && sizeY > 0 && sizeZ > 0) {
            ROS_INFO("Obtain whole zone range");
            scan_space = sizeX * sizeY * sizeZ;
            ROS_INFO_STREAM("The number of cells to be scanned is: " << scan_space);
        }
        else {
            ROS_ERROR("Size should be larger than 0. Please check the input bounding box coordinates");
            return;
        }
    }
    else {
        ROS_INFO("Failed to set occupied zone.");
    }
    float occ_log = 0.0;
    for (int dx = 0; dx < sizeX; dx++) {
        bbxkey[0] = m_allbbxminkey[0] + dx;
        for (int dy = 0; dy < sizeY; dy++) {
            bbxkey[1] = m_allbbxminkey[1] + dy;
            for (int dz = 0; dz < sizeZ; dz++) {
                bbxkey[2] = m_allbbxminkey[2] + dz;
                mp_octree->updateNode(bbxkey, occ_log, false);
            }
        }
    }
    /* Pre-defined free zone axis-aligned coordinates */
    octomap::point3d m_freebbxmin(m_freebbxMinX, m_freebbxMinY, m_freebbxMinZ);
    octomap::point3d m_freebbxmax(m_freebbxMaxX, m_freebbxMaxY, m_freebbxMaxZ);
    /* Set pre-defined free zone */
    if (nbv_octree->coordToKeyChecked(m_freebbxmin, m_freebbxminkey) && nbv_octree->coordToKeyChecked(m_freebbxmax, m_freebbxmaxkey)) {
        sizeX = m_freebbxmaxkey[0] - m_freebbxminkey[0] + 1;
        sizeY = m_freebbxmaxkey[1] - m_freebbxminkey[1] + 1;
        sizeZ = m_freebbxmaxkey[2] - m_freebbxminkey[2] + 1;
        if (sizeX > 0 && sizeY > 0 && sizeZ > 0) {
            ROS_INFO("Obtain free zone range");
        }
        else {
            ROS_ERROR("Size should be larger than 0. Please check the input bounding box coordinates");
            return;
        }
    }
    for (int dx = 0; dx < sizeX; dx++) {
        bbxkey[0] = m_freebbxminkey[0] + dx;
        for (int dy = 0; dy < sizeY; dy++) {
            bbxkey[1] = m_freebbxminkey[1] + dy;
            for (int dz = 0; dz < sizeZ; dz++) {
                bbxkey[2] = m_freebbxminkey[2] + dz;
                octomap::OcTreeNode *updt_node = nbv_octree->updateNode(bbxkey, false, false);
                mp_octree->updateNode(bbxkey, false, false);
            }
        }
    }

    /* Set pre-defined free zone for robot torso */
    octomap::point3d torso_freebbxmin(torso_freebbxMinX, torso_freebbxMinY, m_pointcloudMinZ);
    octomap::point3d torso_freebbxmax(torso_freebbxMaxX, m_pointcloudMaxY, m_freebbxMaxZ);
    if (nbv_octree->coordToKeyChecked(torso_freebbxmin, torso_freebbxminkey) && nbv_octree->coordToKeyChecked(torso_freebbxmax, torso_freebbxmaxkey)) {
        sizeX = torso_freebbxmaxkey[0] - torso_freebbxminkey[0] + 1;
        sizeY = torso_freebbxmaxkey[1] - torso_freebbxminkey[1] + 1;
        sizeZ = torso_freebbxmaxkey[2] - torso_freebbxminkey[2] + 1;
        if (sizeX > 0 && sizeY > 0 && sizeZ > 0) {
            ROS_INFO("Obtain free zone range");
        }
        else {
            ROS_ERROR("Size should be larger than 0. Please check the input bounding box coordinates");
            return;
        }
    }
    for (int dx = 0; dx < sizeX; dx++) {
        bbxkey[0] = torso_freebbxminkey[0] + dx;
        for (int dy = 0; dy < sizeY; dy++) {
            bbxkey[1] = torso_freebbxminkey[1] + dy;
            for (int dz = 0; dz < sizeZ; dz++) {
                bbxkey[2] = torso_freebbxminkey[2] + dz;
                octomap::OcTreeNode *updt_node = nbv_octree->updateNode(bbxkey, false, false);
                mp_octree->updateNode(bbxkey, false, false);
            }
        }
    }

    ROS_INFO("Pre-defined free zone set!");

    /* Set neighbor directions for speckles check */
    nb_dir.push_back(octomap::OcTreeLUT::W);
    nb_dir.push_back(octomap::OcTreeLUT::E);
    nb_dir.push_back(octomap::OcTreeLUT::S);
    nb_dir.push_back(octomap::OcTreeLUT::T);
    nb_dir.push_back(octomap::OcTreeLUT::N);
    nb_dir.push_back(octomap::OcTreeLUT::B);

    /* Initialize markers publisher */
    nbv_occ_pub = m_nh.advertise<visualization_msgs::MarkerArray>("nbv_marker_occ", 1);
    /* Set colors for free and occupied cells */
    free_color.r = 0;
    free_color.g = 1;
    free_color.b = 0;
    free_color.a = 1;
    occ_color.r = 1;
    occ_color.g = 0;
    occ_color.b = 0;
    occ_color.a = 1;

    /* Set publisher of updating pointcloud */
    updtPc_pub = m_nh.advertise<PCLPointCloud>("updating_pointcloud", 1);

    /* Listen to pointcloud and convert it to octomap */
    m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, m_pointCloudTopic, 1);
    m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 1);
    m_tfPointCloudSub->registerCallback(boost::bind(&OctomapBuilder::insertClouddiffCallback, this, _1));

}

void OctomapBuilder::specklesfilter()
{
    octomap::OcTreeLUT lut(m_treeDepth);
    octomap::OcTreeKey leaf_key, nb_key;
    bool neighborfound = false;
    for (octomap::OcTree::leaf_iterator leaf_it = nbv_octree->begin_leafs(); leaf_it != nbv_octree->end_leafs(); ++leaf_it)
    {
        neighborfound = false;
        if (leaf_it.getDepth() == m_treeDepth) {
            leaf_key = leaf_it.getKey();
            for (std::vector<octomap::OcTreeLUT::NeighborDirection>::iterator nb_dir_it = nb_dir.begin(); nb_dir_it != nb_dir.end(); nb_dir_it++)
            {
                lut.genNeighborKey(leaf_key, *nb_dir_it, nb_key);
                octomap::OcTreeNode *node = nbv_octree->search(nb_key);
                if (node && nbv_octree->isNodeOccupied(node)) {
                    neighborfound = true;
                    break;
                }
            }
            if (!neighborfound) {
                nbv_octree->deleteNode(leaf_key);
            }
        }
    }
}

void OctomapBuilder::insertClouddiffCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    ros::WallTime startTime = ros::WallTime::now();

    /* Listen to the tf for transformation between sensor frame and base frame */
    tf::StampedTransform sensorToWorldTf;
    try {
        m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
        //ROS_INFO("Obtain transformation between sensor frame and base frame.");
    } catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback.");
        return;
    }
    /* Input cloud for filtering */
    PCLPointCloud pc;
    /* Convert std_msgs pointcloud to PCL ponitcloud in order to use PCL functions to filter pointcloud */
    pcl::fromROSMsg(*cloud, pc);
    /* Transform pointcloud from sensor frame to base frame */
    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
    pcl::transformPointCloud(pc, pc, sensorToWorld);
    /* Publish filtered pointcloud for checking the filter outcome*/
    std::vector<int> map_index;
    pcl::removeNaNFromPointCloud(pc, pc, map_index);

    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    //ROS_INFO("Pointcloud was filtered in %f sec.", total_elapsed);
    insertScandiff(sensorToWorldTf.getOrigin(), pc);
    /* Ignore speckles in the map */
    if (m_filterSpeckles) {
        specklesfilter();
    }
    total_elapsed = (ros::WallTime::now() - startTime).toSec();
    //ROS_INFO("Pointcloud insertion in OctomapBuilder done in %f sec.", total_elapsed);
    /* Publish display markers */
    publishAlldiff(cloud->header.stamp);
}

void OctomapBuilder::insertScandiff(const tf::Point& sensorOriginTf, const PCLPointCloud& input_pc)
{
    size_insert = 0;
    size_insert = input_pc.size();

    octomap::KeySet free_cells, occupied_cells;
    sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

    /* For all the points: free on ray, occupied on endpoint. */
    for (PCLPointCloud::const_iterator it = input_pc.begin(); it != input_pc.end(); ++it)
    {
        octomap::point3d point(it->x, it->y, it->z);
        /* max_range check */
        if ((m_maxRange < 0.0) || (point - sensorOrigin).norm() <= m_maxRange) {
            /* Calculate free cells along each ray */
            if (nbv_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }
            /* Occupied endpoint */
            octomap::OcTreeKey key;
            if (nbv_octree->coordToKeyChecked(point, key)) {
                occupied_cells.insert(key);
            }
        }
        else {
            /* Situation where ray is longer than maxrange */
            octomap::point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            if (nbv_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)) {
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }
        }
    }

    for (octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it)
    {
        if ((*it)[0] > m_allbbxminkey[0] && (*it)[1] > m_allbbxminkey[1] && (*it)[2] > m_allbbxminkey[2]
                && (*it)[0] < m_allbbxmaxkey[0] && (*it)[1] < m_allbbxmaxkey[1] && (*it)[2] < m_allbbxmaxkey[2]) {
            if (!nbv_octree->search(*it)) {
                updt_pc.push_back(octomap::pointOctomapToPCL<pcl::PointXYZ>(nbv_octree->keyToCoord(*it)));
            }
        }
        octomap::OcTreeNode *updt_node = nbv_octree->updateNode(*it, false);
        mp_octree->updateNode(*it, false);
    }
    /* Mark all occupied cells */
    for (octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
    {
        if ((*it)[0] > m_allbbxminkey[0] && (*it)[1] > m_allbbxminkey[1] && (*it)[2] > m_allbbxminkey[2]
                && (*it)[0] < m_allbbxmaxkey[0] && (*it)[1] < m_allbbxmaxkey[1] && (*it)[2] < m_allbbxmaxkey[2]) {
            if (!nbv_octree->search(*it)) {
                updt_pc.push_back(octomap::pointOctomapToPCL<pcl::PointXYZ>(nbv_octree->keyToCoord(*it)));
            }
        }
        // TODO filter points on the robot itself
        /* Filter occupied points in the free zone out */
        if (!((*it)[0] > m_freebbxminkey[0] && (*it)[1] > m_freebbxminkey[1] && (*it)[2] > m_freebbxminkey[2]
                    && (*it)[0] < m_freebbxmaxkey[0] && (*it)[1] < m_freebbxmaxkey[1] && (*it)[2] < m_freebbxmaxkey[2])) {
            if (!((*it)[0] > torso_freebbxminkey[0] && (*it)[1] > torso_freebbxminkey[1] && (*it)[2] > torso_freebbxminkey[2]
                        && (*it)[0] < torso_freebbxmaxkey[0] && (*it)[1] < torso_freebbxmaxkey[1] && (*it)[2] < torso_freebbxmaxkey[2])) {
                octomap::OcTreeNode *updt_node = nbv_octree->updateNode(*it, true);
                mp_octree->updateNode(*it, true);
            }
        }
    }
}


void OctomapBuilder::publishAlldiff(const ros::Time& rostime)
{
    /* Markers for free space and occupied space */
    visualization_msgs::MarkerArray nbv_occNodesVis;
    /* Each array stores all cubes of a different size, one for each depth level */
    nbv_occNodesVis.markers.resize(m_treeDepth + 1);

    geometry_msgs::Point cubeCenter;
    /* NBV markers */
    for (octomap::OcTree::leaf_iterator leaf_it = nbv_octree->begin_leafs(); leaf_it != nbv_octree->end_leafs(); ++leaf_it)
    {
        unsigned idx = leaf_it.getDepth();
        cubeCenter = octomap::pointOctomapToMsg(leaf_it.getCoordinate());
        if (nbv_octree->isNodeOccupied(*leaf_it)) {
            nbv_occNodesVis.markers[idx].points.push_back(cubeCenter);
        }
    }
    visualization::dispMkarr(nbv_octree, nbv_occNodesVis, occ_color, rostime, m_worldFrameId);
    nbv_occ_pub.publish(nbv_occNodesVis);
    /* Set header of pointcloud */
    sensor_msgs::PointCloud2 temp_pc;
    temp_pc.header.frame_id = m_worldFrameId;
    temp_pc.header.stamp = ros::Time::now();
    updt_pc.header = pcl_conversions::toPCL(temp_pc.header);
    updtPc_pub.publish(updt_pc);
}

void OctomapBuilder::start()
{
    ros::Rate r(0.5);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}

void OctomapBuilder::handle_thread()
{
    build_thread = new boost::thread(boost::bind(&OctomapBuilder::start, this));
}

void OctomapBuilder::join_thrd()
{
    //build_thread->join();
    build_thread->detach();
}
