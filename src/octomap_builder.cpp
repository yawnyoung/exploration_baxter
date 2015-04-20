#include <exploration_baxter/octomap_builder.h>

OctomapBuilder::OctomapBuilder(ros::NodeHandle nh, bool diffmap):
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
    double m_freebbxMinX, m_freebbxMinY, m_freebbxMinZ, m_freebbxMaxX, m_freebbxMaxY, m_freebbxMaxZ, torso_freebbxMinX, torso_freebbxMaxX;
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
    if (diffmap) {
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
    }
    else {
        ROS_INFO("No octomap for motion planning");
    }
    /* Pre-defined free zone axis-aligned coordinates */
    octomap::point3d m_freebbxmin(m_freebbxMinX, m_freebbxMinY, m_freebbxMinZ);
    octomap::point3d m_freebbxmax(m_freebbxMaxX, m_freebbxMaxY, m_freebbxMaxZ);
    /* Pre-defined free zone bounding box keys */
    octomap::OcTreeKey m_freebbxminkey, m_freebbxmaxkey;
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
    if (diffmap) {
        for (int dx = 0; dx < sizeX; dx++) {
            bbxkey[0] = m_freebbxminkey[0] + dx;
            for (int dy = 0; dy < sizeY; dy++) {
                bbxkey[1] = m_freebbxminkey[1] + dy;
                for (int dz = 0; dz < sizeZ; dz++) {
                    bbxkey[2] = m_freebbxminkey[2] + dz;
                    nbv_octree->updateNode(bbxkey, false, false);
                    mp_octree->updateNode(bbxkey, false, false);
                }
            }
        }
    }
    else {
        for (int dx = 0; dx < sizeX; dx++) {
            bbxkey[0] = m_freebbxminkey[0] + dx;
            for (int dy = 0; dy < sizeY; dy++) {
                bbxkey[1] = m_freebbxminkey[1] + dy;
                for (int dz = 0; dz < sizeZ; dz++) {
                    bbxkey[2] = m_freebbxminkey[2] + dz;
                    nbv_octree->updateNode(bbxkey, false, false);
                }
            }
        }
    }

    /* Set pre-defined free zone for robot torso */
    octomap::point3d torso_freebbxmin(torso_freebbxMinX, m_freebbxMaxY, m_pointcloudMinZ);
    octomap::point3d torso_freebbxmax(torso_freebbxMaxX, m_pointcloudMaxY, m_pointcloudMaxZ);
    octomap::OcTreeKey torso_freebbxminkey, torso_freebbxmaxkey;
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
    if (diffmap) {
        for (int dx = 0; dx < sizeX; dx++) {
            bbxkey[0] = torso_freebbxminkey[0] + dx;
            for (int dy = 0; dy < sizeY; dy++) {
                bbxkey[1] = torso_freebbxminkey[1] + dy;
                for (int dz = 0; dz < sizeZ; dz++) {
                    bbxkey[2] = torso_freebbxminkey[2] + dz;
                    nbv_octree->updateNode(bbxkey, false, false);
                    mp_octree->updateNode(bbxkey, false, false);
                }
            }
        }
    }
    else {
        for (int dx = 0; dx < sizeX; dx++) {
            bbxkey[0] = torso_freebbxminkey[0] + dx;
            for (int dy = 0; dy < sizeY; dy++) {
                bbxkey[1] = torso_freebbxminkey[1] + dy;
                for (int dz = 0; dz < sizeZ; dz++) {
                    bbxkey[2] = torso_freebbxminkey[2] + dz;
                    nbv_octree->updateNode(bbxkey, false, false);
                }
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
    nbv_free_pub = m_nh.advertise<visualization_msgs::MarkerArray>("nbv_marker_free", 1);
    nbv_occ_pub = m_nh.advertise<visualization_msgs::MarkerArray>("nbv_marker_occ", 1);
    mp_free_pub = m_nh.advertise<visualization_msgs::MarkerArray>("mp_marker_free", 1);
    mp_occ_pub = m_nh.advertise<visualization_msgs::MarkerArray>("mp_marker_occ", 1);
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
    /* Set publisher of filtered pointcloud */
    filterPc_pub = m_nh.advertise<PCLPointCloud>("filtered_pointcloud", 1);

    /* Listen to pointcloud and convert it to octomap */
    m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, m_pointCloudTopic, 1);
    m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 1);
    if (!diffmap) {
        m_tfPointCloudSub->registerCallback(boost::bind(&OctomapBuilder::insertCloudCallback, this, _1));
    }
    else {
        m_tfPointCloudSub->registerCallback(boost::bind(&OctomapBuilder::insertClouddiffCallback, this, _1));
    }

}

void OctomapBuilder::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
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
    /* Set up filter for pointcloud. Filter out points not in the interested bounding box, and also remove the NAN points */
    /*pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pc.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
    pass.filter(pc);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
    pass.filter(pc);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);
    pass.filter(pc);*/
    //ROS_INFO("Filtered pointcloud size is %lu", pc.size());

    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    //ROS_INFO("Pointcloud was filtered in %f sec.", total_elapsed);
    insertScan(sensorToWorldTf.getOrigin(), pc);
    /* Ignore speckles in the map */
    if (m_filterSpeckles) {
        specklesfilter();
    }
    total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_INFO("Pointcloud insertion in OctomapBuilder done in %f sec.", total_elapsed);
    /* Publish display markers */
    publishAll(cloud->header.stamp);
}

void OctomapBuilder::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& input_pc)
{
    octomap::KeySet free_cells, occupied_cells;
    sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);
    ROS_INFO_STREAM("X: " << sensorOrigin.x() << " Y: " << sensorOrigin.y() << " Z: " << sensorOrigin.z());

    /* For all the points: free on ray, occupied on endpoint. */
    for (PCLPointCloud::const_iterator it = input_pc.begin(); it != input_pc.end(); ++it)
    {
        octomap::point3d point(it->x, it->y, it->z);
        // max_range check
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
    /* Mark free cells only if not seen occupied in this cloud */
    for (octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it)
    {
        if (occupied_cells.find(*it) == occupied_cells.end()) {
            nbv_octree->updateNode(*it, false);
        }
    }
    /* Mark all occupied cells */
    for (octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
    {
        nbv_octree->updateNode(*it, true);
    }
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
    /* Set up filter for pointcloud. Filter out points not in the interested bounding box, and also remove the NAN points */
    /*pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pc.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
    pass.filter(pc);
    pass.setInputCloud(pc.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
    pass.filter(pc);
    pass.setInputCloud(pc.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);
    pass.filter(pc);*/
    //ROS_INFO("Filtered pointcloud size is %lu", pc.size());
    /* Publish filtered pointcloud for checking the filter outcome*/
    std::vector<int> map_index;
    pcl::removeNaNFromPointCloud(pc, pc, map_index);
    sensor_msgs::PointCloud2 temp_pc;
    temp_pc.header.frame_id = m_worldFrameId;
    temp_pc.header.stamp = ros::Time::now();
    pc.header = pcl_conversions::toPCL(temp_pc.header);
    filterPc_pub.publish(pc);

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
        if (occupied_cells.find(*it) == occupied_cells.end()) {
            if (!nbv_octree->search(*it)
            && (*it)[0] > m_allbbxminkey[0] && (*it)[1] > m_allbbxminkey[1] && (*it)[2] > m_allbbxminkey[2]
            && (*it)[0] < m_allbbxmaxkey[0] && (*it)[1] < m_allbbxmaxkey[1] && (*it)[2] < m_allbbxmaxkey[2]) {

                /* updating pointcloud */
                updt_pc.push_back(octomap::pointOctomapToPCL<pcl::PointXYZ>(nbv_octree->keyToCoord(*it)));
            }
            nbv_octree->updateNode(*it, false);
            mp_octree->updateNode(*it, false);
        }
    }
    /* Mark all occupied cells */
    for (octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
    {
        if (!nbv_octree->search(*it)
            && (*it)[0] > m_allbbxminkey[0] && (*it)[1] > m_allbbxminkey[1] && (*it)[2] > m_allbbxminkey[2]
            && (*it)[0] < m_allbbxmaxkey[0] && (*it)[1] < m_allbbxmaxkey[1] && (*it)[2] < m_allbbxmaxkey[2]) {
            /* updating pointcloud */
            updt_pc.push_back(octomap::pointOctomapToPCL<pcl::PointXYZ>(nbv_octree->keyToCoord(*it)));
        }
        nbv_octree->updateNode(*it, true);
        mp_octree->updateNode(*it, true);
    }
}

void OctomapBuilder::publishAll(const ros::Time& rostime)
{
    /* Markers for free space and occupied space */
    visualization_msgs::MarkerArray freeNodesVis, occNodesVis;
    /* Each array stores all cubes of a different size, one for each depth level */
    freeNodesVis.markers.resize(m_treeDepth + 1);
    occNodesVis.markers.resize(m_treeDepth + 1);

    geometry_msgs::Point cubeCenter;
    for (octomap::OcTree::leaf_iterator leaf_it = nbv_octree->begin_leafs(); leaf_it != nbv_octree->end_leafs(); ++leaf_it)
    {
        unsigned idx = leaf_it.getDepth();
        cubeCenter = octomap::pointOctomapToMsg(leaf_it.getCoordinate());
        if (nbv_octree->isNodeOccupied(*leaf_it)) {
            occNodesVis.markers[idx].points.push_back(cubeCenter);
        }
        else {
            freeNodesVis.markers[idx].points.push_back(cubeCenter);
        }
    }
    visualization::dispMkarr(nbv_octree, occNodesVis, occ_color, rostime, m_worldFrameId);
    visualization::dispMkarr(nbv_octree, freeNodesVis, free_color, rostime, m_worldFrameId);
    nbv_free_pub.publish(freeNodesVis);
    nbv_occ_pub.publish(occNodesVis);
}

void OctomapBuilder::publishAlldiff(const ros::Time& rostime)
{
    /* Markers for free space and occupied space */
    visualization_msgs::MarkerArray nbv_freeNodesVis, nbv_occNodesVis, mp_freeNodesVis, mp_occNodesVis;
    /* Each array stores all cubes of a different size, one for each depth level */
    nbv_freeNodesVis.markers.resize(m_treeDepth + 1);
    nbv_occNodesVis.markers.resize(m_treeDepth + 1);
    mp_freeNodesVis.markers.resize(m_treeDepth + 1);
    mp_occNodesVis.markers.resize(m_treeDepth + 1);

    geometry_msgs::Point cubeCenter;
    /* NBV markers */
    for (octomap::OcTree::leaf_iterator leaf_it = nbv_octree->begin_leafs(); leaf_it != nbv_octree->end_leafs(); ++leaf_it)
    {
        unsigned idx = leaf_it.getDepth();
        cubeCenter = octomap::pointOctomapToMsg(leaf_it.getCoordinate());
        if (nbv_octree->isNodeOccupied(*leaf_it)) {
            nbv_occNodesVis.markers[idx].points.push_back(cubeCenter);
        }
        else {
            nbv_freeNodesVis.markers[idx].points.push_back(cubeCenter);
        }
    }
    /* MP markers */
    for (octomap::OcTree::leaf_iterator leaf_it = mp_octree->begin_leafs(); leaf_it != mp_octree->end_leafs(); ++leaf_it)
    {
        unsigned idx = leaf_it.getDepth();
        cubeCenter = octomap::pointOctomapToMsg(leaf_it.getCoordinate());
        if (mp_octree->isNodeOccupied(*leaf_it)) {
            mp_occNodesVis.markers[idx].points.push_back(cubeCenter);
        }
        else {
            mp_freeNodesVis.markers[idx].points.push_back(cubeCenter);
        }
    }
    visualization::dispMkarr(nbv_octree, nbv_occNodesVis, occ_color, rostime, m_worldFrameId);
    visualization::dispMkarr(nbv_octree, nbv_freeNodesVis, free_color, rostime, m_worldFrameId);
    visualization::dispMkarr(mp_octree, mp_occNodesVis, occ_color, rostime, m_worldFrameId);
    visualization::dispMkarr(mp_octree, mp_freeNodesVis, free_color, rostime, m_worldFrameId);
    nbv_free_pub.publish(nbv_freeNodesVis);
    nbv_occ_pub.publish(nbv_occNodesVis);
    mp_free_pub.publish(mp_freeNodesVis);
    mp_occ_pub.publish(mp_occNodesVis);
    /* Set header of pointcloud */
    sensor_msgs::PointCloud2 temp_pc;
    temp_pc.header.frame_id = m_worldFrameId;
    temp_pc.header.stamp = ros::Time::now();
    updt_pc.header = pcl_conversions::toPCL(temp_pc.header);
    updtPc_pub.publish(updt_pc);
}

void OctomapBuilder::start()
{
    ros::Rate r(1);
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
