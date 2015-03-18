#include <exploration_baxter/octomap_builder.h>

OctomapBuilder::OctomapBuilder(ros::NodeHandle nh, octomap::point3d freebbxmin, octomap::point3d freebbxmax):
    m_nh(nh),
    m_res(0.01),            // the initial value of octree resolution is 0.05m
    m_treeDepth(0),
    m_probHit(0.7),
    m_probMiss(0.4),
    m_thresMin(0.12),
    m_thresMax(0.97),
    m_freebbxmin(freebbxmin),
    m_freebbxmax(freebbxmax),
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
    /* Initialize octomap object & params */
    m_nh.param("resolution", m_res, m_res);
    m_nh.param("sensor_model/hit", m_probHit, m_probHit);
    m_nh.param("sensor_model/miss", m_probMiss, m_probMiss);
    m_nh.param("sensor_model/min", m_thresMin, m_thresMin);
    m_nh.param("sensor_model/max", m_thresMax, m_thresMax);
    m_octree = new octomap::OcTree(m_res);
    m_octree->setProbHit(m_probHit);
    m_octree->setProbMiss(m_probMiss);
    m_octree->setClampingThresMin(m_thresMin);
    m_octree->setClampingThresMax(m_thresMax);
    m_treeDepth = m_octree->getTreeDepth();
    /* Set pre-defined free zone */
    unsigned int sizeX, sizeY, sizeZ;
    if (m_octree->coordToKeyChecked(m_freebbxmin, m_freebbxminkey) && m_octree->coordToKeyChecked(m_freebbxmax, m_freebbxmaxkey)) {
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
    octomap::OcTreeKey bbxkey;
    for (int dx = 0; dx < sizeX; dx++) {
        bbxkey[0] = m_freebbxminkey[0] + dx;
        for (int dy = 0; dy < sizeY; dy++) {
            bbxkey[1] = m_freebbxminkey[1] + dy;
            for (int dz = 0; dz < sizeZ; dz++) {
                bbxkey[2] = m_freebbxminkey[2] + dz;
                m_octree->updateNode(bbxkey, false, false);
            }
        }
    }
    ROS_INFO("Pre-defined free zone set!");

    m_nh.param("pointcloud_min_x", m_pointcloudMinX, m_pointcloudMinX);
    m_nh.param("pointcloud_max_x", m_pointcloudMaxX, m_pointcloudMaxX);
    m_nh.param("pointcloud_min_y", m_pointcloudMinY, m_pointcloudMinY);
    m_nh.param("pointcloud_max_y", m_pointcloudMaxY, m_pointcloudMaxY);
    m_nh.param("pointcloud_min_z", m_pointcloudMinZ, m_pointcloudMinZ);
    m_nh.param("pointcloud_max_z", m_pointcloudMaxZ, m_pointcloudMaxZ);
    m_nh.param("sensor_model/max_range", m_maxRange, m_maxRange);
    m_nh.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
    /* Set neighbor directions for speckles check */
    nb_dir.push_back(octomap::OcTreeLUT::W);
    nb_dir.push_back(octomap::OcTreeLUT::E);
    nb_dir.push_back(octomap::OcTreeLUT::S);
    nb_dir.push_back(octomap::OcTreeLUT::T);
    nb_dir.push_back(octomap::OcTreeLUT::N);
    nb_dir.push_back(octomap::OcTreeLUT::B);

    /* Initialize markers publisher */
    free_pub = m_nh.advertise<visualization_msgs::MarkerArray>("marker_free", 1);
    occ_pub = m_nh.advertise<visualization_msgs::MarkerArray>("marker_occ", 1);
    /* Set colors for free and occupied cells */
    free_color.r = 0;
    free_color.g = 1;
    free_color.b = 0;
    free_color.a = 1;
    occ_color.r = 1;
    occ_color.g = 0;
    occ_color.b = 0;
    occ_color.a = 1;

    m_nh.param("pointcloud_topic", m_pointCloudTopic, m_pointCloudTopic);
    m_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
    m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, m_pointCloudTopic, 5);
    m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
    m_tfPointCloudSub->registerCallback(boost::bind(&OctomapBuilder::insertCloudCallback, this, _1));

}

void OctomapBuilder::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    ros::WallTime startTime = ros::WallTime::now();

    /* Listen to the tf for transformation between sensor frame and base frame */
    tf::StampedTransform sensorToWorldTf;
    try {
        m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
        ROS_INFO("Obtain transformation between sensor frame and base frame.");
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
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pc.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
    pass.filter(pc);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
    pass.filter(pc);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);
    pass.filter(pc);

    insertScan(sensorToWorldTf.getOrigin(), pc);
    /* Ignore speckles in the map */
    if (m_filterSpeckles) {
        specklesfilter();
    }
    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_INFO("Pointcloud insertion in OctomapBuilder done in %f sec.", total_elapsed);
    /* Publish display markers */
    publishAll(cloud->header.stamp);
}

void OctomapBuilder::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& input_pc)
{
    octomap::KeySet free_cells, occupied_cells;
    octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

    /* For all the points: free on ray, occupied on endpoint. */
    for (PCLPointCloud::const_iterator it = input_pc.begin(); it != input_pc.end(); ++it)
    {
        octomap::point3d point(it->x, it->y, it->z);
        // max_range check
        if ((m_maxRange < 0.0) || (point - sensorOrigin).norm() <= m_maxRange) {
            /* Calculate free cells along each ray */
            if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }
            /* Occupied endpoint */
            octomap::OcTreeKey key;
            if (m_octree->coordToKeyChecked(point, key)) {
                occupied_cells.insert(key);
            }
        }
        else {
            /* Situation where ray is longer than maxrange */
            octomap::point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)) {
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }
        }
    }
    /* Mark free cells only if not seen occupied in this cloud */
    for (octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it)
    {
        if (occupied_cells.find(*it) == occupied_cells.end()) {
            m_octree->updateNode(*it, false);
        }
    }
    /* Mark all occupied cells */
    for (octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
    {
        m_octree->updateNode(*it, true);
    }
}

void OctomapBuilder::specklesfilter()
{
    octomap::OcTreeLUT lut(m_treeDepth);
    octomap::OcTreeKey leaf_key, nb_key;
    bool neighborfound = false;
    for (octomap::OcTree::leaf_iterator leaf_it = m_octree->begin_leafs(); leaf_it != m_octree->end_leafs(); ++leaf_it)
    {
        neighborfound = false;
        if (leaf_it.getDepth() == m_treeDepth) {
            leaf_key = leaf_it.getKey();
            for (std::vector<octomap::OcTreeLUT::NeighborDirection>::iterator nb_dir_it = nb_dir.begin(); nb_dir_it != nb_dir.end(); nb_dir_it++)
            {
                lut.genNeighborKey(leaf_key, *nb_dir_it, nb_key);
                octomap::OcTreeNode *node = m_octree->search(nb_key);
                if (node && m_octree->isNodeOccupied(node)) {
                    neighborfound = true;
                    break;
                }
            }
            if (!neighborfound) {
                m_octree->deleteNode(leaf_key);
            }
        }
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
    for (octomap::OcTree::leaf_iterator leaf_it = m_octree->begin_leafs(); leaf_it != m_octree->end_leafs(); ++leaf_it)
    {
        unsigned idx = leaf_it.getDepth();
        cubeCenter = octomap::pointOctomapToMsg(leaf_it.getCoordinate());
        if (m_octree->isNodeOccupied(*leaf_it)) {
            occNodesVis.markers[idx].points.push_back(cubeCenter);
        }
        else {
            freeNodesVis.markers[idx].points.push_back(cubeCenter);
        }
    }
    visualization::dispMkarr(m_octree, occNodesVis, occ_color, rostime, m_worldFrameId);
    visualization::dispMkarr(m_octree, freeNodesVis, free_color, rostime, m_worldFrameId);
    free_pub.publish(freeNodesVis);
    occ_pub.publish(occNodesVis);
}

