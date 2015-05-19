#include "exploration_baxter/NBVStrategy.h"

NBVStrategy::NBVStrategy(ros::NodeHandle nh):
    m_nh(nh),
    lower_limit(0.5),                       // Set default value to be 0.5 meter
    m_WorldFrame("/base"),                  // Set default world frame to be base frame
    plan_eef("/right_hand_camera"),          // Set default plan end effector to be right_hand_camera
    score_one(80),
    score_two(50),
    score_three(20),
    weight_ig(0.6),
    weight_cont(0.4)
{
    /* Get Parameter */
    m_nh.param("sensor_model/lower_limit", lower_limit, lower_limit);
    m_nh.param("frame_id", m_WorldFrame, m_WorldFrame);
    m_nh.param("infogain_scoreone", score_one, score_one);
    m_nh.param("infogain_scoretwo", score_two, score_two);
    m_nh.param("infogain_scorethree", score_three, score_three);
    m_nh.param("infogain_weight", weight_ig, weight_ig);
    m_nh.param("continuity_weight", weight_cont, weight_cont);
    //m_nh.param("plan_endeffector", plan_eef, plan_eef);

    /* Set ranges */
    s1_pos.x() = 0.0639686;
    s1_pos.y() = -0.259257;
    s1_pos.z() = 0.390049;
    range_one = 0.489;
    range_two = 0.969;
    next1 = 1151752134u;
    next2 = 2070363486u;

    /* Initialize the frontier properties to be observed */
    frtprp_obsv.frt_node.x() = 0;
    frtprp_obsv.frt_node.y() = 0;
    frtprp_obsv.frt_node.z() = 0;

    /* Set publisher of visual normals */
    normal_pub = m_nh.advertise<visualization_msgs::MarkerArray>("normals", 1);
    /* Set publisher of frontier point */
    frtpt_pub = m_nh.advertise<visualization_msgs::Marker>("frontier_point", 1);
    /* Listen to transformation of sensor frame relative to right hand camera frame */
    tf::TransformListener m_tfListener;             // TF listener
    tf::StampedTransform sensorToRhcTf;
    try {
        m_tfListener.waitForTransform(plan_eef, "/camera_rgb_optical_frame", ros::Time::now(), ros::Duration(3.0));
        m_tfListener.lookupTransform(plan_eef, "/camera_rgb_optical_frame", ros::Time(0), sensorToRhcTf);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ROS_ERROR_STREAM("Can't get transformation of sensor frame relative to " << plan_eef << " frame");
    }
    /* Convert the tf transform into a Eigen Affine3d */
    tf::transformTFToEigen(sensorToRhcTf, TsensorTorhc);
};

void NBVStrategy::FrtNearTracker(PointCloud &cloud, DistPoints &distpoints, int &cand_num, octomap::point3d origin, geometry_msgs::Pose last_view, float portion, FrtNb frt_nb)
{
    /* Temporary point pairs assignment */
    DistPoints temp_distpt(distpoints);
    /* Sort point pairs */
    doSort(temp_distpt);
    /* Convert sensor origin from octomap::point3d type to Eigen::Vector3f */
    Eigen::Vector3f sensorOrigin(origin.x(), origin.y(), origin.z());
    /* Create a normal estimation class */
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimate;
    PointCloud::Ptr m_cloud(new PointCloud(cloud));    // Pointer to input pointcloud
    normal_estimate.setInputCloud(m_cloud);            // Assign input pointcloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr m_kdtree(new pcl::search::KdTree<pcl::PointXYZ> ());        // Create kdtree of input pointcloud to search the K nearest neighbors for normal estimation
    m_kdtree->setInputCloud(m_cloud);
    normal_estimate.setSearchMethod(m_kdtree);         // Use kdtree to search point
    int k = 10;                                        // k nearest neighbor search
    /* Visualization makers for normals */
    visualization_msgs::MarkerArray normalVis;
    normalVis.markers.resize(cand_num);                               // Resize normal marker array
    /* Select top N (cand_num) nearest frontier */
    unsigned int num_distpt(temp_distpt.size());       // Number of frontier point pairs
    unsigned int step = portion * num_distpt / cand_num;   // Top 30% points divided by candidate number
    nbvCands.clear();                                     // Clear next views
    nbvCands.resize(cand_num);                            // Set next views size as candidate number
    for (int i = 0; i < cand_num; i++) {
        octomap::point3d near_frt = temp_distpt[step * i].first;                    // Near frontier point
        FrtNb::iterator it = std::find_if(frt_nb.begin(), frt_nb.end(), find_nb(*this, near_frt));
        octomap::point3d frt_nb(it->second);
        nbvCands[i].first = near_frt;
        nbvCands[i].second.pose.position = octomap::pointOctomapToMsg(near_frt);                // Assign near frontier point to position candidate
        /* Normal estimation at this point */
        pcl::PointXYZ near_frt_pcl(near_frt.x(), near_frt.y(), near_frt.z());       // Convert octomap point to pcl point
        std::vector<int> pointIdxNKNSearch(k);                                      // the resultant indicies of the neighboring points
        std::vector<float> pointNKNSquaredDistance(k);                              // the resultant squared distances to the neighboring points
        if (m_kdtree->nearestKSearch(near_frt_pcl, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            Eigen::Vector4f plane_param_;            // Plane parameter holding the normal on the first 3 coordinates
            float curvature_;                        // Output surface curvature
            normal_estimate.computePointNormal(cloud, pointIdxNKNSearch, plane_param_, curvature_);     // Compute normal at this point
            Eigen::Vector3f normal(plane_param_[0], plane_param_[1], plane_param_[2]);                  // Normal
            Eigen::Vector3f frt(near_frt.x(), near_frt.y(), near_frt.z());                              // Convert nearest frontier point from octomap::point3d to Eigen::Vector3f
            /* Flip normal if normal directs to known space */
            Eigen::Vector3f frtnb_eig(frt_nb.x(), frt_nb.y(), frt_nb.z());
            Eigen::Vector3f frt_unkn(frtnb_eig - frt);
            if (normal.dot(frt_unkn) < 0) {
                normal = normal * -1;
            }
            normal.normalize();                                                                         // Normalize vector
            /* Calculate position candidate */
            Eigen::Vector3f position_cand;
            position_cand = frt - lower_limit * normal;
            if (std::abs(last_view.position.x - position_cand.x()) <= 0.02 &&
                    std::abs(last_view.position.y - position_cand.y()) <= 0.02 &&
                    std::abs(last_view.position.z - position_cand.z()) <= 0.02) {
                position_cand =  frt - 0.5 * normal;
                ROS_INFO("Consider sensor lower limit");
            }
            nbvCands[i].second.header.frame_id = m_WorldFrame;
            nbvCands[i].second.pose.position.x = position_cand.x();
            nbvCands[i].second.pose.position.y = position_cand.y();
            nbvCands[i].second.pose.position.z = position_cand.z();
            /* Normal for visualization*/
            Eigen::Vector3f vis_orient_identity(1, 0, 0);                                               // Identity orientation in RVIZ
            Eigen::Quaternion<float> vis_orient;                                                        // Quaternion for visualization
            vis_orient.setFromTwoVectors(vis_orient_identity, normal);
            /* Convert Eigen quaternion to geometry_msgs quaternion */
            normalVis.markers[i].pose.orientation.x = vis_orient.x();
            normalVis.markers[i].pose.orientation.y = vis_orient.y();
            normalVis.markers[i].pose.orientation.z = vis_orient.z();
            normalVis.markers[i].pose.orientation.w = vis_orient.w();
            normalVis.markers[i].pose.position = nbvCands[i].second.pose.position;
            /* Normal for motion planning */
            Eigen::Vector3f mp_orient_identity(0, 0, 1);                                                // Calculate rotation transformation between z axes
            Eigen::Quaternion<float> mp_orient;                                                         // Quaternion for motion planning
            mp_orient.setFromTwoVectors(mp_orient_identity, normal);
            nbvCands[i].second.pose.orientation.x = mp_orient.x();
            nbvCands[i].second.pose.orientation.y = mp_orient.y();
            nbvCands[i].second.pose.orientation.z = mp_orient.z();
            nbvCands[i].second.pose.orientation.w = mp_orient.w();
            /* OUTPUT TEST */
            /*
               ROS_INFO_STREAM("Position - x: " << nbvCands[i].pose.position.x
               << " y: " << nbvCands[i].pose.position.y
               << " z: " << nbvCands[i].pose.position.z);
               ROS_INFO_STREAM("Orientation -x: " << nbvCands[i].pose.orientation.x
               << " y: " << nbvCands[i].pose.orientation.y
               << " z: " << nbvCands[i].pose.orientation.z
               << " w: " << nbvCands[i].pose.orientation.w);*/
        }
    }
    /* Calculate right_hand_camera next pose */
    //RHCPose();
    /* Set color for visual normals */
    std_msgs::ColorRGBA normal_color;
    normal_color.r = 1;
    normal_color.g = 0;
    normal_color.b = 0;
    normal_color.a = 1;
    /* Set scale for visual normals */
    geometry_msgs::Vector3 normal_scale;
    normal_scale.x = 0.4;
    normal_scale.y = 0.04;
    normal_scale.z = 0.04;
    uint8_t normal_shape = visualization_msgs::Marker::ARROW;
    visualization::dispMkarr(normalVis, normal_shape, normal_color, normal_scale, m_WorldFrame);
    normal_pub.publish(normalVis);
}

void NBVStrategy::ManeuverContinue(PointCloud &cloud, FrtPrp &areaone, FrtPrp &areatwo, FrtPrp &areathree, int &cand_num, octomap::point3d origin, float portion)
{
    /* Sort three frontier area in order of distance to sensor origin */
    sortFrtprp(areaone);
    sortFrtprp(areatwo);
    sortFrtprp(areathree);
    /* Merge the three struct vector */
    FrtPrp mansort_frtprp;
    mansort_frtprp.reserve(areaone.size() + areatwo.size() + areathree.size());
    mansort_frtprp.insert(mansort_frtprp.end(), areaone.begin(), areaone.end());
    mansort_frtprp.insert(mansort_frtprp.end(), areatwo.begin(), areatwo.end());
    mansort_frtprp.insert(mansort_frtprp.end(), areathree.begin(), areathree.end());

    /* Convert sensor origin from octomap::point3d type to Eigen::Vector3f */
    Eigen::Vector3f sensorOrigin(origin.x(), origin.y(), origin.z());
    /* Create a normal estimation class */
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimate;
    PointCloud::Ptr m_cloud(new PointCloud(cloud));    // Pointer to input pointcloud
    normal_estimate.setInputCloud(m_cloud);            // Assign input pointcloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr m_kdtree(new pcl::search::KdTree<pcl::PointXYZ> ());        // Create kdtree of input pointcloud to search the K nearest neighbors for normal estimation
    m_kdtree->setInputCloud(m_cloud);
    normal_estimate.setSearchMethod(m_kdtree);         // Use kdtree to search point
    int k = 10;                                        // k nearest neighbor search
    /* Visualization makers for normals */
    visualization_msgs::MarkerArray normalVis;
    normalVis.markers.resize(cand_num);                               // Resize normal marker array
    /* Select top N (cand_num) nearest frontier */
    unsigned int num_distpt(mansort_frtprp.size());       // Number of frontier point pairs
    unsigned int step = portion * num_distpt / cand_num;   // Top 30% points divided by candidate number
    nbvCands.clear();                                     // Clear next views
    nbvCands.resize(cand_num);                            // Set next views size as candidate number
    for (int i = 0; i < cand_num; i++) {
        octomap::point3d near_frt = mansort_frtprp[step * i].frt_node;                    // Near frontier point
        octomap::point3d frt_nb(mansort_frtprp[step * i].unk_nb);
        nbvCands[i].second.pose.position = octomap::pointOctomapToMsg(near_frt);                // Assign near frontier point to position candidate
        /* Normal estimation at this point */
        pcl::PointXYZ near_frt_pcl(near_frt.x(), near_frt.y(), near_frt.z());       // Convert octomap point to pcl point
        std::vector<int> pointIdxNKNSearch(k);                                      // the resultant indicies of the neighboring points
        std::vector<float> pointNKNSquaredDistance(k);                              // the resultant squared distances to the neighboring points
        if (m_kdtree->nearestKSearch(near_frt_pcl, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            Eigen::Vector4f plane_param_;            // Plane parameter holding the normal on the first 3 coordinates
            float curvature_;                        // Output surface curvature
            normal_estimate.computePointNormal(cloud, pointIdxNKNSearch, plane_param_, curvature_);     // Compute normal at this point
            Eigen::Vector3f normal(plane_param_[0], plane_param_[1], plane_param_[2]);                  // Normal
            Eigen::Vector3f frt(near_frt.x(), near_frt.y(), near_frt.z());                              // Convert nearest frontier point from octomap::point3d to Eigen::Vector3f
            /* Flip normal if normal directs to known space */
            Eigen::Vector3f frtnb_eig(frt_nb.x(), frt_nb.y(), frt_nb.z());
            Eigen::Vector3f frt_unkn(frtnb_eig - frt);
            if (normal.dot(frt_unkn) < 0) {
                normal = normal * -1;
            }
            normal.normalize();                                                                         // Normalize vector
            /* Calculate position candidate */
            Eigen::Vector3f position_cand;
            if (!mansort_frtprp[step * i].isvoid) {
                position_cand = frt - lower_limit * normal;
            }
            else {
                position_cand = frt - 0.5 * normal;
            }
            nbvCands[i].second.header.frame_id = m_WorldFrame;
            nbvCands[i].second.pose.position.x = position_cand.x();
            nbvCands[i].second.pose.position.y = position_cand.y();
            nbvCands[i].second.pose.position.z = position_cand.z();
            /* Normal for visualization*/
            Eigen::Vector3f vis_orient_identity(1, 0, 0);                                               // Identity orientation in RVIZ
            Eigen::Quaternion<float> vis_orient;                                                        // Quaternion for visualization
            vis_orient.setFromTwoVectors(vis_orient_identity, normal);
            /* Convert Eigen quaternion to geometry_msgs quaternion */
            normalVis.markers[i].pose.orientation.x = vis_orient.x();
            normalVis.markers[i].pose.orientation.y = vis_orient.y();
            normalVis.markers[i].pose.orientation.z = vis_orient.z();
            normalVis.markers[i].pose.orientation.w = vis_orient.w();
            normalVis.markers[i].pose.position = nbvCands[i].second.pose.position;
            /* Normal for motion planning */
            Eigen::Vector3f mp_orient_identity(0, 0, 1);                                                // Calculate rotation transformation between z axes
            Eigen::Quaternion<float> mp_orient;                                                         // Quaternion for motion planning
            mp_orient.setFromTwoVectors(mp_orient_identity, normal);
            nbvCands[i].second.pose.orientation.x = mp_orient.x();
            nbvCands[i].second.pose.orientation.y = mp_orient.y();
            nbvCands[i].second.pose.orientation.z = mp_orient.z();
            nbvCands[i].second.pose.orientation.w = mp_orient.w();
        }
    }
    /* Calculate right_hand_camera next pose */
    //RHCPose();
    /* Set color for visual normals */
    std_msgs::ColorRGBA normal_color;
    normal_color.r = 1;
    normal_color.g = 0;
    normal_color.b = 0;
    normal_color.a = 1;
    /* Set scale for visual normals */
    geometry_msgs::Vector3 normal_scale;
    normal_scale.x = 0.4;
    normal_scale.y = 0.04;
    normal_scale.z = 0.04;
    uint8_t normal_shape = visualization_msgs::Marker::ARROW;
    visualization::dispMkarr(normalVis, normal_shape, normal_color, normal_scale, m_WorldFrame);
    normal_pub.publish(normalVis);
}

void NBVStrategy::ContInfogain(PointCloud &cloud, FrtPrp &frtprps, int &cand_num, octomap::point3d origin)
{
    //ScoreAssign(frtprps);
    /* Convert sensor origin from octomap::point3d type to Eigen::Vector3f */
    Eigen::Vector3f sensorOrigin(origin.x(), origin.y(), origin.z());
    /* Create a normal estimation class */
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimate;
    PointCloud::Ptr m_cloud(new PointCloud(cloud));    // Pointer to input pointcloud
    normal_estimate.setInputCloud(m_cloud);            // Assign input pointcloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr m_kdtree(new pcl::search::KdTree<pcl::PointXYZ> ());        // Create kdtree of input pointcloud to search the K nearest neighbors for normal estimation
    m_kdtree->setInputCloud(m_cloud);
    normal_estimate.setSearchMethod(m_kdtree);         // Use kdtree to search point
    int k = 10;                                        // k nearest neighbor search
    /* Visualization makers for normals */
    visualization_msgs::MarkerArray normalVis;
    normalVis.markers.resize(cand_num);                               // Resize normal marker array
    /* Select top N (cand_num) nearest frontier */
    nbvCands.clear();                                     // Clear next views
    nbvCands.resize(cand_num);                            // Set next views size as candidate number
    for (int i = 0; i < cand_num; i++) {
        octomap::point3d near_frt = frtprps[i].frt_node;                    // Near frontier point
        octomap::point3d frt_nb(frtprps[i].unk_nb);
        nbvCands[i].second.pose.position = octomap::pointOctomapToMsg(near_frt);                // Assign near frontier point to position candidate
        /* Normal estimation at this point */
        pcl::PointXYZ near_frt_pcl(near_frt.x(), near_frt.y(), near_frt.z());       // Convert octomap point to pcl point
        std::vector<int> pointIdxNKNSearch(k);                                      // the resultant indicies of the neighboring points
        std::vector<float> pointNKNSquaredDistance(k);                              // the resultant squared distances to the neighboring points
        if (m_kdtree->nearestKSearch(near_frt_pcl, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            Eigen::Vector4f plane_param_;            // Plane parameter holding the normal on the first 3 coordinates
            float curvature_;                        // Output surface curvature
            normal_estimate.computePointNormal(cloud, pointIdxNKNSearch, plane_param_, curvature_);     // Compute normal at this point
            Eigen::Vector3f normal(plane_param_[0], plane_param_[1], plane_param_[2]);                  // Normal
            Eigen::Vector3f frt(near_frt.x(), near_frt.y(), near_frt.z());                              // Convert nearest frontier point from octomap::point3d to Eigen::Vector3f
            /* Flip normal if normal directs to known space */
            Eigen::Vector3f frtnb_eig(frt_nb.x(), frt_nb.y(), frt_nb.z());
            Eigen::Vector3f frt_unkn(frtnb_eig - frt);
            if (normal.dot(frt_unkn) < 0) {
                normal = normal * -1;
            }
            normal.normalize();                                                                         // Normalize vector
            /* Calculate position candidate */
            Eigen::Vector3f position_cand;
            if (!frtprps[i].isvoid) {
                position_cand = frt - lower_limit * normal;
            }
            else {
                position_cand = frt - 0.5 * normal;
            }
            nbvCands[i].second.header.frame_id = m_WorldFrame;
            nbvCands[i].second.pose.position.x = position_cand.x();
            nbvCands[i].second.pose.position.y = position_cand.y();
            nbvCands[i].second.pose.position.z = position_cand.z();
            /* Normal for visualization*/
            Eigen::Vector3f vis_orient_identity(1, 0, 0);                                               // Identity orientation in RVIZ
            Eigen::Quaternion<float> vis_orient;                                                        // Quaternion for visualization
            vis_orient.setFromTwoVectors(vis_orient_identity, normal);
            /* Convert Eigen quaternion to geometry_msgs quaternion */
            normalVis.markers[i].pose.orientation.x = vis_orient.x();
            normalVis.markers[i].pose.orientation.y = vis_orient.y();
            normalVis.markers[i].pose.orientation.z = vis_orient.z();
            normalVis.markers[i].pose.orientation.w = vis_orient.w();
            normalVis.markers[i].pose.position = nbvCands[i].second.pose.position;
            /* Normal for motion planning */
            Eigen::Vector3f mp_orient_identity(0, 0, 1);                                                // Calculate rotation transformation between z axes
            Eigen::Quaternion<float> mp_orient;                                                         // Quaternion for motion planning
            mp_orient.setFromTwoVectors(mp_orient_identity, normal);
            nbvCands[i].second.pose.orientation.x = mp_orient.x();
            nbvCands[i].second.pose.orientation.y = mp_orient.y();
            nbvCands[i].second.pose.orientation.z = mp_orient.z();
            nbvCands[i].second.pose.orientation.w = mp_orient.w();
        }
    }
    /* Calculate right_hand_camera next pose */
    //RHCPose();
    /* Set color for visual normals */
    std_msgs::ColorRGBA normal_color;
    normal_color.r = 1;
    normal_color.g = 0;
    normal_color.b = 0;
    normal_color.a = 1;
    /* Set scale for visual normals */
    geometry_msgs::Vector3 normal_scale;
    normal_scale.x = 0.4;
    normal_scale.y = 0.04;
    normal_scale.z = 0.04;
    uint8_t normal_shape = visualization_msgs::Marker::ARROW;
    visualization::dispMkarr(normalVis, normal_shape, normal_color, normal_scale, m_WorldFrame);
    normal_pub.publish(normalVis);
}

void NBVStrategy::ContInfogainMulCands(PointCloud &cloud, FrtPrp &frtprps, octomap::point3d origin, double &propone, double &proptwo, double &propthree)
{
    ScoreAssign(frtprps, propone, proptwo, propthree);
    /* Convert sensor origin from octomap::point3d type to Eigen::Vector3f */
    Eigen::Vector3f sensorOrigin(origin.x(), origin.y(), origin.z());
    /* Create a normal estimation class */
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimate;
    PointCloud::Ptr m_cloud(new PointCloud(cloud));    // Pointer to input pointcloud
    normal_estimate.setInputCloud(m_cloud);            // Assign input pointcloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr m_kdtree(new pcl::search::KdTree<pcl::PointXYZ> ());        // Create kdtree of input pointcloud to search the K nearest neighbors for normal estimation
    m_kdtree->setInputCloud(m_cloud);
    normal_estimate.setSearchMethod(m_kdtree);         // Use kdtree to search point
    int k = 10;                                        // k nearest neighbor search
    octomap::point3d near_frt = frtprps[0].frt_node;                    // Near frontier point
    octomap::point3d frt_nb(frtprps[0].unk_nb);
    /* Normal estimation at this point */
    pcl::PointXYZ near_frt_pcl(near_frt.x(), near_frt.y(), near_frt.z());       // Convert octomap point to pcl point
    std::vector<int> pointIdxNKNSearch(k);                                      // the resultant indicies of the neighboring points
    std::vector<float> pointNKNSquaredDistance(k);                              // the resultant squared distances to the neighboring points
    Eigen::Vector3f normal;
    if (m_kdtree->nearestKSearch(near_frt_pcl, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        Eigen::Vector4f plane_param_;            // Plane parameter holding the normal on the first 3 coordinates
        float curvature_;                        // Output surface curvature
        normal_estimate.computePointNormal(cloud, pointIdxNKNSearch, plane_param_, curvature_);     // Compute normal at this point
        normal.x() = plane_param_[0];
        normal.y() = plane_param_[1];
        normal.z() = plane_param_[2];                  // Normal
        Eigen::Vector3f frt(near_frt.x(), near_frt.y(), near_frt.z());                              // Convert nearest frontier point from octomap::point3d to Eigen::Vector3f
        /* Flip normal if normal directs to known space */
        Eigen::Vector3f frtnb_eig(frt_nb.x(), frt_nb.y(), frt_nb.z());
        Eigen::Vector3f frt_unkn(frtnb_eig - frt);
        // the direction is from point on sphere to frontier
        if (normal.dot(frt_unkn) < 0) {
            normal = normal * -1;
        }
        normal.normalize();                                                                         // Normalize vector
    }
    normal_obsv = normal;
    frtprp_obsv = frtprps[0];
}

void NBVStrategy::ComputePose(int &cand_num, bool &observed)
{
    /* Select top N (cand_num) nearest frontier */
    nbvCands.clear();                                     // Clear next views
    nbvCands.resize(cand_num);                            // Set next views size as candidate number
    /* Visualization makers for normals */
    visualization_msgs::MarkerArray normalVis;
    normalVis.markers.resize(cand_num);                               // Resize normal marker array
    for (int i = 0; i < cand_num; i++) {
        Eigen::Vector3f random_pt = RandomPointHemisphere(normal_obsv);
        //Eigen::Vector3f random_pt = RandomPointSphere();
        //ROS_INFO_STREAM("The x: " << random_pt.x() << " The y: " << random_pt.y() << " The z: " << random_pt.z());
        Eigen::Vector3f mp_orient_identity(0, 0, 1);                                                // Calculate rotation transformation between z axes
        Eigen::Quaternion<float> mp_orient;                                                         // Quaternion for motion planning
        mp_orient.setFromTwoVectors(mp_orient_identity, random_pt * -1);
        nbvCands[i].second.pose.orientation.x = mp_orient.x();
        nbvCands[i].second.pose.orientation.y = mp_orient.y();
        nbvCands[i].second.pose.orientation.z = mp_orient.z();
        nbvCands[i].second.pose.orientation.w = mp_orient.w();
        /* Normal for visualization*/
        Eigen::Vector3f vis_orient_identity(1, 0, 0);                                               // Identity orientation in RVIZ
        Eigen::Quaternion<float> vis_orient;                                                        // Quaternion for visualization
        vis_orient.setFromTwoVectors(vis_orient_identity, random_pt * -1);
        /* Convert Eigen quaternion to geometry_msgs quaternion */
        normalVis.markers[i].pose.orientation.x = vis_orient.x();
        normalVis.markers[i].pose.orientation.y = vis_orient.y();
        normalVis.markers[i].pose.orientation.z = vis_orient.z();
        normalVis.markers[i].pose.orientation.w = vis_orient.w();

        if (observed) {
             if (!frtprp_obsv.isvoid) {
                random_pt = random_pt * lower_limit;
            }
            else {
                random_pt = random_pt * 0.5;
            }
        }
        else {
            random_pt = random_pt * 0.5;
        }
        nbvCands[i].second.header.frame_id = m_WorldFrame;
        nbvCands[i].second.pose.position.x = random_pt.x() + frtprp_obsv.frt_node.x();
        nbvCands[i].second.pose.position.y = random_pt.y() + frtprp_obsv.frt_node.y();
        nbvCands[i].second.pose.position.z = random_pt.z() + frtprp_obsv.frt_node.z();

        normalVis.markers[i].pose.position = nbvCands[i].second.pose.position;
    }
    /* Calculate right_hand_camera next pose */
    //RHCPose();
    /* Set color for visual normals */
    std_msgs::ColorRGBA normal_color;
    normal_color.r = 1;
    normal_color.g = 0;
    normal_color.b = 0;
    normal_color.a = 1;
    /* Set scale for visual normals */
    geometry_msgs::Vector3 normal_scale;
    normal_scale.x = 0.4;
    normal_scale.y = 0.04;
    normal_scale.z = 0.04;
    uint8_t normal_shape = visualization_msgs::Marker::ARROW;
    visualization::dispMkarr(normalVis, normal_shape, normal_color, normal_scale, m_WorldFrame);
    normal_pub.publish(normalVis);

    /* Display the frontier point to be observed */
    visualization_msgs::Marker frtptVis;
    frtptVis.pose.position.x = frtprp_obsv.frt_node.x();
    frtptVis.pose.position.y = frtprp_obsv.frt_node.y();
    frtptVis.pose.position.z = frtprp_obsv.frt_node.z();
    std_msgs::ColorRGBA frtpt_color;
    frtpt_color.a = 1.0;
    frtpt_color.r = 1.0;
    frtpt_color.g = 1.0;
    frtpt_color.b = 0.0;
    geometry_msgs::Vector3 frtpt_scale;
    frtpt_scale.x = 0.01;
    frtpt_scale.y = 0.01;
    frtpt_scale.z = 0.01;
    visualization::dispPoint(frtptVis, frtpt_color, frtpt_scale, m_WorldFrame);
    frtpt_pub.publish(frtptVis);
}

void NBVStrategy::ScoreAssign(FrtPrp &frtprps, double &propone, double &proptwo, double &propthree)
{
    double max_dist = (std::max_element(frtprps.begin(), frtprps.end(), maxmin_dist(*this)))->sensor_frt;
    double min_dist = (std::min_element(frtprps.begin(), frtprps.end(), maxmin_dist(*this)))->sensor_frt;
    for (FrtPrp::iterator it = frtprps.begin(); it != frtprps.end(); it++)
    {
        if (it->frt_node.y() < -0.22 && it->frt_node.distance(s1_pos) < range_one) {
            if (propone < 0.97) {
             if (!it->isvoid) {
                it->avg_score = weight_ig * score_one;
            }
            else {
                it->avg_score = 0;
            }
            }
            else {
                it->avg_score = 0;
            }

        }
        else if (it->frt_node.y() >= -0.22 && it->frt_node.distance(s1_pos) < range_one || it->frt_node.distance(s1_pos) < range_two) {
            if (proptwo < 0.95) {
             if (!it->isvoid) {
                it->avg_score = weight_ig * score_two;
            }
            else {
                it->avg_score = 0;
            }
            }
            else {
                it->avg_score = 0;
            }

        }
        else {
            if (propthree < 0.9) {
             if (!it->isvoid) {
                it->avg_score = weight_ig * score_three;
            }
            else {
                it->avg_score = 0;
            }
            }
            else {
                it->avg_score = 0;
            }

        }
        double score_cont = (max_dist - it->sensor_frt) / (max_dist - min_dist) * 100;
        it->avg_score += weight_cont * score_cont;
    }
    std::sort(frtprps.begin(), frtprps.end(), comp_score(*this));
}

Eigen::Vector3f NBVStrategy::RandomPointSphere()
{
    float x0, x1, x2, x3, d2;
    do {
        x0 = U_m1_p1();
        x1 = U_m1_p1();
        x2 = U_m1_p1();
        x3 = U_m1_p1();
        d2 = x0*x0+x1*x1+x2*x2+x3*x3;
    } while (d2 > 1.0f);
    float scale = 1.0f / d2;
    return Eigen::Vector3f(2*(x1*x3+x0*x2)*scale, 2*(x2*x3+x0*x1)*scale, (x0*x0+x3*x3-x1*x1-x2*x2)*scale);
}

Eigen::Vector3f NBVStrategy::RandomPointHemisphere(Eigen::Vector3f const &v)
{
    Eigen::Vector3f result = RandomPointSphere();
    if (result.dot(v) > 0) {
        result.x() = -result.x();
        result.y() = -result.y();
        result.z() = -result.z();
    }
    return result;
}

unsigned NBVStrategy::my_rand(void)
{
    next1 = next1 * 1701532575u + 571550083u;
    next2 = next2 * 3145804233u + 4178903934u;
    return (next1 << 16) ^ next2;
}

float NBVStrategy::U_m1_p1()
{
    return float(my_rand() * (1.0f / 2147483648.0f)) - 1.0f;
}

void NBVStrategy::sortFrtprp(FrtPrp &frtprp)
{
    std::sort(frtprp.begin(), frtprp.end(), comp_dist(*this));
}

void NBVStrategy::doSort(DistPoints &distpoints)
{
    std::sort(distpoints.begin(), distpoints.end(), comp_dist(*this));
}


void NBVStrategy::RHCPose()
{
    /* Clear */
    RHCnbvCands.clear();
    /* Resize */
    RHCnbvCands.resize(nbvCands.size());
    for (int i = 0; i < nbvCands.size(); i++)
    {
        /* Pose candidate for sensor in Eigen matrix */
        Eigen::Affine3d nbvCand_eigen;
        /* Convert Pose message to Eigen Transform */
        tf::poseMsgToEigen(nbvCands[i].second.pose, nbvCand_eigen);
        /* Pose candidate for right_hand_camera in Eigen matrix */
        Eigen::Affine3d RHCnbvCand_eigen;
        /* Calculate right_hand_camera pose */
        RHCnbvCand_eigen = nbvCand_eigen * TsensorTorhc.inverse();
        /* Convert Eigen pose to Message pose */
        tf::poseEigenToMsg(RHCnbvCand_eigen, RHCnbvCands[i].second.pose);
        /* Assign pose header frame_id */
        RHCnbvCands[i].second.header.frame_id = m_WorldFrame;
        /* Data validation test */
        //ROS_INFO_STREAM("Frame id is: " << RHCnbvCands[i].header.frame_id);
    }
}
