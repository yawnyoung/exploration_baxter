#include "exploration_baxter/NBVStrategy.h"

NBVStrategy::NBVStrategy(ros::NodeHandle nh):
    m_nh(nh),
    lower_limit(0.5),                       // Set default value to be 0.5 meter
    m_WorldFrame("/base"),                  // Set default world frame to be base frame
    plan_eef("/right_hand_camera")          // Set default plan end effector to be right_hand_camera
{
    /* Get Parameter */
    m_nh.param("sensor_model/lower_limit", lower_limit, lower_limit);
    m_nh.param("frame_id", m_WorldFrame, m_WorldFrame);
    //m_nh.param("plan_endeffector", plan_eef, plan_eef);
    /* Set publisher of visual normals */
    normal_pub = m_nh.advertise<visualization_msgs::MarkerArray>("normals", 1);
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
        nbvCands[i].pose.position = octomap::pointOctomapToMsg(near_frt);                // Assign near frontier point to position candidate
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
            Eigen::Vector3f origin_frt(frt - sensorOrigin);
            /* Flip normal if normal directs to inner space*/
            /*if (normal.dot(origin_frt) < 0) {
                normal= normal * -1;
            }*/
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
            nbvCands[i].header.frame_id = m_WorldFrame;
            nbvCands[i].pose.position.x = position_cand.x();
            nbvCands[i].pose.position.y = position_cand.y();
            nbvCands[i].pose.position.z = position_cand.z();
            /* Normal for visualization*/
            Eigen::Vector3f vis_orient_identity(1, 0, 0);                                               // Identity orientation in RVIZ
            Eigen::Quaternion<float> vis_orient;                                                        // Quaternion for visualization
            vis_orient.setFromTwoVectors(vis_orient_identity, normal);
            /* Convert Eigen quaternion to geometry_msgs quaternion */
            normalVis.markers[i].pose.orientation.x = vis_orient.x();
            normalVis.markers[i].pose.orientation.y = vis_orient.y();
            normalVis.markers[i].pose.orientation.z = vis_orient.z();
            normalVis.markers[i].pose.orientation.w = vis_orient.w();
            normalVis.markers[i].pose.position = nbvCands[i].pose.position;
            /* Normal for motion planning */
            Eigen::Vector3f mp_orient_identity(0, 0, 1);                                                // Calculate rotation transformation between z axes
            Eigen::Quaternion<float> mp_orient;                                                         // Quaternion for motion planning
            mp_orient.setFromTwoVectors(mp_orient_identity, normal);
            nbvCands[i].pose.orientation.x = mp_orient.x();
            nbvCands[i].pose.orientation.y = mp_orient.y();
            nbvCands[i].pose.orientation.z = mp_orient.z();
            nbvCands[i].pose.orientation.w = mp_orient.w();
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
        tf::poseMsgToEigen(nbvCands[i].pose, nbvCand_eigen);
        /* Pose candidate for right_hand_camera in Eigen matrix */
        Eigen::Affine3d RHCnbvCand_eigen;
        /* Calculate right_hand_camera pose */
        RHCnbvCand_eigen = nbvCand_eigen * TsensorTorhc.inverse();
        /* Convert Eigen pose to Message pose */
        tf::poseEigenToMsg(RHCnbvCand_eigen, RHCnbvCands[i].pose);
        /* Assign pose header frame_id */
        RHCnbvCands[i].header.frame_id = m_WorldFrame;
        /* Data validation test */
        //ROS_INFO_STREAM("Frame id is: " << RHCnbvCands[i].header.frame_id);
    }
}
