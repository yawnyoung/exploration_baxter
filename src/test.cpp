/*************************************************
 * TEST for CLASSES...
 ************************************************/

#include "ros/ros.h"

#include "exploration_baxter/dir_vec.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

// display on rviz
#include <visualization_msgs/Marker.h>

// global variable of octree
octomap::OcTree* ot;
// Callback function to obtain octomap_msgs.
void otCB (const octomap_msgs::OctomapConstPtr& msg);

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "Test");
    ros::NodeHandle nh("~");

    std::string ot_topic = "/octomap_binary";
    ros::Subscriber ot_sub = nh.subscribe(ot_topic, 1, otCB);

    // publish arrow of direction to rviz
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    // set the shape type to be an arrow
    unsigned int shape = visualization_msgs::Marker::ARROW;
    // set the start point of arrow
    geometry_msgs::Point start;

    start.x = 0;
    start.y = 0;
    start.z = 0;
    // end point of arrow
    geometry_msgs::Point end;

    // publish octomap node to rviz
    //ros::Publisher occ_pub = nh.advertise<visualization_msgs::MarkerArray>("occupied_cells", 1);

    // set octomap box boundary
    octomap::point3d bbxmin(0.3, -0.5, -0.2);
    octomap::point3d bbxmax(0.6, -0.2, 0.2);
    CalDir cal_dir(bbxmin, bbxmax, nh);
    ROS_INFO("Create CalDir class.");

    // direction vector
    octomap::point3d direction;

    ros::Rate wait(10);
    while (!ot) {
        ROS_INFO("Waiting for octomap message .......");
        ros::spinOnce();
        wait.sleep();
    }

    ros::Rate r(1);

    while (ros::ok() && ot) {
        ros::spinOnce();
        direction = cal_dir.cal_dirvec_sensor(ot);
        /*
        start.x = cal_dir.sensor_origin.x();
        start.y = cal_dir.sensor_origin.y();
        start.z = cal_dir.sensor_origin.z();*/
        visualization_msgs::Marker marker;
        // set the frame id and timestamp
        marker.header.frame_id = "/camera_rgb_optical_frame";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        end.x = direction.x();
        end.y = direction.y();
        end.z = direction.z();
        marker.points.push_back(start);
        marker.points.push_back(end);
        marker.scale.x = 0.005;
        marker.scale.y = 0.005;
        marker.scale.z = 0.005;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        marker_pub.publish(marker);

        cal_dir.cell_vis();

        r.sleep();
    }
    return 0;
}

void otCB (const octomap_msgs::OctomapConstPtr& msg)
{
    ot = octomap_msgs::binaryMsgToMap(*msg);
}
