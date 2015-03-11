/*************************************************
 * TEST for CLASSES...
 ************************************************/
#include "exploration_baxter/dir_vec.h"
#include "exploration_baxter/nbv_module.h"

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

    ros::Rate wait(10);
    while (!ot) {
        ROS_INFO("Waiting for octomap message .......");
        ros::spinOnce();
        wait.sleep();
    }

    octomap::point3d obsrvbbxmin, obsrvbbxmax, freebbxmin, freebbxmax;

    int region = 1;
    while (ros::ok() && region < 4)
    {
        if (region == 1) {
            // set octomap box boundary
            obsrvbbxmin.x() = -0.943;
            obsrvbbxmin.y() = -1.305;
            obsrvbbxmin.z() = -0.17;
            obsrvbbxmax.x() = -0.1;
            obsrvbbxmax.y() = -0.6;
            obsrvbbxmax.z() = 0.5;
            freebbxmin.x() = -0.943;
            freebbxmin.y() = -0.6;
            freebbxmin.z() = -0.2;
            freebbxmax.x() = -0.1;
            freebbxmax.y() = 0;
            freebbxmax.z() = 1;
        }
        else if (region == 2) {
            // set octomap box boundary
            obsrvbbxmin.x() = -0.943;
            obsrvbbxmin.y() = -1.305;
            obsrvbbxmin.z() = -0.17;
            obsrvbbxmax.x() = -0.1;
            obsrvbbxmax.y() = -0.6;
            obsrvbbxmax.z() = 0.5;
            freebbxmin.x() = -0.943;
            freebbxmin.y() = -0.6;
            freebbxmin.z() = -0.2;
            freebbxmax.x() = -0.1;
            freebbxmax.y() = 0;
            freebbxmax.z() = 1;
        }
        else if (region == 3) {
            // set octomap box boundary
            obsrvbbxmin.x() = -0.943;
            obsrvbbxmin.y() = -1.305;
            obsrvbbxmin.z() = -0.17;
            obsrvbbxmax.x() = -0.1;
            obsrvbbxmax.y() = -0.6;
            obsrvbbxmax.z() = 0.5;
            freebbxmin.x() = -0.943;
            freebbxmin.y() = -0.6;
            freebbxmin.z() = -0.2;
            freebbxmax.x() = -0.1;
            freebbxmax.y() = 0;
            freebbxmax.z() = 1;
        }
        else {
            ROS_ERROR("Region Number Error.");
        }

    // test for nbv_module
    NBVModule *nbv_module = new NBVModule(obsrvbbxmin, obsrvbbxmax, freebbxmin, freebbxmax, ot, nh);
    bool disp = true;

    ros::Rate r(1);

    while (ros::ok() && ot) {
        ros::spinOnce();
        if (nbv_module->update_nbv(ot)) {
            ROS_INFO("NBV Module updated!");
            nbv_module->trvs_tree();
            nbv_module->cal_dirvec(disp);
            nbv_module->explr_bd();
            //nbv_module.intersection();
            nbv_module->nearestfrt();
        }
        else {
            ROS_ERROR("Failed to update NBV Module!");
        }
        // whether or not terminate based on how many unknown nodes remain
        if (nbv_module->num_unknown < 6) {
            ROS_INFO_STREAM("Complete exploration of region " << region << ".");
            region++;
            delete nbv_module;
            break;
        }
        r.sleep();
    }
    }
    return 0;
}

void otCB (const octomap_msgs::OctomapConstPtr& msg)
{
    ot = octomap_msgs::binaryMsgToMap(*msg);
}
