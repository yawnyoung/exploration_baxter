/*************************************************
 * TEST for CLASSES...
 ************************************************/
#include "exploration_baxter/dir_vec.h"
#include "exploration_baxter/nbv_module.h"

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

// display on rviz
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/PoseStamped.h>
//#include <moveit_msgs/Constraints.h>
#include <moveit/kinematic_constraints/utils.h>

#include <sensor_msgs/JointState.h>

// global variable of octree
octomap::OcTree* ot;
// global robot state (joint states)
moveit_msgs::RobotState rt_state;
// Callback function to obtain octomap_msgs.
void otCB (const octomap_msgs::OctomapConstPtr& msg);
void RobotStateCB(const sensor_msgs::JointStateConstPtr& msg);

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "Test");
    ros::NodeHandle nh("~");

    std::string ot_topic = "/octomap_binary";
    ros::Subscriber ot_sub = nh.subscribe(ot_topic, 1, otCB);

    std::string robot_state_topic = "/robot/joint_states";
    ros::Subscriber robot_state_sub = nh.subscribe(robot_state_topic, 1, RobotStateCB);

    ros::Rate wait(10);
    while (!ot) {
        ROS_INFO("Waiting for octomap message .......");
        ros::spinOnce();
        wait.sleep();
    }

    octomap::point3d obsrvbbxmin, obsrvbbxmax, freebbxmin, freebbxmax;


    planning_interface::MotionPlanRequest request;
    geometry_msgs::PoseStamped my_pose;
    my_pose.header.frame_id = "/base";
    my_pose.pose.position.x = -0.5;
    my_pose.pose.position.y = -0.5;
    my_pose.pose.position.z = 0.280;
    my_pose.pose.orientation.x = 0.667;
    my_pose.pose.orientation.y = 0.663;
    my_pose.pose.orientation.z = -0.250;
    my_pose.pose.orientation.w = -0.233;

    // tolerance of 0.01(m) is specified in position
    // and 0.01(radians) in orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.1);

    request.group_name = "right_arm";

    //moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(n, my_pose);
    //request.goal_constraints.push_back(pose_goal);
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
            nbv_module->explr_bd(rt_state);
            moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(nbv_module->end_effector_name, my_pose);
            request.goal_constraints.push_back(pose_goal);
            //nbv_module->nearestfrt();
            nbv_module->test_mp(request);
            rt_state.joint_state.name.clear();
            rt_state.joint_state.position.clear();
            request.goal_constraints.clear();
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

void RobotStateCB(const sensor_msgs::JointStateConstPtr& msg)
{
    for (int i = 9; i < 16; i++)
    {
        rt_state.joint_state.name.push_back(msg->name[i]);
        rt_state.joint_state.position.push_back(msg->position[i]);
    }
}
