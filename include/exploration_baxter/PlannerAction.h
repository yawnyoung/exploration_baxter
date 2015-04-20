/*************************************************************
 * Introduction: Test candidates and take action
 ************************************************************/

#include "ros/ros.h"
#include <pluginlib/class_loader.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <sensor_msgs/JointState.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef std::vector<geometry_msgs::Pose> views;
typedef std::vector<geometry_msgs::PoseStamped> viewsStamped;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class PlannerAction {
    /* ros NodeHandle */
    ros::NodeHandle pa_nh;
    /* Robot model loader */
    robot_model_loader::RobotModelLoader robot_model_loader;
    /* Robot model */
    robot_model::RobotModelPtr robot_model;
    /* Planning scene */
    planning_scene::PlanningScenePtr planning_scene;
    /* Motion planner pluginlib loader in ros */
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
    /* Planner manager instance */
    planning_interface::PlannerManagerPtr planner_instance;
    /* Motion planning response */
    planning_interface::MotionPlanResponse res;
    /* Motion planning response message */
    moveit_msgs::MotionPlanResponse res_msg;
    /* Planner plugin name */
    std::string planner_plugin_name;
    /* Motion plan group */
    std::string plan_group;
    /* Planning link name */
    std::string link_name;
    /* Subscriber for listening to current robot state */
    ros::Subscriber robot_state_sub;
    /* Current Robot state message */
    moveit_msgs::RobotState robotstate_crr;
    /* Publisher for publishing the trajectory message */
    ros::Publisher display_pub;
    /* Publisher for planning scene */
    ros::Publisher ps_pub;
    /* Action client */
    TrajClient *traj_client;
    /* Action result subscriber */
    ros::Subscriber action_res_sub;

    /* Callback function to get robot_state */
    void RobotStateCB(const sensor_msgs::JointStatePtr& msg);
    /* Callback function to get action result */
    void ActionResultCB(const control_msgs::FollowJointTrajectoryActionResultPtr& msg);
    /**
     * @brief Visualize planned path
     */
    void VisPlanpath();

    public:
    /* The last view record */
    geometry_msgs::Pose last_view;
    /* Plan outcome flag */
    bool plan_out;
    /* Action result */
    int action_res;

    PlannerAction(ros::NodeHandle nh);
    void motionPlan(viewsStamped &view_cand, octomap::OcTree *ot_map);
    /**
     * @brief Action for motion planning
     */
    void ActPlan();
};
