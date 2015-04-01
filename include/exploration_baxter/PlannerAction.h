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

typedef std::vector<geometry_msgs::Pose> views;

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
    /* Planner plugin name */
    std::string planner_plugin_name;
    /* Motion plan group */
    std::string plan_group;
    public:
        PlannerAction(ros::NodeHandle nh);
        void motionPlan(views &view_cand, octomap::OcTree ot_map);
};
