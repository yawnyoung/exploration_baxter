#include "exploration_baxter/PlannerAction.h"

PlannerAction::PlannerAction(ros::NodeHandle nh):
    pa_nh(nh),
    robot_model_loader("robot_description"),
    plan_group("right_arm")                         // Set default planning group to be right_arm
{
    robot_model = robot_model_loader.getModel();
    planning_scene = boost::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(robot_model));
    /* SETUP THE PLANNER */
    if (!pa_nh.getParam("planning_plugin", planner_plugin_name)) {
        ROS_FATAL_STREAM("Could not find planner plugin name");
    }
    try {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    } catch (pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader" << ex.what());
    }
    try {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, pa_nh.getNamespace())) {
            ROS_FATAL_STREAM("Could not initialize planner instance");
        }
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    } catch (pluginlib::PluginlibException& ex) {
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); i++)
        {
            ss << classes[i] << " ";
        }
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "'" <<ex.what() << std::endl << "Available plugins: " << ss.str());
    }
    /* Get Parameters */
    pa_nh.param("plan_group", plan_group, plan_group);
}

void PlannerAction::motionPlan(views &view_cand, octomap::OcTree *ot_map)
{
    /* Configure planning scene */
    octomap_msgs::Octomap ot_msg;                               // Octomap message processed in planning scene
    if (octomap_msgs::fullMapToMsg(*ot_map, ot_msg)) {
        planning_scene->processOctomapMsg(ot_msg);
    }

    /* Motion plan request configuration */
    planning_interface::MotionPlanRequest req;
    req.group_name = plan_group;
    req.planner_id = planner_plugin_name;
    for (views::iterator cand_it = view_cand.begin(); cand_it != view_cand.end(); cand_it++)
    {

    }
}
