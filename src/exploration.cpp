/************************************************************
 * This program is the Planning Scene Server for the BAXTER
 * EXPLORATION PROJECT. It starts the "get_planning_scene"
 * service so that other nodes can obtain Planning Scene.
 * Actually, this is a kind of modification of move_group.
 ***********************************************************/

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/macros/console_colors.h>
#include <moveit/move_group/move_group_capability.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/tokenizer.hpp>

#include <tf/transform_listener.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string NODE_NAME = "Exploration";

namespace move_group {
    class CapabilityLoad {
        ros::NodeHandle node_handle_;
        MoveGroupContextPtr context_;
        boost::shared_ptr<pluginlib::ClassLoader<MoveGroupCapability> > capability_plugin_loader_;
        std::vector<boost::shared_ptr<MoveGroupCapability> > capabilities_;
        void configureCapabilities();
        public:
            CapabilityLoad(const planning_scene_monitor::PlanningSceneMonitorPtr& psm, bool debug);
            void status();
    };
    CapabilityLoad::CapabilityLoad(const planning_scene_monitor::PlanningSceneMonitorPtr& psm, bool debug):
        node_handle_("~")
    {
        ROS_INFO("enter CapabilityLoad");
        // if the user wants to be able to disable execution of paths, they can just set this ROS param to false
        bool allow_trajectory_execution;
        node_handle_.param("allow_trajectory_execution", allow_trajectory_execution, true);
        context_.reset(new MoveGroupContext(psm, allow_trajectory_execution, debug));
        ROS_INFO("ALLOW TRAJECTORY EXECUTION");

        // start the capabilities
        configureCapabilities();
    }
    void CapabilityLoad::status()
    {
        if (context_) {
            if (context_->status()) {
                if (capabilities_.empty()) {
                    printf(MOVEIT_CONSOLE_COLOR_CYAN "\nAll is well but no capabilities are loaded. There will be no party :(\n\n"MOVEIT_CONSOLE_COLOR_RESET);
                }
                else {
                    printf(MOVEIT_CONSOLE_COLOR_CYAN "\nAll is well! Everyone is happy! You can start planning now!\n\n" MOVEIT_CONSOLE_COLOR_RESET);
                }
                fflush(stdout);
            }
        }
        else
            ROS_ERROR("No MoveGroup context created. Nothing will work.");
    }
    void CapabilityLoad::configureCapabilities()
    {
        try {
            capability_plugin_loader_.reset(new pluginlib::ClassLoader<MoveGroupCapability>("moveit_ros_move_group", "move_group::MoveGroupCapability"));
        } catch (pluginlib::PluginlibException& ex) {
            ROS_FATAL_STREAM("Exception while creaing plugin loader for move_group capabilities: " << ex.what());
            return;
        }

        // add individual capabilities move_group supports
        std::string capability_plugins;
        if (node_handle_.getParam("capabilities", capability_plugins)) {
            boost::char_separator<char> sep(" ");
            boost::tokenizer<boost::char_separator<char> > tok(capability_plugins, sep);
            for (boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin(); beg != tok.end(); ++beg)
            {
                std::string plugin = *beg;
                try {
                    printf(MOVEIT_CONSOLE_COLOR_CYAN "Loading '%s'...\n" MOVEIT_CONSOLE_COLOR_RESET, plugin.c_str());
                    MoveGroupCapability* cap = capability_plugin_loader_->createUnmanagedInstance(plugin);
                    cap->setContext(context_);
                    cap->initialize();
                    capabilities_.push_back(boost::shared_ptr<MoveGroupCapability>(cap));
                } catch (pluginlib::PluginlibException& ex) {
                    ROS_ERROR_STREAM("Exception while loading move_group capability '" << plugin << "': " << ex.what() << std::endl
                                     << "Available capabilities: " << boost::algorithm::join(capability_plugin_loader_->getDeclaredClasses(), ", "));
                }
            }
            std::stringstream ss;
            ss << std::endl;
            ss << std::endl;
            ss << "***************************************************************" << std::endl;
            ss << "* Exploration using: " << std::endl;
            for (std::size_t i = 0; i < capabilities_.size(); i++) {
                ss << "*        - " << capabilities_[i]->getName() << std::endl;
            }
            ss << "****************************************************************" << std::endl;
            ROS_INFO_STREAM(ss.str());
        }
    }
    bool debug;
} // namespace move_group


int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);

    // Create one thread
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create a planning scene monitor and check whether the planning scene is configured or not.
    boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(10.0)));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, tf));

    if (!planning_scene_monitor->getPlanningScene()) {
        ROS_ERROR("Planning scene not configured");
    } else {
        ROS_INFO("Planning scene configured");
        move_group::debug = false;
        for (int i = 0; i < argc; i++) {
            if (strncmp(argv[i], "--debug", 7) == 0) {
                move_group::debug = true;
                break;
            }
        }
        if (move_group::debug) {
            ROS_INFO("Exploration debug mode is ON");
        }
        else {
            ROS_INFO("Exploration debug mode is OFF");
        }
    }

    printf(MOVEIT_CONSOLE_COLOR_CYAN "Starting context monitors...\n" MOVEIT_CONSOLE_COLOR_RESET);
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor();        // Start to monitor the octomap.
    planning_scene_monitor->startStateMonitor();
    printf(MOVEIT_CONSOLE_COLOR_CYAN "Context monitors started.\n" MOVEIT_CONSOLE_COLOR_RESET);

    move_group::CapabilityLoad cap_load(planning_scene_monitor, move_group::debug);

    ROS_INFO("CONFIGURE!");
    planning_scene_monitor->publishDebugInformation(move_group::debug);


    //planning_scene_monitor::LockedPlanningSceneRW planning_scene_locked(planning_scene_monitor);
    planning_scene::PlanningScenePtr ps = planning_scene_monitor->getPlanningScene();
    ros::Time begin = ros::Time::now();
    while (ros::ok() && ps)
    {
        //planning_scene::PlanningScenePtr ps = planning_scene_monitor->getPlanningScene();
        std::vector<std::string> obj_id = ps->getWorld()->getObjectIds();
        if (!obj_id.empty()) {
            ROS_INFO("Object get");
            ros::Duration elapsed_time = ros::Time::now() - begin;
            ROS_INFO_STREAM("Time elapsed: " << elapsed_time << " sec.");
            break;
        }
    }
    //planning_scene::PlanningScenePtr ps = planning_scene_monitor->getPlanningScene();
    std::vector<std::string> obj_id = ps->getWorld()->getObjectIds();
    std::cout << obj_id.size() << std::endl;
    for (std::vector<std::string>::iterator it = obj_id.begin(); it != obj_id.end(); ++it)
    {
        std::cout << *it << std::endl;
    }

    collision_detection::World::ObjectConstPtr obj_octm = ps->getWorld()->getObject(planning_scene::PlanningScene::OCTOMAP_NS);
    if (obj_octm) {
        ROS_INFO("OCTOMAP GET");
        const shapes::OcTree* ot_shape = dynamic_cast<const shapes::OcTree*>(obj_octm->shapes_[0].get());
        if (ot_shape) {
            ROS_INFO("Cast shape octree to octomap octree");
            boost::shared_ptr<const octomap::OcTree> ot = ot_shape->octree;
            for (unsigned int i = 0; i < 5; ++i)
            {
                std::string i_str;
                std::stringstream out;
                out << i;
                i_str = out.str();
                std::string FileName = "my_octomap" + i_str;
                FileName += ".ot";
                ot->write(FileName);
                ROS_INFO("WRITE!");
                ros::Duration(2).sleep();
            }
        } else {
            ROS_ERROR("Failed to cast shape octree to octomap octree");
        }
        //TODO plan!
    }
/*
    while (!obj) {
        ROS_INFO("OCTOMAP NOT GET");
        const shapes::OcTree* ot_shape = dynamic_cast<const shapes::OcTree*>(obj->shapes_[0].get());
        if (ot_shape) {
            ROS_INFO("Octomap get");
            boost::shared_ptr<const octomap::OcTree> ot = ot_shape->octree;
        }
    }
    ROS_INFO("OCTOMAP GET");*/

    ros::waitForShutdown();


    return 0;
}
