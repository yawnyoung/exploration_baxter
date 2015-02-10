/****************************************************
 * This program is the MAIN PROCESS for the BAXTER
 * EXPLORATION PROJECT.
 ***************************************************/

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Exploration");
    ros::NodeHandle nh;

    // Create one thread
    ros::AsyncSpinner spinner(1);
    spinner.start();



    return 0;
}
