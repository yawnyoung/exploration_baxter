
/******************************************************************************
 * Functions: To get and monitor the 3D environment information (pointcloud);
 *            To convert this information to octomap format;
 *            To publish self-filtered octomap on the topic.
 *****************************************************************************/

#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>

#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ExplSensingMonitor");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(10.0)));
    // Octomap monitor
    boost::scoped_ptr<occupancy_map_monitor::OccupancyMapMonitor> octomap_monitor;
    // octomap_frame
    //const std::string map_frame = "base";
    if (!octomap_monitor) {
        octomap_monitor.reset(new occupancy_map_monitor::OccupancyMapMonitor(tf));
        // must set???
    }
    octomap_monitor->startMonitor();

    ros::waitForShutdown();
    return 0;
}
