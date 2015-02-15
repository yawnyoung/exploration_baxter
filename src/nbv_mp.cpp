/******************************************************
 * This program is integration of two modules: NBV and
 * Virtual Arm Navigation. The reason for integration
 * is to guarantee the simultaneity.
 *****************************************************/

#include "ros/ros.h"

#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/GetPlanningScene.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "NBV_MP");
    ros::NodeHandle nh;

    // Create Service Client to obtain Planning Scene
    ros::ServiceClient planning_scene_service = nh.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    moveit_msgs::GetPlanningScene ps_srv;
    ps_srv.request.components.components =
        ps_srv.request.components.SCENE_SETTINGS |
        ps_srv.request.components.ROBOT_STATE |
        ps_srv.request.components.ROBOT_STATE_ATTACHED_OBJECTS |
        ps_srv.request.components.WORLD_OBJECT_NAMES |
        ps_srv.request.components.WORLD_OBJECT_GEOMETRY |
        ps_srv.request.components.OCTOMAP |
        ps_srv.request.components.TRANSFORMS |
        ps_srv.request.components.ALLOWED_COLLISION_MATRIX |
        ps_srv.request.components.LINK_PADDING_AND_SCALING |
        ps_srv.request.components.OBJECT_COLORS;

    // Make sure client is connected to server
    if (!planning_scene_service.exists()) {
        ROS_DEBUG("Waiting for service: 'get_planning_scene' to exist.");
        planning_scene_service.waitForExistence(ros::Duration(5.0));
    }
    if (planning_scene_service.call(ps_srv)) {
        ROS_INFO("Call service successfully!");
    }


    return 0;
}
