/******************************************************
 * This program is integration of two modules: NBV and
 * Virtual Arm Navigation. The reason for integration
 * is to guarantee the simultaneity.
 *****************************************************/

#include "ros/ros.h"

#include <moveit/move_group/capability_names.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <octomap_msgs/conversions.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "NBV_MP");
    ros::NodeHandle nh;

    ROS_INFO("Start NBV_MP");
    /**************************************************************
     * Construct a "PlanningScene" to maintain the state of world
     * including the robot. Load the robot model through robot
     * description.
     *
     * Get planning scene monitored by planning_scene_monitor, and
     * set the planning scene instance as the planning_scene_msgs.
     *************************************************************/

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    planning_scene::PlanningScenePtr ps_ch(new planning_scene::PlanningScene(robot_model));

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
        planning_scene->setPlanningSceneMsg(ps_srv.response.scene);
        ROS_INFO("Set planning_scene successfully!");
    }


    // Check octomap in planning scene.
    collision_detection::World::ObjectConstPtr obj_octm = planning_scene->getWorld()->getObject(planning_scene::PlanningScene::OCTOMAP_NS);
    if (obj_octm) {
        ROS_INFO("OCTOMAP GET");
        const shapes::OcTree* ot_shape = dynamic_cast<const shapes::OcTree*>(obj_octm->shapes_[0].get());
        if (ot_shape) {
            boost::shared_ptr<const octomap::OcTree> ot = ot_shape->octree;     //cast shape octree to octomap octree.
            octomap::OcTree* ot_ch = new octomap::OcTree(*ot);
            //boost::shared_ptr<octomap::OcTree> ot_ch = ot;
            std::string file = "ps_octm.ot";
            ot->write(file);
            ROS_INFO("WRITE");

            // Change the octomap (Set unknown area as occupied area in a bounding box)
            // Set range of bounding box

            octomap::point3d min(-1, -1.3, -0.5);
            octomap::point3d max(1, 0, 1.43);

            octomap::OcTree::leaf_bbx_iterator begin = ot->begin_leafs_bbx(min, max);
            octomap::OcTree::leaf_bbx_iterator end = ot->end_leafs_bbx();
            octomap::OcTreeKey bbxminkey = ot->coordToKey(min);
            octomap::OcTreeKey bbxmaxkey = ot->coordToKey(max);
            int sizeX = bbxmaxkey[0] - bbxminkey[0] + 1;
            int sizeY = bbxmaxkey[1] - bbxminkey[1] + 1;
            int sizeZ = bbxmaxkey[2] - bbxminkey[2] + 1;

            octomap::OcTreeKey bbxkey;
            float log_value = 0.8;
            for (int dx = 0; dx < sizeX; dx++)
            {
                //ROS_INFO("HERE");
                bbxkey[0] = bbxminkey[0] + dx;
                for (int dy = 0; dy < sizeY; dy++)
                {
                    bbxkey[1] = bbxminkey[1] + dy;
                    for (int dz = 0; dz < sizeZ; dz++)
                    {
                        bbxkey[2] = bbxminkey[2] + dz;
                        octomap::OcTreeNode* node = ot->search(bbxkey);
                        if (!node) {
                            node = ot_ch->updateNode(bbxkey, true, false);

                            //ot->updateNodedeLogOdds(node, log_value);
                        }
                    }
                }
            }
            octomap::OcTree octm_ch = *ot_ch;
            octomap_msgs::Octomap msg;
            //octomap_msgs::Octomap *oc_msg = new octomap_msgs::Octomap(ps_srv.response.scene.world.octomap.octomap);
            if (binaryMapToMsg(octm_ch, msg)) {
                ps_srv.response.scene.world.octomap.octomap = msg;
                ps_ch->setPlanningSceneMsg(ps_srv.response.scene);
            }

            // check the octomap in changed planning scene.
            collision_detection::World::ObjectConstPtr obj_occh = ps_ch->getWorld()->getObject(planning_scene::PlanningScene::OCTOMAP_NS);
            if (obj_occh) {
                ROS_INFO("OCTOMAP CHANGED GET!");
                const shapes::OcTree *occh_shape = dynamic_cast<const shapes::OcTree*>(obj_occh->shapes_[0].get());
                if (occh_shape) {
                    boost::shared_ptr<const octomap::OcTree> occh = occh_shape->octree;
                    file = "ps_octm_ch.ot";
                    occh->write(file);
                    ROS_INFO("OCTOMAP CHANGED WRITE");
                }
            }
/*
            file = "ps_octm_ch.ot";
            ot_ch->write(file);
            ROS_INFO("CHANGE WRITE!");
*/

        }
        else {
            ROS_ERROR("Failed to get shape octree");
        }
    }


    return 0;
}
