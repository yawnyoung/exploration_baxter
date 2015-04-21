#include "exploration_baxter/octomap_builder.h"
#include "exploration_baxter/MPmap_filter.h"
#include "exploration_baxter/NBVStrategy.h"
#include "exploration_baxter/PlannerAction.h"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_builder_test");
    ros::NodeHandle n("~");
    /* Flag indicates that whether build NBV map and MP map separatedly */
    bool createMPmap = true;
    n.param("create_MPmap", createMPmap, createMPmap);
    OctomapBuilder ot_builder(n, createMPmap);
    MPmapFilter map_filter(n, ot_builder.m_allbbxminkey, ot_builder.m_allbbxmaxkey);
    NBVStrategy m_strategy(n);
    PlannerAction planner_action(n);

    // Test for threads
    ot_builder.handle_thread();
    // Number of candidates
    int num_cand = 20;
    ros::WallDuration rest_time(2);
    while (ros::ok()) {
        int fail_count = 0;
        float cand_portion = 0.3;
        while (ros::ok()) {
            rest_time.sleep();
            //ROS_INFO("Ready to plan motion? \n Please enter 1 to start motion planning:\n");
            //int plan_flag = 0;
            //std::cin >> plan_flag;
            ROS_INFO_STREAM("The updating pointcloud size is: " << ot_builder.updt_pc.size());
            ROS_INFO("The result is: %d", planner_action.action_res);
            if (planner_action.action_res <= 0) {
                map_filter.FrtNbvCandidates(ot_builder.nbv_octree, ot_builder.sensorOrigin);
                if (fail_count < 10) {
                    m_strategy.FrtNearTracker(map_filter.frt_pc, map_filter.cand_pair, num_cand, ot_builder.sensorOrigin, planner_action.last_view, cand_portion);
                }
                else {
                    cand_portion += 0.03;
                    m_strategy.FrtNearTracker(map_filter.frt_pc, map_filter.cand_pair, num_cand, ot_builder.sensorOrigin, planner_action.last_view, cand_portion);
                }
                planner_action.motionPlan(m_strategy.nbvCands, ot_builder.mp_octree);
                //planner_action.motionPlan(m_strategy.RHCnbvCands, ot_builder.mp_octree);
                //}
                if (!planner_action.plan_out) {
                    fail_count++;
                    continue;
                }
        }
        else {
            continue;
        }
        ROS_INFO("Is result satisfied? Please enter 1 to start acting.");
        int act_flag = 0;
        std::cin >> act_flag;
        if (act_flag == 1) {
            break;
        }
        else {
            planner_action.action_res = 0;
            continue;
        }
    }
    // TODO clear the updating nodes in order to count information gain
    ot_builder.updt_pc.clear();
    planner_action.ActPlan();
}


return 0;
}
