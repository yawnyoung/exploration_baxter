#include "exploration_baxter/octomap_builder.h"
#include "exploration_baxter/MPmap_filter.h"
#include "exploration_baxter/NBVStrategy.h"
#include "exploration_baxter/PlannerAction.h"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <iostream>
#include <fstream>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_builder_test");
    ros::NodeHandle n("~");
    /* Record the time spent during whole process */
    double begin = ros::Time::now().toSec();
    /* Flag indicates that whether build NBV map and MP map separatedly */
    OctomapBuilder ot_builder(n);
    MPmapFilter map_filter(n, ot_builder.m_allbbxminkey, ot_builder.m_allbbxmaxkey);
    NBVStrategy m_strategy(n);
    PlannerAction planner_action(n);

    // Test for threads
    ot_builder.handle_thread();
    // Number of candidates
    int num_cand = 20;
    ros::WallDuration rest_time(1);

    /* Files to record updating points and planning time */
    std::ofstream pt_file;
    pt_file.open("updating_points.txt");
    std::ofstream plantime_file("plan_time.txt");
    std::ofstream proportion_file("proportion.txt");
    std::ofstream proprt_voidfrt("void_frontier.txt");

    /* Number of cells added */
    unsigned int num_add = 0;
    /* Proportion of cells added to whole cells to be scanned */
    double addTowhole = 0;
    while (addTowhole < 0.85) {
        int fail_count = 0;
        float cand_portion = 0.2;
        ros::Time plan_begin;
        ros::Duration plan_time;
        while (ros::ok()) {
            rest_time.sleep();
            ROS_INFO_STREAM("The size of inserted pointcloud is: " << ot_builder.size_insert);
            m_strategy.lower_limit = 0.2;
            if (ot_builder.size_insert < 4000) {
                //TODO sample on sphere which is further a bit. Modify in nbv strategy
                m_strategy.lower_limit = 0.5;
            }
            ROS_INFO("The result is: %d", planner_action.action_res);
            if (planner_action.action_res <= 0) {
                map_filter.FrtNbvCandidates(ot_builder.nbv_octree, ot_builder.sensorOrigin);
                if (fail_count < 10) {
                    m_strategy.FrtNearTracker(map_filter.frt_pc, map_filter.cand_pair, num_cand, ot_builder.sensorOrigin, planner_action.last_view, cand_portion, map_filter.frt_unknown);
                }
                else {
                    cand_portion += 0.03;
                    m_strategy.FrtNearTracker(map_filter.frt_pc, map_filter.cand_pair, num_cand, ot_builder.sensorOrigin, planner_action.last_view, cand_portion, map_filter.frt_unknown);
                }
                /* Record planning time */
                if (fail_count == 0) {
                    ROS_INFO("Start to record planning time");
                    plan_begin = ros::Time::now();
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
        ROS_INFO_STREAM("The updating pointcloud size is: " << ot_builder.updt_pc.size());
        pt_file << ot_builder.updt_pc.size() << std::endl;
        num_add = num_add + ot_builder.updt_pc.size();
        addTowhole = float(num_add) / float(ot_builder.scan_space);
        ROS_INFO_STREAM("The proportion of added cells to whole space is: " << addTowhole);
        proportion_file << addTowhole << std::endl;


        plan_time = ros::Time::now() - plan_begin;
        ROS_INFO_STREAM("The planner spent " << plan_time << " sec.");
        plantime_file << plan_time << std::endl;

        proprt_voidfrt << map_filter.voidfrt_frt << std::endl;

        break;

    }
    // TODO clear the updating nodes in order to count information gain
    ot_builder.updt_pc.clear();
    planner_action.ActPlan();
}

double elapse = ros::Time::now().toSec() - begin;
double elapse_min = elapse / 60;
ROS_INFO_STREAM("The whole program spent " << elapse_min << " min");

return 0;
}
