#include "exploration_baxter/octomap_builder.h"
//#include "exploration_baxter/MPmap_filter.h"              MPmap_filter.h was already included in NBVStrategy.h
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
    //int num_cand = 20;
    ros::WallDuration rest_time(0.5);

    /* Files to record updating points and planning time */
    std::ofstream pt_file;
    pt_file.open("updating_points.txt");
    std::ofstream plantime_file("plan_time.txt");
    std::ofstream proportion_file("proportion.txt");
    std::ofstream proprt_voidfrtone("void_frontierone.txt");
    std::ofstream proprt_voidfrttwo("void_frontiertwo.txt");
    std::ofstream proprt_voidfrtthree("void_frontierthree.txt");
    std::ofstream areaone("areaone_prop.txt");
    std::ofstream areatwo("areatwo_prop.txt");
    std::ofstream areathree("areathree_prop.txt");

    /* Number of cells added */
    unsigned int num_add = 0;

    /* Last planned frontier to be observed */
    octomap::point3d last_frt(0, 0, 0);
    double propone = 0;
    double proptwo = 0;
    double propthree = 0;
    while (propone < 0.975 || proptwo || 0.95 && propthree || 0.9) {
        int fail_count = 0;
        float cand_portion = 0.05;
        int num_cand = 20;
        ros::Time plan_begin;
        ros::Duration plan_time;
        bool loop_start = true;
        propone = map_filter.vol_areaone_nbv / ot_builder.vol_areaone;
        proptwo = map_filter.vol_areatwo_nbv / ot_builder.vol_areatwo;
        propthree = map_filter.vol_areathree_nbv / ot_builder.vol_areathree;
        while (ros::ok()) {
            rest_time.sleep();
            ROS_INFO_STREAM("The size of inserted pointcloud is: " << ot_builder.size_insert);
            m_strategy.lower_limit = 0.2;
            ROS_INFO("The result is: %d", planner_action.action_res);
            if (planner_action.action_res <= 0) {
                map_filter.FrtNbvCandidates(ot_builder.nbv_octree, ot_builder.sensorOrigin, last_frt);
                rest_time.sleep();
                if (!map_filter.frt_observed && loop_start) {
                    /*
                    m_strategy.lower_limit = 0.5;
                    cand_portion = 0.05;
                    ROS_INFO("Change Mode!!!!!!");
                    m_strategy.FrtNearTracker(map_filter.frt_pc, map_filter.cand_pair, num_cand, ot_builder.sensorOrigin, planner_action.last_view, cand_portion, map_filter.frt_unknown);
                    */
                    ROS_INFO("Change mode!!!!!!!!!!!!!");
                    m_strategy.ComputePose(num_cand, map_filter.frt_observed);
                }
                else {
                    if (fail_count < 10) {
                        if (map_filter.frt_pc.size() > 0) {
                            //m_strategy.ManeuverContinue(map_filter.frt_pc, map_filter.frt_areaone, map_filter.frt_areatwo, map_filter.frt_areathree, num_cand, ot_builder.sensorOrigin, cand_portion);
                            //m_strategy.ContInfogain(map_filter.frt_pc, map_filter.frt_group, num_cand, ot_builder.sensorOrigin);
                            m_strategy.ContInfogainMulCands(map_filter.frt_pc, map_filter.frt_group, ot_builder.sensorOrigin, propone, proptwo, propthree);
                            m_strategy.ComputePose(num_cand, map_filter.frt_observed);
                        }
                        else {
                            continue;
                        }
                    }
                    else {
                        //cand_portion += 0.03;
                        num_cand += num_cand;
                        if (map_filter.frt_pc.size() > 0) {
                            //m_strategy.ManeuverContinue(map_filter.frt_pc, map_filter.frt_areaone, map_filter.frt_areatwo, map_filter.frt_areathree, num_cand, ot_builder.sensorOrigin, cand_portion);
                            //m_strategy.ContInfogain(map_filter.frt_pc, map_filter.frt_group, num_cand, ot_builder.sensorOrigin);
                            m_strategy.ContInfogainMulCands(map_filter.frt_pc, map_filter.frt_group, ot_builder.sensorOrigin, propone, proptwo, propthree);
                            m_strategy.ComputePose(num_cand, map_filter.frt_observed);
                        }
                        else {
                            continue;
                        }
                    }
                }

                /* Record planning time */
                if (fail_count == 0) {
                    ROS_INFO("Start to record planning time");
                    plan_begin = ros::Time::now();
                }
                planner_action.motionPlan(m_strategy.nbvCands, ot_builder.mp_octree);
                if (!planner_action.plan_out) {
                    fail_count++;
                    ROS_INFO("Failed to plan motion");
                    loop_start = false;
                    continue;
                }
            }
            else {
                continue;
            }

            last_frt = m_strategy.frtprp_obsv.frt_node;

            ROS_INFO_STREAM("The updating pointcloud size is: " << ot_builder.updt_pc.size());
            pt_file << ot_builder.updt_pc.size() << std::endl;
            num_add = num_add + ot_builder.updt_pc.size();
            proportion_file << map_filter.vol_workspace_nbv / ot_builder.vol_workspace << std::endl;


            plan_time = ros::Time::now() - plan_begin;
            ROS_INFO_STREAM("The planner spent " << plan_time << " sec.");
            plantime_file << plan_time << std::endl;

            proprt_voidfrtone << map_filter.voidfrt_frtone << std::endl;
            proprt_voidfrttwo << map_filter.voidfrt_frttwo << std::endl;
            proprt_voidfrtthree << map_filter.voidfrt_frtthree << std::endl;

            areaone << map_filter.vol_areaone_nbv / ot_builder.vol_areaone << std::endl;
            areatwo << map_filter.vol_areatwo_nbv / ot_builder.vol_areatwo << std::endl;
            areathree << map_filter.vol_areathree_nbv / ot_builder.vol_areathree << std::endl;

            ROS_INFO("Writing MP octomap to binary file");
            ot_builder.mp_octree->writeBinary("my_mpoctree.bt");

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
