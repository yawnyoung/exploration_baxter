#include "exploration_baxter/octomap_builder.h"
#include "exploration_baxter/MPmap_filter.h"
#include "exploration_baxter/NBVStrategy.h"


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
    // Test for threads
    ot_builder.handle_thread();
    // Number of candidates
    int num_cand = 10;
    ros::Rate r(0.5);
    while (ros::ok()) {
        map_filter.FrtNbvCandidates(ot_builder.nbv_octree, ot_builder.sensorOrigin);
        m_strategy.FrtNearTracker(map_filter.frt_pc, map_filter.cand_pair, num_cand, ot_builder.sensorOrigin);
        r.sleep();
    }

    return 0;
}
