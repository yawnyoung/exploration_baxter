#include <exploration_baxter/octomap_builder.h>
#include "exploration_baxter/MPmap_filter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_builder_test");
    ros::NodeHandle n("~");
    octomap::point3d freebbxmin(-0.5, -0.6, -0.2);
    octomap::point3d freebbxmax(0.2, 0, 1);
    OctomapBuilder ot_builder(n, freebbxmin, freebbxmax);
    MPmapFilter map_filter(n);
    while (ros::ok()) {
        ros::spinOnce();
        map_filter.frontierExtraction(ot_builder.m_octree);
    }

    return 0;
}
