#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

class ExplorationSensing {

    /* Private functions */
    void cloudMsgCB (const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
    void self_filter ();

    /* Private data members */
    ros::NodeHandle nh;


    // Parameters for octomap construction.


    // Parameters for listened topic and self-filtered function.
    public:
        ExplorationSensing();
};
ExplorationSensing::ExplorationSensing() :
    nh("~");
{

}
