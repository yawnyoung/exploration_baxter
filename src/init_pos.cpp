#include "ros/ros.h"

#include <baxter_core_msgs/JointCommand.h>

int main(int argc, char **argv)
{
    // Initialize ros node
    ros::init(argc, argv, "InitPos");
    ros::NodeHandle nh("~");

    // Create publisher
    ros::Publisher cmd_pub = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);

    // Set command
    baxter_core_msgs::JointCommand jt_cmd;

    // set command mode as position mode
    jt_cmd.mode = 1;

    /* Middle */

    jt_cmd.command.push_back(-0.767);
    jt_cmd.command.push_back(2.389);
    jt_cmd.command.push_back(-1.623);
    jt_cmd.command.push_back(-1.056);
    jt_cmd.command.push_back(-1.857);
    jt_cmd.command.push_back(-1.570);
    jt_cmd.command.push_back(0.049);

    /* Right */
    /*
    jt_cmd.command.push_back(-1.773);
    jt_cmd.command.push_back(2.172);
    jt_cmd.command.push_back(-0.560);
    jt_cmd.command.push_back(-1.420);
    jt_cmd.command.push_back(1.795);
    jt_cmd.command.push_back(-1.065);
    jt_cmd.command.push_back(-1.521);*/

    jt_cmd.names.push_back("right_e0");
    jt_cmd.names.push_back("right_e1");
    jt_cmd.names.push_back("right_s0");
    jt_cmd.names.push_back("right_s1");
    jt_cmd.names.push_back("right_w0");
    jt_cmd.names.push_back("right_w1");
    jt_cmd.names.push_back("right_w2");

    while (ros::ok()) {
        cmd_pub.publish(jt_cmd);
    }
    return 0;
}
