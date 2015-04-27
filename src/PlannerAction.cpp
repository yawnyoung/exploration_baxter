#include "exploration_baxter/PlannerAction.h"

#define PI 3.1415926838597

PlannerAction::PlannerAction(ros::NodeHandle nh):
    pa_nh(nh),
    robot_model_loader("robot_description"),
    plan_group("right_arm"),                         // Set default planning group to be right_arm
    m_planner_id("RRTstarkConfigDefault"),
    link_name("camera_rgb_optical_frame"),           // Set default planning link name to be right hand camera
    action_res(0)                                    // Set default value to be 0 in order to launch the first motion plan
{
    /* Get Parameters */
    pa_nh.param("plan_group", plan_group, plan_group);
    pa_nh.param("plan_endeffector", link_name, link_name);
    pa_nh.param("planner_id", m_planner_id, m_planner_id);

    /* Create robot model and planning scene instance */
    robot_model = robot_model_loader.getModel();
    planning_scene = boost::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(robot_model));
    /* SETUP THE PLANNER */
    if (!pa_nh.getParam("planning_plugin", planner_plugin_name)) {
        ROS_FATAL_STREAM("Could not find planner plugin name");
    }
    try {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    } catch (pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader" << ex.what());
    }
    try {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, pa_nh.getNamespace())) {
            ROS_FATAL_STREAM("Could not initialize planner instance");
        }
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    } catch (pluginlib::PluginlibException& ex) {
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); i++)
        {
            ss << classes[i] << " ";
        }
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "'" <<ex.what() << std::endl << "Available plugins: " << ss.str());
    }

    /* Setup action client */
    std::string action_topic = "robot/limb/right/follow_joint_trajectory";
    traj_client = new TrajClient(action_topic, true);
    while (!traj_client->waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for action server");
    }

    /* Listen to robot_state */
    robot_state_sub = pa_nh.subscribe("/robot/joint_states", 1, &PlannerAction::RobotStateCB, this);
    /* Set publisher to publish the trajectory message */
    display_pub = pa_nh.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1, true);
    /* Set publisher to publish planning scene */
    ps_pub = pa_nh.advertise<moveit_msgs::PlanningScene>("my_planning_scene", 1);
    /* Listen to action result */
    action_res_sub = pa_nh.subscribe("/robot/limb/right/follow_joint_trajectory/result", 1, &PlannerAction::ActionResultCB, this);

    /* Initialize the last view record */
    geometry_msgs::Point last_point;
    last_point.x = 0;
    last_point.y = 0;
    last_point.z = 0;
    geometry_msgs::Quaternion last_ort;
    last_ort.x = 0;
    last_ort.y = 0;
    last_ort.z = 0;
    last_ort.w = 1;
    last_view.position = last_point;
    last_view.orientation = last_ort;
    ROS_INFO("Last view has been initialized!");
}

void PlannerAction::RobotStateCB(const sensor_msgs::JointStatePtr& msg)
{
    /* Clear the last robot state */
    robotstate_crr.joint_state.name.clear();
    robotstate_crr.joint_state.position.clear();
    robotstate_crr.joint_state.velocity.clear();
    robotstate_crr.joint_state.effort.clear();
    /* Assign joint_states to robot_state without joint 'head_nod', joitn 'head_pan' and joint 'torso_t0'*/
    for (int i = 9; i < 16; i++)
    {
        robotstate_crr.joint_state.name.push_back(msg->name[i]);
        robotstate_crr.joint_state.position.push_back(msg->position[i]);
        robotstate_crr.joint_state.velocity.push_back(msg->velocity[i]);
        robotstate_crr.joint_state.effort.push_back(msg->effort[i]);
    }
    /* Data validation */
    //ROS_INFO("Number of joints is: %lu", robotstate_crr.joint_state.name.size());
}

void PlannerAction::motionPlan(viewsStamped &view_cand, octomap::OcTree *ot_map)
{
    /* Configure planning scene */
    planning_scene->setCurrentState(robotstate_crr);            // Set current robot state to planning scene (the start state of motion planning in each loop)
    octomap_msgs::Octomap ot_msg;                               // Octomap message processed in planning scene
    if (octomap_msgs::fullMapToMsg(*ot_map, ot_msg)) {
        planning_scene->processOctomapMsg(ot_msg);
    }
    else {
        ROS_ERROR("Failed to set planning scene octomap");
    }
    /* Publish planning scene */
    moveit_msgs::PlanningScene ps_msg;
    planning_scene->getPlanningSceneMsg(ps_msg);
    ps_pub.publish(ps_msg);
    /* Test robot state validation */
    robot_state::RobotState currt_state = planning_scene->getCurrentStateNonConst();
    const robot_state::JointModelGroup *joint_model_group = currt_state.getJointModelGroup(plan_group);
    std::string end_eff = joint_model_group->getLinkModelNames().back();
    ROS_INFO_STREAM("The end effector name is: " << end_eff);

    /* Motion plan request configuration */
    planning_interface::MotionPlanRequest req;
    req.group_name = plan_group;
    req.planner_id = m_planner_id;
    std::vector<double> tolerance_pose(3 , 0.02);           // Pose tolerance
    std::vector<double> tolerance_angle;                    // Angle tolerance
    tolerance_angle.push_back(PI / 12);
    tolerance_angle.push_back(PI / 12);
    tolerance_angle.push_back(2 * PI);
    //req.goal_constraints.clear();                           // Clear last requests
    int fail_count = 0;
    for (viewsStamped::iterator cand_it = view_cand.begin(); cand_it != view_cand.end(); cand_it++)
    {
        ROS_INFO_STREAM("The last view x: " << last_view.position.x <<
                                     " y: " << last_view.position.y <<
                                     " z: " << last_view.position.z);


        /*if (cand_it->pose.position.x == last_view.position.x &&
            cand_it->pose.position.y == last_view.position.y &&
            cand_it->pose.position.z == last_view.position.z) {
            ROS_INFO("This view is the same with the last view. Skip it!");
            continue;
        }*/
        req.goal_constraints.clear();                         // Clear last request
        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(link_name, *cand_it, tolerance_pose, tolerance_angle);
        req.goal_constraints.push_back(pose_goal);
        planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
        context->solve(res);
        if (res.error_code_.val == res.error_code_.SUCCESS) {
            ROS_INFO("Compute plan successfully");
            //break;
            /* Check path validation with planning scene function */
            moveit_msgs::RobotState curr_state_msg;
            moveit::core::robotStateToRobotStateMsg(currt_state, curr_state_msg);
            ROS_INFO("Convert robot state to robot state message");
            res.getMessage(res_msg);
            if (!planning_scene->isPathValid(curr_state_msg, res_msg.trajectory, plan_group)) {
                ROS_INFO("The planned path is invalid checked through planning scene function");
            }
            else {
                ROS_INFO("The planned path is valid!!!!");
                int point_nb = res_msg.trajectory.joint_trajectory.points.size();
                int joint_nb = res_msg.trajectory.joint_trajectory.joint_names.size();
                ROS_INFO("The number of planned trajectory points is: %d", point_nb);
                for (int i = 0; i < joint_nb; i++) {
                    //ROS_INFO_STREAM("The planned joints are: " << res_msg.trajectory.joint_trajectory.joint_names[i]);
                    ROS_INFO_STREAM("Joint: " << res_msg.trajectory.joint_trajectory.joint_names[i] << " Planned position: " << res_msg.trajectory.joint_trajectory.points[0].positions[i]);
                }
                ROS_INFO_STREAM("The planned view x: " << cand_it->pose.position.x <<
                                                " y: " << cand_it->pose.position.y <<
                                                " z: " << cand_it->pose.position.z);
                /* Update last view */
                last_view.position = cand_it->pose.position;
                last_view.orientation = cand_it->pose.orientation;
                /* Update plan outcome flag */
                plan_out = true;
                /* Update action result */
                action_res = 1;
                break;
            }
        }
        else {
            fail_count++;
            ROS_INFO("%d: Failed! The reason is: %d", fail_count, res.error_code_.val);
        }
    }
    if (res.error_code_.val != res.error_code_.SUCCESS) {
        ROS_ERROR_STREAM("Could not compute plan successfully: " << res.error_code_.val);
        plan_out = false;
    }
    else {
        /* Visualize the result */
        VisPlanpath();
        /* Take Action */
    }
}

void PlannerAction::VisPlanpath()
{
    ros::WallDuration sleep_time(5.0);

    moveit_msgs::DisplayTrajectory display_trajectory;
    ROS_INFO("Visualizing the trajectory");
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.clear();
    display_trajectory.trajectory.push_back(response.trajectory);
    display_pub.publish(display_trajectory);

    //sleep_time.sleep();
}

void PlannerAction::ActPlan()
{
    control_msgs::FollowJointTrajectoryGoal goal;
    /* Test trajectory points */
    int point_nb = res_msg.trajectory.joint_trajectory.points.size();
    double step_time = 0.3;
    //ros::Duration step(move_time / point_nb);
    ros::Duration step(step_time);
    //res_msg.trajectory.joint_trajectory.points[1].time_from_start = move_time;
    ROS_INFO("The number of trajectory points is: %d", point_nb);
    goal.trajectory = res_msg.trajectory.joint_trajectory;
    goal.trajectory.header.stamp = ros::Time::now();
    for (int i = 0; i < point_nb; i++) {
        goal.trajectory.points[i].time_from_start = step * (i + 1);
    }
    /* Set velocities at each trajectory point */
    std::vector<double> curr_joints;                    // Current joint positions
    curr_joints.push_back(robotstate_crr.joint_state.position[2]);
    curr_joints.push_back(robotstate_crr.joint_state.position[3]);
    curr_joints.push_back(robotstate_crr.joint_state.position[0]);
    curr_joints.push_back(robotstate_crr.joint_state.position[1]);
    curr_joints.push_back(robotstate_crr.joint_state.position[4]);
    curr_joints.push_back(robotstate_crr.joint_state.position[5]);
    curr_joints.push_back(robotstate_crr.joint_state.position[6]);

    Eigen::MatrixXd AvgVel_mat(7, point_nb);                // Average velocity matrix
    Eigen::VectorXd AvgVel_col(7);                             // Average velocity column
    for (int i = 0; i < 7; i++) {
        AvgVel_col[i] = (goal.trajectory.points[0].positions[i] - curr_joints[i]) / step_time;
    }
    AvgVel_mat.col(0) = AvgVel_col;
    for (int i = 1; i < point_nb; i++) {
        for (int j = 0; j < 7; j++) {
            AvgVel_col[j] = (goal.trajectory.points[i].positions[j] - goal.trajectory.points[i-1].positions[j]) / step_time;
        }
        AvgVel_mat.col(i) = AvgVel_col;
    }

    for (int i = 0; i < (point_nb - 1); i++) {
        for (int j = 0; j < 7; j++) {
            goal.trajectory.points[i].velocities[j] = (AvgVel_mat(j, i) + AvgVel_mat(j, i+1))/2;
        }
    }

    traj_client->sendGoal(goal);
}

void PlannerAction::ActionResultCB(const control_msgs::FollowJointTrajectoryActionResultPtr& msg)
{
    action_res = msg->result.error_code;
}
