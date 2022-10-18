#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

/**
 * @brief 关节空间运动
 *
 * @param move_group_interface
 * @param joint_group_positions
 * @param joint_model_group
 * @return true
 * @return false
 */
bool joint_move(moveit::planning_interface::MoveGroupInterface &move_group_interface, const std::vector<double> &joint_group_positions, const moveit::core::JointModelGroup *joint_model_group)
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group_interface.setJointValueTarget(joint_group_positions);
    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    move_group_interface.execute(my_plan);
    return success;
}


/**
 * @brief 笛卡尔空间运动带方向约束
 *
 * @param move_group_interface
 * @param target_pose
 * @return true
 * @return false
 */
bool cart_move_with_constaint(moveit::planning_interface::MoveGroupInterface &move_group_interface, geometry_msgs::Pose &target_pose,const moveit::core::JointModelGroup *joint_model_group)
{
   
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();


    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "flan";
    ocm.header.frame_id = "base_link";
    ocm.orientation.w = current_pose.pose.orientation.w;
    ocm.orientation.x = current_pose.pose.orientation.x;
    ocm.orientation.y = current_pose.pose.orientation.y;
    ocm.orientation.z = current_pose.pose.orientation.z;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;
    // Now, set it as the path constraint for the group.
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_interface.setPathConstraints(test_constraints);

    moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
    start_state.setFromIK(joint_model_group, current_pose.pose);
    move_group_interface.setStartState(start_state);
    move_group_interface.setPoseTarget(target_pose);
    move_group_interface.setPlanningTime(10.0);

    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizin plan 3 (constraints) %s", success ? "" : "FAILED");
    ROS_INFO("Success with %ld points", my_plan.trajectory_.joint_trajectory.points.size());
    move_group_interface.execute(my_plan);

    move_group_interface.clearPathConstraints();

    return success;
}

int main(int argc, char **argv)
{
    static const std::string PLANNING_GROUP = "manipulator";
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // 建立规划组
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    const moveit::core::JointModelGroup *joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    // 当前状态
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    current_state->printStateInfo();
    // 关节运动
    joint_move(move_group_interface, {0, -M_PI /2, M_PI / 3, -M_PI / 3, M_PI / 2, 0}, joint_model_group);
    // 当前位置
    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
    // 下降30cm后运动
    current_pose.pose.position.z -= 0.3;
    cart_move_with_constaint(move_group_interface, current_pose.pose, joint_model_group);
    ros::shutdown();
}
