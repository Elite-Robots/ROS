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
 * @brief 关节空间运动-joint space
 * 
 * @param move_group_interface 
 * @param joint_group_positions 
 * @param joint_model_group 
 * @return true 
 * @return false 
 */
bool joint_move(moveit::planning_interface::MoveGroupInterface &move_group_interface, const std::vector<double> &joint_group_positions,const moveit::core::JointModelGroup *joint_model_group)
{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group_interface.setJointValueTarget(joint_group_positions);
  move_group_interface.setMaxVelocityScalingFactor(0.05);
  move_group_interface.setMaxAccelerationScalingFactor(0.05);
  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
  move_group_interface.execute(my_plan);
  return false;
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
    ros::shutdown();
}
