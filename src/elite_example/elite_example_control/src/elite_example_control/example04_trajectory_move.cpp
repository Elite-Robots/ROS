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
 * @brief 笛卡尔空间waypoints轨迹运动
 *
 * @param move_group_interface
 * @param target_pose
 * @return true
 * @return false
 */
bool cart_move_with_waypoints(moveit::planning_interface::MoveGroupInterface &move_group_interface, const std::vector<geometry_msgs::Pose> waypoints)
{
   
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    move_group_interface.execute(trajectory);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string input;
    static const std::string PLANNING_GROUP = "manipulator";
    // 建立规划组
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    const moveit::core::JointModelGroup *joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    joint_move(move_group_interface, {0, -M_PI /2, M_PI / 3, -M_PI / 3, M_PI / 2, 0}, joint_model_group);

    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
    std::vector<geometry_msgs::Pose> waypoints;
    for(int i=0;i<30;i++)
    {
        current_pose.pose.position.z += 0.01;
        waypoints.push_back(current_pose.pose);
    }
    cart_move_with_waypoints(move_group_interface, waypoints);
    ros::shutdown();
}
