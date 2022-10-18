import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse


def ec66_ik():
    rospy.init_node("examplae_03_compute_ik")
    rospy.wait_for_service("/compute_ik")
    compute_ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)
    req = GetPositionIKRequest()
    req.ik_request.group_name = "manipulator"
    req.ik_request.pose_stamped.header.frame_id = "flan"
    req.ik_request.pose_stamped.pose.position.x = 0.44266969512928483
    req.ik_request.pose_stamped.pose.position.y = 0.12200100877698622
    req.ik_request.pose_stamped.pose.position.z = 0.5740383823016267
    req.ik_request.pose_stamped.pose.orientation.w = 1
    req.ik_request.pose_stamped.pose.orientation.x = -0.7071375799397517
    req.ik_request.pose_stamped.pose.orientation.y = 0.7070759770159464
    req.ik_request.pose_stamped.pose.orientation.z = -6.813034747646873e-05
    req.ik_request.pose_stamped.pose.orientation.w = 3.349853902185735e-05
    req.ik_request.robot_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4',
                                                'joint5', 'joint6',]
    req.ik_request.robot_state.joint_state.position = [0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0]

    print(req)
    res = compute_ik(req)
    print(res)


ec66_ik()