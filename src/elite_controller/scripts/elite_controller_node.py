import rospy
from elite_controller.elite_controller import EliteController

def main():
    """主函数"""
    rospy.init_node('elite_controller', disable_signals=True)
    virtual_robot = EliteController()
    rate = rospy.Rate(125)
    while not rospy.is_shutdown():
        virtual_robot.spin()
        rate.sleep()


if __name__ == "__main__":
    main()
