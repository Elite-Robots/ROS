
from elite_controller.joint_trajectory_action import JointTrajectoryAction


class EliteController(JointTrajectoryAction):
    """EliteController"""

    def __init__(self) -> None:
        JointTrajectoryAction.__init__(self)

    def spin(self):
        """spin"""
        pass


