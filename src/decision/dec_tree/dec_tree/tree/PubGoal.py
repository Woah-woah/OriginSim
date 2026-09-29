import py_trees
import yaml
from rclpy.node import Node
from py_trees.common import Status
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from .parameter import *
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class PubGoal(py_trees.behaviour.Behaviour):
    '''
        发布目标点
    '''
    def __init__(self, name: str, nav: BasicNavigator):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.yaml = self.attach_blackboard_client(namespace="yaml")
        self.blackboard.register_key("Referee",py_trees.common.Access.READ)
        self.blackboard.register_key("goal",py_trees.common.Access.READ)
        self.blackboard.register_key("nav_status",py_trees.common.Access.READ)
        self.blackboard.register_key("nav_status",py_trees.common.Access.WRITE)
        self.nav = nav

    def update(self):
        
        if self.blackboard.nav_status == NAV_STATUS.RUNNING:
            return Status.FAILURE
        
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.orientation.w = 1.0
        print("正在前往目标点")
        goal_msg.pose.position.x = float(self.blackboard.goal['x'])
        goal_msg.pose.position.y = float(self.blackboard.goal['y'])

        if self.nav.goToPose(goal_msg):
            self.blackboard.nav_status = NAV_STATUS.RUNNING

        return Status.SUCCESS
