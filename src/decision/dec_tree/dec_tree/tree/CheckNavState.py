import py_trees
import yaml
from rclpy.node import Node
from py_trees.common import Status
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from .parameter import *







class CheckNavState(py_trees.behaviour.Behaviour):
    '''
        检查导航状态
    '''
    def __init__(self, name: str, nav: BasicNavigator, node: Node):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("nav_status",py_trees.common.Access.READ)
        self.blackboard.register_key("nav_status",py_trees.common.Access.WRITE)


        self.nav = nav
        self.blackboard.nav_status = NAV_STATUS.NOGOAL
        self.node = node


    def update(self):
        if self.blackboard.nav_status == NAV_STATUS.NOGOAL: #导航不在运行状态，都不用查
            return Status.SUCCESS
        
        if self.nav.isTaskComplete():
            result = self.nav.getResult()

            match result:
                case TaskResult.SUCCEEDED:
                    self.node.get_logger().info("导航成功完成")
                    self.blackboard.nav_status = NAV_STATUS.SUCCEEDED

                case TaskResult.FAILED:
                    self.node.get_logger().error("导航失败")
                    self.nav.clearAllCostmaps()
                    self.blackboard.nav_status = NAV_STATUS.FAILED


                case TaskResult.CANCELED:
                    self.node.get_logger().warn("导航被取消")
                    self.nav.clearAllCostmaps()
                    self.blackboard.nav_status = NAV_STATUS.CANCELED

                case _:
                    self.node.get_logger().error(f"导航异常终止: {result}")
                    self.nav.clearAllCostmaps()
                    self.blackboard.nav_status = NAV_STATUS.ERROR
        else:
            self.blackboard.nav_status = NAV_STATUS.RUNNING


        return Status.SUCCESS
    