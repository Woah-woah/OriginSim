import time
import py_trees
from geometry_msgs.msg import PoseStamped
from py_trees.common import Status
from .parameter import NAV_STATUS
from ..nav_executor_adapter import NavExecutorAdapter


class PubGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, nav: NavExecutorAdapter):
        super().__init__(name)

        self.nav = nav
        self.node = nav.node

        self.blackboard = self.attach_blackboard_client()

        self.blackboard.register_key('goal', py_trees.common.Access.READ)

        self.blackboard.register_key('nav_status', py_trees.common.Access.READ)
        self.blackboard.register_key('nav_status', py_trees.common.Access.WRITE)

        self.last_goal = None     # 记录上一次发送的目标
        self.retry_at = None
        self.retry_delay = 1.0    # 目标 terminal_failed 之后时隔多久再重试


    def update(self):
        goal = self.blackboard.goal
        x = goal["x"]
        y = goal["y"]
        current_goal = (x, y)


        # 1. 有新目标了，发出去
        if current_goal != self.last_goal:
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.orientation.w = 1.0

            if not self.nav.send_goal(msg):
                return Status.FAILURE

            self.last_goal = current_goal   # 更新 goal
            self.retry_at = None            # 不需要 retry，所以没有时间
            self.blackboard.nav_status = NAV_STATUS.RUNNING
            self.node.get_logger().info(f"发布目标：{x:.2f}, {y:.2f}")
            return Status.SUCCESS


        # 2. 正在执行最新发布的目标时，没有 terminal_failed
        if self.blackboard.nav_status != NAV_STATUS.FAILED:
            self.retry_at = None
            return Status.SUCCESS


        # 3. 正在执行的这个目标 terminal_failed 了
        now = time.monotonic()    # returns a steadily increasing clock value in fractional seconds that never goes backward

        if self.retry_at is None:
            self.retry_at = now + self.retry_delay

        if now < self.retry_at:   # 还在等待阶段
            return Status.SUCCESS

        if not self.nav.replan():
            return Status.FAILURE
        
        self.retry_at = now + self.retry_delay   # 重新更新
        self.blackboard.nav_status = NAV_STATUS.RUNNING
        self.node.get_logger().info("导航最终失败，重新规划当前目标")

        return Status.SUCCESS
