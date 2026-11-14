import py_trees
from py_trees.common import Status
from .parameter import *
from ..nav_executor_adapter import NavExecutorAdapter


class CheckNavState(py_trees.behaviour.Behaviour):
    '''
        检查导航状态
        Status.SUCCESS 节点执行状态
        NAV_STATUS.SUCCEEDED 导航状态枚举
    '''
    def __init__(self, name: str, nav: NavExecutorAdapter):
        super().__init__(name)

        self.nav = nav
        self.node = nav.node

        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("nav_status",py_trees.common.Access.READ)
        self.blackboard.register_key("nav_status",py_trees.common.Access.WRITE)

        self.blackboard.nav_status = NAV_STATUS.NOGOAL


    def update(self):

        if self.blackboard.nav_status == NAV_STATUS.NOGOAL: #导航不在运行状态，都不用查
            return Status.SUCCESS

        state = self.nav.get_current_state()    # 从 nav executor adapter 读取最新导航状态 

        if state is None:         # 未收到导航状态
            return Status.SUCCESS

        if state == "SUCCEEDED":    # 导航成功
            new_status = NAV_STATUS.SUCCEEDED

        elif state == "TERMINAL_FAILED":  # 导航最终失败
            new_status = NAV_STATUS.FAILED

        elif state in (
            "WAITING_FOR_MAP",
            "PLANNING",
            "PATH_READY",
            "RECOVERING",
            "RECOVERY_PENDING",
            "FAILED"                # 在新架构中是继续尝试 重规划 或者 escape 的意思，而不是最终失败
        ):
            new_status = NAV_STATUS.RUNNING

        elif state == "IDLE":     # 当前没有导航任务
            new_status = NAV_STATUS.NOGOAL
            
        else:          # 其他未知状态
            self.node.get_logger().error(f"未识别到的 nav_executor 状态：{state!r}")
            new_status = NAV_STATUS.ERROR

        '''
        new_status	             NAV_STATUS.RUNNING	    NAV_STATUS
        new_status.name	               "RUNNING"	       str
        new_status.value	               2	           int
        '''

        # 更新黑板状态
        old_status = self.blackboard.nav_status
        if new_status != self.blackboard.nav_status:
            self.blackboard.nav_status = new_status
            self.node.get_logger().info(f"导航状态更新：{old_status.name} -> {new_status.name}")

        return Status.SUCCESS
    