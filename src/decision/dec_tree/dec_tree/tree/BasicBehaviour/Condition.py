import py_trees
from rclpy.node import Node
from py_trees.common import Status



"""
a.判断单个 blackboard 变量

check_mid_occupy = BlackboardCondition(
    name="check_mid_occupy",
    node=node,
    keys=["mid_occupy"],
    condition_func=lambda values: values["mid_occupy"] == 1,
    default_values={
        "mid_occupy": 0
    }
)



b.判断多个 blackboard 变量

check_ready_to_attack = BlackboardCondition(
    name="check_ready_to_attack",
    node=node,
    keys=["nav_status", "mid_occupy", "is_attack"],
    condition_func=lambda values: (
        values["nav_status"] == NAV_STATUS.SUCCEEDED
        and values["mid_occupy"] == 1
        and values["is_attack"] is True
    ),
    default_values={
        "mid_occupy": 0,
        "is_attack": False
    }
)

含义：
同时读取 blackboard.nav_status、blackboard.mid_occupy、blackboard.is_attack。
只有导航成功、中立区状态为 1、攻击标志为 True 时，返回 SUCCESS。
否则返回 FAILURE。
"""
class Condition(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        node: Node,
        keys: list[str],
        condition_func,
        default_values: dict | None = None
    ):
        super().__init__(name)

        self.node = node
        self.keys = keys
        self.condition_func = condition_func
        self.default_values = default_values or {}

        self.blackboard = self.attach_blackboard_client()

        for key in self.keys:
            self.blackboard.register_key(
                key=key,
                access=py_trees.common.Access.READ
            )

    def update(self):
        values = {}

        for key in self.keys:
            try:
                values[key] = getattr(self.blackboard, key)
            except KeyError:
                if key in self.default_values:
                    values[key] = self.default_values[key]
                else:
                    self.node.get_logger().warn(
                        f"blackboard 中不存在变量: {key}"
                    )
                    return Status.FAILURE

        if self.condition_func(values):
            return Status.SUCCESS

        return Status.FAILURE