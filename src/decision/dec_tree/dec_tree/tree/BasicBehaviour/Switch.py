import py_trees
from rclpy.node import Node
from py_trees.common import Status

class Switch(py_trees.composites.Composite):
    """
    Switch 根据 blackboard 中指定变量的值选择并 tick 对应子节点。
    最终返回被选中子节点的 status，若无匹配且无默认子节点则返回 FAILURE。

    使用方式：

    switch = Switch(
        name="mode_switch",
        node=node,
        key="mode",
        cases={
            "attack": attack_tree,
            "defence": defence_tree,
            "patrol": patrol_tree,
        },
        default_child=default_tree
    )

    """

    def __init__(
        self,
        name: str,
        node: Node,
        key: str,
        cases: dict,
        default_child=None
    ):
        super().__init__(name=name)

        self.node = node
        self.key = key
        self.cases = cases
        self.default_child = default_child

        self.blackboard = self.attach_blackboard_client()

        root_key = self.key.split(".")[0]

        self.blackboard.register_key(
            key=root_key,
            access=py_trees.common.Access.READ
        )

        children = list(self.cases.values())

        if self.default_child is not None:
            children.append(self.default_child)

        unique_children = []
        seen = set()
        for child in children:
            if id(child) not in seen:
                unique_children.append(child)
                seen.add(id(child))

        self.add_children(unique_children)

    def _get_blackboard_value(self):

        parts = self.key.split(".")

        try:
            value = getattr(self.blackboard, parts[0])
        except KeyError:
            raise KeyError(parts[0])

        for part in parts[1:]:
            if hasattr(value, part):
                value = getattr(value, part)
            elif isinstance(value, dict) and part in value:
                value = value[part]
            else:
                raise KeyError(self.key)

        return value

    def tick(self):
        try:
            value = self._get_blackboard_value()
        except KeyError:
            self.node.get_logger().warn(
                f"Switch 读取 blackboard.{self.key} 失败"
            )
            self.status = py_trees.common.Status.FAILURE
            yield self
            return
        print(value)
        selected_child = self.cases.get(str(value), self.default_child)

        if selected_child is None:
            self.node.get_logger().warn(
                f"Switch 没有匹配 case: {value}"
            )
            self.status = py_trees.common.Status.FAILURE
            yield self
            return

        for child in self.children:
            if child is not selected_child:
                if child.status == py_trees.common.Status.RUNNING:
                    child.stop(py_trees.common.Status.INVALID)

        for node in selected_child.tick():
            yield node

        self.status = selected_child.status
        yield self