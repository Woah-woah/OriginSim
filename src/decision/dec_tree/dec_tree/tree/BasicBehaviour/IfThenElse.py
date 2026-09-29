import py_trees
from py_trees.common import Status


class IfThenElse(py_trees.composites.Composite):
    """
    IfThenElse:
        先 tick condition_child。
        如果 condition_child 返回 SUCCESS，则 tick then_child。
        如果 condition_child 返回 FAILURE，则 tick else_child。
        如果 condition_child 返回 RUNNING，则自身返回 RUNNING。
        最终状态等于被执行分支的状态。


    使用方式：


    if_then_else = IfThenElse(
    name="attack_or_patrol",
    condition_child=Condition(
        name="should_attack",
        node=node,
        key="is_attack",
        condition_func=lambda value: value is True
    ),
    then_child=attack_tree,
    else_child=patrol_tree
    )

    """

    def __init__(
        self,
        name: str,
        condition_child: py_trees.behaviour.Behaviour,
        then_child: py_trees.behaviour.Behaviour,
        else_child: py_trees.behaviour.Behaviour | None = None,
    ):
        super().__init__(name=name)

        self.condition_child = condition_child
        self.then_child = then_child
        self.else_child = else_child

        children = [condition_child, then_child]
        if else_child is not None:
            children.append(else_child)

        self.add_children(children)

    def tick(self):
        self.logger.debug("%s.tick()" % self.__class__.__name__)

        # 先 tick 条件节点
        for node in self.condition_child.tick():
            yield node

        # condition 还在运行，则整个 IfThenElse 也 RUNNING
        if self.condition_child.status == Status.RUNNING:
            self.status = Status.RUNNING
            yield self
            return

        # condition 成功，选择 then 分支
        if self.condition_child.status == Status.SUCCESS:
            selected_child = self.then_child
            unselected_child = self.else_child

        # condition 失败，选择 else 分支
        elif self.condition_child.status == Status.FAILURE:
            selected_child = self.else_child
            unselected_child = self.then_child

        else:
            self.status = Status.INVALID
            yield self
            return

        # 停止未被选择且正在运行的分支
        if unselected_child is not None:
            if unselected_child.status == Status.RUNNING:
                unselected_child.stop(Status.INVALID)

        # 如果没有 else 分支，并且 condition 失败，则自身 FAILURE
        if selected_child is None:
            self.status = Status.FAILURE
            yield self
            return

        # tick 被选择的分支
        for node in selected_child.tick():
            yield node

        # 自身状态等于被选择分支的状态
        self.status = selected_child.status
        yield self