import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import py_trees_ros
import py_trees
from rclpy.qos import QoSProfile
from .topic_to_blackboard_node import TopicToBlackboardNode
from .tree.CheckNavState import CheckNavState
from .tree.GetDataFromYaml import GetDataFromYaml
from .tree.PubGoal import PubGoal
from .tree.BasicBehaviour import Condition, IfThenElse, Patrol, Switch
from .tree.Home import Home
from .nav_executor_adapter import NavExecutorAdapter

"""

1. Sequence (且)
   顺序执行所有子节点；遇到 FAILURE 或 RUNNING 立即返回该状态，只有所有子节点都 SUCCESS 时才返回 SUCCESS。
   常用于“前置条件 + 执行动作”的流程链。

2. Selector (或)
   也可理解为 Fallback；顺序尝试子节点；遇到 SUCCESS 或 RUNNING 立即返回该状态，只有所有子节点都 FAILURE 时才返回 FAILURE。
   常用于“优先级选择”或“多个备选方案”。

3. Parallel
   同时 tick 所有子节点；返回状态由 policy 决定，例如 SuccessOnAll 表示全部 SUCCESS 才 SUCCESS，SuccessOnOne 表示任一 SUCCESS 即 SUCCESS。
   常用于多个检测、监听、更新任务并行执行。

4. Switch
   自定义 Composite；根据 blackboard 中指定变量的值选择并 tick 对应子节点，最终返回被选中子节点的 status；若无匹配且无默认子节点，则返回 FAILURE。
   常用于根据 mode / 状态机变量选择 attack、defence、patrol 等不同子树。

5. IfThenElse
   自定义 Composite；先 tick condition 子节点，condition 返回 SUCCESS 时执行 then_child，返回 FAILURE 时执行 else_child，返回 RUNNING 时自身返回 RUNNING；最终返回被执行分支的 status。
   常用于“如果条件成立则执行 A，否则执行 B”的二分支逻辑。


"""

"""
几点修改说明：
1. patrol类中的random参数定义：
    0   内部点按照固定顺序不巡逻
    1   内部点随机巡逻
    2   外部点随机巡逻

2.patrol类中name的声明：
    请和yaml文件中的点对应，记得在yaml中补充声明
"""


# ---------------- START 数据预处理 Parallel ----------------
def create_get_data(node, qos_profile, nav):
    get_data = py_trees.composites.Parallel(
        name="get_data",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )

    get_data_from_yaml = GetDataFromYaml(
        name="get_data_from_yaml",
        yaml_name="rmuc_simple",
        node=node
    )

    check_nav_state = CheckNavState(
        name="check_nav_state",
        nav=nav
    )

    get_data.add_children(
        [get_data_from_yaml, check_nav_state]
    )

    return get_data
# ---------------- END 数据预处理 Parallel -------------------



###################### MAIN TREE START ######################

# ------------- START 主树 第一层 自己的能量 Sequence ------------
def create_main_tree(node, qos_profile, nav):
    first_layer = py_trees.composites.Sequence(
        name="first_layer_sequence",
        memory=False
    )

    if_has_energy = IfThenElse(
        name="if_has_energy",
        condition_child=Condition(
            name="has_energy",
            node=node,
            keys=["Referee"],
            condition_func=lambda value: value["Referee"].remain_energy > 5
        ),
        then_child=create_energy_tree(node, qos_profile, nav),
        else_child=create_no_energy_tree(node, qos_profile, nav)
    )

    first_layer.add_children(
        [if_has_energy]
    )

    return first_layer
# ------------- END 主树 第一层 自己的能量 Sequence ------------


# ------------- START 主树 第二层 有能量 Selector ------------
def create_energy_tree(node, qos_profile, nav):
    energy_tree = py_trees.composites.Selector(
        name="energy_selector",
        memory=False
    )

    go_home = Home(
        name="home",
        nav=nav,
        condition_func=condition_home
    )

    attack_outpost = Patrol(
        name="attack_outpost_has_energy",
        nav=nav,
        random=0,
        points_key="outpost"
    )

    energy_tree.add_children(
        [go_home, attack_outpost]
    )

    return energy_tree
# ------------- END 主树 第二层 有能量 Selector ------------


# ------------- START 主树 第二层 无能量 Sequence ------------
def create_no_energy_tree(node, qos_profile, nav):
    no_energy_tree = py_trees.composites.Sequence(
        name="no_energy_sequence",
        memory=False
    )

    go_home = Home(
        name="home",
        nav=nav,
        condition_func=condition_home
    )

    no_energy_tree.add_children(
        [go_home]
    )

    return no_energy_tree
# ------------- END 主树 第二层 无能量 Sequence ------------


# ------------- START SUBTREE 回家 ------------
def condition_home(home):
    return False
# ------------- END SUBTREE 回家 ------------

###################### MAIN TREE END ######################



# ---------------- START 发布点以及对应的决策 --------------------------
def create_pub_goal_and_behaviour(node, qos_profile, nav):
    pub_goal_and_behavior = py_trees.composites.Sequence(
        name="pub_goal_and_behavior_sequence",
        memory=False
    )

    pub_goal = PubGoal(
        name="pub_goal",
        nav=nav
    )

    pub_goal_and_behavior.add_children(
        [pub_goal]
    )

    return pub_goal_and_behavior
# ---------------- END 发布点以及对应的决策 --------------------------



def create_root_tree(node, qos_profile, nav):
    root = py_trees.composites.Sequence(
        name="root",
        memory=False
    )

    root.add_children([
        create_get_data(node, qos_profile, nav),
        create_main_tree(node, qos_profile, nav),
        create_pub_goal_and_behaviour(node, qos_profile, nav)
    ])

    return root


def main(args=None):
    rclpy.init(args=args)
    qos_profile = QoSProfile(depth=10)
    receiver_node = TopicToBlackboardNode(qos_profile=qos_profile)
    node = Node("tree_node")
    nav = NavExecutorAdapter(node)
    period_ms = 100

    root = create_root_tree(node, qos_profile, nav)
    tree = py_trees_ros.trees.BehaviourTree(root)

    tree.setup(node=node)
    tree.tick_tock(period_ms=period_ms)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(receiver_node)
    print(py_trees.display.ascii_tree(root))
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        receiver_node.destroy_node()
    rclpy.shutdown()
