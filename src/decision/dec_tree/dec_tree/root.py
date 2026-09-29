import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import py_trees_ros
import py_trees
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.qos import QoSProfile
from .topic_to_blackboard_node import TopicToBlackboardNode
from .tree.CheckNavState import CheckNavState
from .tree.GetDataFromYaml import GetDataFromYaml
from .tree.PubGoal import PubGoal
from .tree.BasicBehaviour import Condition, IfThenElse, Patrol, Switch
from .tree.Home import Home




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
def create_get_data(node,qos_profile,nav):
    get_data = py_trees.composites.Parallel(
        name="get_data",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )

    get_data_from_yaml = GetDataFromYaml(
        name="get_data_from_yaml",
        yaml_name="rmuc",
        node=node
    )

    check_nav_state = CheckNavState(
        name="check_nav_state",
        nav=nav,
        node=node
    )

    
    get_data.add_children(
        [get_data_from_yaml,check_nav_state]
    )

    return get_data
# ---------------- END 数据预处理 Parallel ----------------

###################### MAIN TREE START ######################
# ---------------- START 主树第一层 判断颠簸路段、自己的能量 Selector ----------------
def create_main_tree(node,qos_profile,nav):
    first_layer = py_trees.composites.Selector(
        name="first_layer_selector",
        memory=False
    )

    # 判断自己是不是在颠簸路段上面
    condition_pass_bumpy_node = Condition(
        name="condition_pass_bumpy_node",
        node=node,
        keys=["region_int"],
        condition_func= lambda values: values["region_int"] == 1,
    )


    # 判断自己是不是有能量
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

    first_layer.add_children([condition_pass_bumpy_node, 
                              if_has_energy])
    return first_layer
# ---------------- END 主树第一层 判断颠簸路段、自己的能量 Selector ----------------

# ---------------- START 第二层 有能量情况 Selector ----------------
def create_energy_tree(node, qos_profile, nav):
    energy_tree = py_trees.composites.Selector(
        name="energy_selector",
        memory=False
    )

    goto_home = Home(
        name="home",
        node=node,
        nav=nav,
        condition_func = condition_home
    )

    # 冲家
    pilot_cmd_attack  = create_pilot_cmd_attack_subtree(node,qos_profile,nav)
        

    # 堡垒回防 有能量情况下
    back_to_fortress = create_subtree_energy_return_fortress_tree(node, qos_profile, nav)

    #基地回防
    back_to_base = create_subtree_back_to_base_tree(node,qos_profile,nav)

    #打前哨站
    attack_outpost = create_attack_outpost_tree_has_energy(node,qos_profile,nav)

    #高地打人
    mid_attack_enemy = create_has_energy_mid_attack_tree(node,qos_profile,nav)




    energy_tree.add_children([
        # 回家
        goto_home,

        # 冲家
        pilot_cmd_attack,

        # 堡垒回防
        back_to_fortress,

        # 基地回防
        back_to_base,

        # 打前哨站
        attack_outpost,

        # 高地打人
        mid_attack_enemy
                              ])
    return energy_tree
# ---------------- END 第二层 有能量情况 Selector ----------------


# ---------------- START 第二层 自己没有能量情况 判断自己位置 Switch ----------------

def create_no_energy_tree(node, qos_profile, nav):
    no_energy_tree = Switch(
        name="no_energy_switch",
        node=node,
        key="region_area",
        #翻译自己所在的具体半场位置，根据位置选择不同的策略
        # 0: 在自己半场，1：在中央高地，2：在敌方半场
        cases={
            "0": create_no_energy_at_home_tree(node, qos_profile, nav),
            "1": create_no_energy_at_mid_tree(node, qos_profile, nav),
            "2": create_no_energy_at_enemy_tree(node, qos_profile, nav),
            # 2: patrol_tree,
        },
        # default_child=default_tree
    )

    return no_energy_tree


# ---------------- END 第二层 自己没有能量情况 判断自己位置 Switch ----------------


# ---------------- START 第三层 有能量情况下 去前哨站决策 Switch ----------------
def create_attack_outpost_tree_has_energy(node, qos_profile, nav):

    if_enemy_outpost_alive = py_trees.composites.Sequence(
        name="if_enemy_outpost_alive",
        memory=False,
    )

    #前往前哨站的打击点位，在敌方前哨前的位置，
    goto_outpost = Patrol(
        name="attack_outpost_has_energy",
        node=node,
        nav=nav,
        random=0,
        points_key="outpost",
    )


    goto_wait_point = Patrol(
        name="attack_outpost_bumpy_wait",
        node=node,
        nav=nav,
        random=0,
        points_key="bunpy_wait",
    )


    # 如果在自己家并且颠簸路段上面有人，去等待点
    check_bumpy_road_blocked = IfThenElse(
        name="check_bumpy_road_blocked",
        condition_child=Condition(
            name="bumpy_road_blocked",
            node=node,
            keys=["Referee","region_area"],
            condition_func= lambda values: values["Referee"].bumpy_road == 1 and values["region_area"] == 0,
        ),
        then_child=goto_wait_point,
        else_child=goto_outpost
    )

    # 检测对面前哨站点是否存活，且比赛时间到达一定时间，说明无人机和英雄并没有能打掉对面前哨站，此时需要哨兵补刀
    enemy_outpost_alive_condition = Condition(
        name="enemy_outpost_alive_condition",
        node=node,
        keys=["Referee"],
        condition_func=lambda value: value["Referee"].enemy_outpost_alive == 1 and value["Referee"].stage_remain_time <=360
    )

    if_enemy_outpost_alive.add_children([enemy_outpost_alive_condition, 
                                         check_bumpy_road_blocked])

    return if_enemy_outpost_alive


# ---------------- END 第三层 有能量情况下 去前哨站决策 Switch ----------------


# ---------------- START 第三层 有能量情况下 高打人决策 Switch ----------------
def create_has_energy_mid_attack_tree(node, qos_profile, nav):



    # 颠簸路段后有人时停留在一级台阶前的等待点
    goto_wait_point = Patrol(
        name="mid_attack_bumpy_wait",
        node=node,
        nav=nav,
        random=0,
        points_key="bunpy_wait",
    )

    # 当在自己半场，我方颠簸路段后是否有车阻拦
    check_bumpy_road_blocked = IfThenElse(
        name="check_bumpy_road_blocked",
        condition_child=Condition(
            name="bumpy_road_blocked",
            node=node,
            keys=["Referee","region_area"],
            condition_func= lambda values: values["Referee"].bumpy_road == 1 and values["region_area"] == 0,
        ),
        then_child=goto_wait_point,
        else_child=create_catch_and_patrol_tree(node, qos_profile, nav)
    )





    return check_bumpy_road_blocked
# ---------------- END 第三层 有能量情况下 高地打人决策 Switch ----------------



# ---------------- START 第三层 没能量-自己在自己家情况 Selector ----------------
def create_no_energy_at_home_tree(node, qos_profile, nav):
    at_home_tree = py_trees.composites.Selector(
        name="at_home_selector",
        memory=False
    )

    # 回补给区
    goto_home = Home(
        name="home",
        node=node,
        nav=nav,
        condition_func = condition_home
    )

    # 在家里遛弯
    goto_somewhere_in_home = Patrol(
        name= "somewhere_in_home",
        node=node,
        nav=nav,
        random=1,
    )

    # 堡垒回防
    back_to_fortress = create_subtree_no_energy_home_return_fortress_tree(node, qos_profile, nav)
    
    # 基地回防:back_to_base_tree
    back_to_base = create_subtree_back_to_base_tree(node,qos_profile,nav)

    at_home_tree.add_children([goto_home, 
                               back_to_fortress,
                               back_to_base,
                               goto_somewhere_in_home])
    return at_home_tree
# ---------------- END 第三层 没能量-自己在自己家情况 Selector ----------------

# ---------------- START 第三层 没能量-在中央高地情况 Selector ----------------
def create_no_energy_at_mid_tree(node, qos_profile, nav):
    at_mid_tree = py_trees.composites.Selector(# 
        name="at_mid_selector",
        memory=False
    )

    # 冲家
    pilot_cmd_attack  = create_pilot_cmd_attack_enemy_mid_subtree(node,qos_profile,nav)
       
    # 去前哨站:
    attack_outpost_tree_in_mid = create_attack_outpost_tree_no_energy_in_mid(node,qos_profile,nav)

    # 堡垒回防
    back_to_fortress = create_subtree_no_energy_mid_return_fortress_tree(node, qos_profile, nav)
    
    # 高地打人:
    catch_and_patrol_tree = create_catch_and_patrol_tree(node,qos_profile,nav)

    at_mid_tree.add_children([pilot_cmd_attack,
                              attack_outpost_tree_in_mid,
                              back_to_fortress,
                              catch_and_patrol_tree])
    return at_mid_tree
# ---------------- END 第三层 没能量-自己在自己家情况 Selector ----------------

# ---------------- START 第三层 没能量-在敌方情况 Selector ----------------
def create_no_energy_at_enemy_tree(node, qos_profile, nav):


    # 堵住台阶或者狗洞，在enemy_outlet中一个或者多个点，开始巡逻
    block_their_way = Patrol(
        name="enemy_outlet_patrol",
        node=node,
        nav=nav,
        random=0,
        points_key="enemy_outlet",
    )

    return block_their_way
# ---------------- END 第三层 没能量-在敌方情况 Selector ----------------

# ------------ START SUBTREE  没能量情况 要去前哨站 ----------------


def create_attack_outpost_tree_no_energy_in_mid(node,qos_profile,nav):
    
    if_enemy_outpost_alive = py_trees.composites.Sequence(
        name="if_enemy_outpost_alive",
        memory=False,
    )

    #前往前前哨站的打击点位，在地方前哨前的位置，
    goto_outpost = Patrol(
        name="attack_outpost_no_energy_mid",
        node=node,
        nav=nav,
        random=0,
        points_key="outpost",
    )


    # 检测对面前哨站点是否存活，且比赛时间到达一定时间，说明无人机和英雄并没有能打掉对面前哨站，此时需要哨兵补刀
    enemy_outpost_alive_condition = Condition(
        name="enemy_outpost_alive_condition",
        node=node,
        keys=["Referee"],
        condition_func=lambda value: value["Referee"].enemy_outpost_alive == 1 and value["Referee"].stage_remain_time <=360
    )

    if_enemy_outpost_alive.add_children([enemy_outpost_alive_condition, 
                                         goto_outpost])

    return if_enemy_outpost_alive

# ------------ START SUBTREE  没能量情况 要去前哨站 ----------------

# ------------ START SUBTREE  回家 ----------------
def condition_home(patrol):
        return False
        is_hp_full = (patrol.blackboard.Referee.remain_hp >= 399)
        is_hp_low = (patrol.blackboard.Referee.remain_hp < patrol.yaml.blood_limit)
        is_bullet_low = (patrol.blackboard.Referee.bullet_remaining_num_17mm < 75)
        is_bullet_empty = (patrol.blackboard.Referee.bullet_remaining_num_17mm <= 0)
        is_final_minute = (patrol.blackboard.Referee.stage_remain_time <=62)
        patrol.got_bullet = ((patrol.blackboard.Referee.bullet_remaining_num_17mm - patrol.bullet_remain_last > 50) and patrol.blackboard.home_occupy != 0) #在家里这一刻拿到弹了
        print(f"got_bullet_in_final_minute:{patrol.got_bullet_in_final_minute},{patrol.got_bullet}")
        '''
        #     需要回家需要满足的条件：
        #     case1： 比赛前六分钟没血或没弹就回家，直到血量满且子弹足

        #     case2： 进入最后一分钟，没有在最后一分钟拿到弹，且血低或弹尽，直到血回满并且拿到弹再走

        #     case3： 最后一分钟并且在最后一分钟拿到过弹后，仅血量不足回家,血回满再走

        #     都需要进行的：判断当前在补给区，并且距离下一波发弹的时间小于10s,则等待12s


        '''
        if is_final_minute:
            if patrol.got_bullet:
                patrol.got_bullet_in_final_minute  = True
            
        #  最后一分钟并且拿到过弹后，仅血量不足回家,血回满再走
        if  patrol.got_bullet_in_final_minute :
            if is_hp_low:
                return True
            elif (not is_hp_full) and patrol.blackboard.dec_now == 'goto_home':
                return True
        else: #其他情况 没血或者没弹回家，补充满再走
            if is_hp_low or is_bullet_empty: 
                return True
            # elif patrol.waiting_for == "home_phase_12s":
            #     return True
            elif ((not is_hp_full) or is_bullet_low) and patrol.blackboard.dec_now == 'goto_home': #血量没回满或者子弹不足，继续在家呆着
                return True
        return False

# ---------------- END  回家 ----------------


# ------------ START SUBTREE 云台手cmd 冲家/高地 Switch----------------
def create_pilot_cmd_attack_subtree(node,qos_profile,nav):
    """
             云台手cmd冲家
             switch 选择冲家/高地
    """
   
    # 冲家
    attack_enemy_base_layer = create_pilot_cmd_attack_enemy_base_subtree(node,qos_profile,nav)
    
    # 冲高地

    attack_mid_layer = create_pilot_cmd_attack_enemy_mid_subtree(node,qos_profile,nav)


   # SWITCH 接受云台手cmd

    pilot_cmd_attack = Switch(
        name="pilot_cmd",
        node=node,
        key="Referee.pilot_cmd",
        cases={
            "0": py_trees.behaviours.Failure(name="receive_no_pilot_cmd"),
            "1": attack_mid_layer,
            "2": attack_enemy_base_layer,
        },
        default_child=py_trees.behaviours.Failure(name="receive_no_pilot_cmd")
    )

    return pilot_cmd_attack

# ------------ END SUBTREE 云台手cmd 冲家/高地 Switch----------------

# ------------ START SUBTREE 云台手cmd 冲家----------------

def create_pilot_cmd_attack_enemy_base_subtree(node,qos_profile,nav):
    
    def create_check_ready_to_attack(node):
        return Condition(
            name="check_ready_to_attack",
            node=node,
            keys=["Referee"],
            condition_func=lambda values: (
                values["Referee"].remain_hp >= 1
                and values["Referee"].bullet_remaining_num_17mm >= 50                
            ),
    )
    
    attack_enemy_base_layer = py_trees.composites.Sequence(
    name="attack_enemy_base_layer",
    memory=False
    )
    
    attack_enemy_base = Patrol(
        name="attack_base_in_their_home",
        node=node,
        nav=nav,
        random=0,
    )
    attack_enemy_base_layer.add_children([create_check_ready_to_attack(node),attack_enemy_base])

    return attack_enemy_base_layer
# ------------ END SUBTREE 云台手cmd 冲家----------------

# ------------ START SUBTREE 云台手cmd 高地 ----------------
def create_pilot_cmd_attack_enemy_mid_subtree(node,qos_profile,nav):

    def create_check_ready_to_attack(node):
        return Condition(
            name="check_ready_to_attack",
            node=node,
            keys=["Referee"],
            condition_func=lambda values: (
                values["Referee"].remain_hp >= 1
                and values["Referee"].bullet_remaining_num_17mm >= 50
                and values["Referee"].pilot_cmd == 1

            ),
    )

    attack_mid_layer = py_trees.composites.Sequence(
        name="attack_mid_layer",
        memory=False
    )

    attack_mid = Patrol(
        name="attack_base_in_mid",
        node=node,
        nav=nav,
        random=1,
    )
    attack_mid_layer.add_children([create_check_ready_to_attack(node),attack_mid])

    return attack_mid_layer

# ------------ END SUBTREE 云台手cmd 高地 ----------------




# ---------------- START SUBTREE 有能量情况下 堡垒回防 Sequence ----------------
def create_subtree_energy_return_fortress_tree(node, qos_profile, nav):
    energy_return_fortress_tree = py_trees.composites.Sequence(
        name="energy_return_fortress_sequence",
        memory=False
    )
    condition_energy_return_fortress = Condition(
        name="energy_should_back_to_fortress",
        node=node,
        keys=["Referee"],
        condition_func=lambda value: value["Referee"].fortress_enemy == 1
    )
    goto_fortress_home = Patrol(
        name="energy_fortress_patrol_home",
        node=node,
        nav=nav,
        random=0,
        points_key="fortress_when_at_home"
    )
    goto_fortress_mid = Patrol(
        name="energy_fortress_patrol_mid",
        node=node,
        nav=nav,
        random=0,
        points_key="fortress_when_at_mid"
    )
    where_to_fortress = Switch(
        name="energy_where_to_fortress",
        node=node,
        key="region_area",
        cases={
            "0": goto_fortress_home,
            "1": goto_fortress_mid
        },
        default_child=goto_fortress_mid
    )
    energy_return_fortress_tree.add_children([condition_energy_return_fortress, where_to_fortress])
    return energy_return_fortress_tree
# ---------------- END SUBTREE 有能量情况下 堡垒回防 Sequence ----------------

# ---------------- START SUBTREE 没能量情况下在家 堡垒回防 Sequence ----------------
def create_subtree_no_energy_home_return_fortress_tree(node, qos_profile, nav):
    no_energy_home_return_fortress_tree = py_trees.composites.Sequence(
        name="no_energy_home_return_fortress_sequence",
        memory=False
    )
    condition_no_energy_home_return_fortress = Condition(
        name="no_energy_home_should_return_fortress",
        node=node,
        keys=["Referee"],
        condition_func=lambda value: value["Referee"].fortress_enemy == 1
    )
    goto_fortress_home = Patrol(
        name="no_energy_fortress_patrol_home",
        node=node,
        nav=nav,
        random=0,
        points_key="fortress_when_at_home"
    )
    no_energy_home_return_fortress_tree.add_children([condition_no_energy_home_return_fortress, goto_fortress_home])
    return no_energy_home_return_fortress_tree
# ---------------- END SUBTREE 没能量情况下在家 堡垒回防 Sequence ----------------

# ---------------- START SUBTREE 没能量情况下在中央高地 堡垒回防 Sequence ----------------
def create_subtree_no_energy_mid_return_fortress_tree(node, qos_profile, nav):
    no_energy_mid_return_fortress_tree = py_trees.composites.Sequence(
        name="no_energy_mid_return_fortress_sequence",
        memory=False
    )
    condition_no_energy_mid_return_fortress = Condition(
        name="no_energy_mid_should_return_fortress",
        node=node,
        keys=["Referee"],
        condition_func=lambda value: value["Referee"].fortress_enemy == 1
    )
    goto_fortress_mid = Patrol(
        name="no_energy_fortress_patrol_mid",
        node=node,
        nav=nav,
        random=0,
        points_key="fortress_when_at_mid"
    )
    no_energy_mid_return_fortress_tree.add_children([condition_no_energy_mid_return_fortress, goto_fortress_mid])
    return no_energy_mid_return_fortress_tree
# ---------------- END SUBTREE 没能量情况下在中央高地 堡垒回防 Sequence ----------------

# ---------------- START SUBTREE 基地回防 Sequence ----------------
def create_subtree_back_to_base_tree(node, qos_profile, nav):
    back_to_base_tree = py_trees.composites.Sequence(
        name="back_to_base_sequence",
        memory=False
    )

    # 判断我方基地血量条件，ally_base_hp变量名参考 referee.msg串口
    condition_back_to_base = Condition(
        name="should_back_to_base",
        node=node,
        keys=["Referee"],
        condition_func=lambda value: value["Referee"].ally_base_hp <= 2000
    )

    # 前往我方基地的节点，专门用于基地回防（开花）
    goto_ally_base = Patrol(
        name="ally_base",
        node=node,
        nav=nav,
        random= 0,
    )

    back_to_base_tree.add_children([condition_back_to_base, 
                                    goto_ally_base])
    return back_to_base_tree
# ---------------- END SUBTREE 基地回防 Selector ----------------


# ---------------- START SUBTREE 雷达抓人 + 高低巡逻 Selector ----------------
def create_catch_and_patrol_tree(node, qos_profile, nav):
    catch_and_patrol_tree = py_trees.composites.Selector(
        name="catch_and_patrol_selector",
        memory=False
    )

    # 抓英雄
    catch_hero = py_trees.composites.Sequence(
        name="catch_hero_selector",
        memory=False
    )

    #判断是否有英雄存在某个点位，处于可打击状态
    condition_catch_hero_node = Condition(
        name="catch_hero_condition",
        node=node,
        keys=["Referee"],
        condition_func= lambda values: values["Referee"].enemy_hero_pos != 0,
    )
    # 前往打击英雄的点位，打击英雄的点位由雷达发出，所以random等于2
    goto_enemy_hero = Patrol(
        name="hero",
        node=node,
        nav=nav,
        random=2,
    )

    catch_hero.add_children([condition_catch_hero_node, 
                             goto_enemy_hero])

    # 抓工程
    catch_engineer = py_trees.composites.Sequence(
        name="catch_engineer_selector",
        memory=False
    )

    # 判断是否有工程存在某个点位，处于可打击状态
    condition_catch_engineer_node = Condition(
        name="catch_engineer_condition",
        node=node,
        keys=["Referee"],
        condition_func= lambda values: values["Referee"].enemy_engineer_pos != 0,
    )

    # 前往打击工程的点位，打击工程的点位由雷达发出，所以random等于2
    goto_enemy_engineer = Patrol(
        name="engineer",
        node=node,
        nav=nav,
        random=2,
    )

    catch_engineer.add_children([condition_catch_engineer_node, 
                                 goto_enemy_engineer])


    # 保底-高地巡逻
    goto_mid = Patrol(
        name= "mid",
        node=node,
        nav=nav,
        random=1
    )
    catch_and_patrol_tree.add_children([catch_hero,
                                        catch_engineer,
                                        goto_mid])

    return catch_and_patrol_tree
# ---------------- END SUBTREE 雷达抓人 + 高低巡逻 Selector ----------------


###################### MAIN TREE END ######################




# ---------------- START 发布点以及对应的决策 --------------------------------
def create_pub_goal_and_behaviour(node, qos_profile, nav):
    pub_goal_and_behaviour = py_trees.composites.Sequence(
        name="pub_goal_and_behaviour_sequence",
        memory=False
    )

    pub_goal = PubGoal(
        name="pub_goal",
        nav=nav
    )


    pub_goal_and_behaviour.add_children([pub_goal])

    return pub_goal_and_behaviour



# ---------------- END 发布点以及对应的决策 --------------------------------



def create_root_tree(node, qos_profile, nav):
    root = py_trees.composites.Sequence(
        name="root",
        memory=False,
    )
    root.add_children([
        create_get_data(node, qos_profile, nav),
        create_main_tree(node, qos_profile, nav),
        create_pub_goal_and_behaviour(node,qos_profile,nav)
    ])
    return root



def main(args = None):
    rclpy.init(args=args)
    qos_profile = QoSProfile(depth=10)
    receiver_node = TopicToBlackboardNode(qos_profile=qos_profile)
    node = Node("tree_node")
    nav = BasicNavigator()
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
