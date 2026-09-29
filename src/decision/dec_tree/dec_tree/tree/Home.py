import py_trees
from rclpy.node import Node
from py_trees.common import Status
from nav2_simple_commander.robot_navigator import BasicNavigator
import time
from .parameter import NAV_STATUS



class Home(py_trees.behaviour.Behaviour):
    '''
        巡逻\n
        name不能重复\n
        （points_name）"home" 指定在yaml内的名称\n
        wait指定到达目标点后等待时间\n
        referee_condition额外附加裁判条件 为1后将条件通过字典传入\n
        interrupt为1可以打断running状态强制发送点位
    '''
    def __init__(self, node:Node, nav: BasicNavigator, condition_func, name="home"):
        super().__init__("home")
        self.yaml = self.attach_blackboard_client(namespace="yaml")
        self.yaml.register_key('home',py_trees.common.Access.READ)
        self.blackboard = self.attach_blackboard_client()
        self.yaml.register_key('blood_limit',py_trees.common.Access.READ)
        self.yaml.register_key('blood_limit',py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal",py_trees.common.Access.WRITE)
        self.blackboard.register_key("dec_now",py_trees.common.Access.WRITE)
        self.blackboard.register_key("nav_status",py_trees.common.Access.READ)
        self.blackboard.register_key("Referee",py_trees.common.Access.READ)
    
        
        self.points = []
        self.name = "home"
        self.len = 0
        self.point_now = 0
        self.condition_func = condition_func
        self.node = node
        self.blackboard.dec_now = None
        self.wait_until = 0           # 等待截止时间
        self.waiting = False       # 等待的原因（如 'normal' 或 'home_phase_12s'）
        self.nav = nav
        self.bullet_remain_last=0
        self.got_bullet = False
        self.got_bullet_in_final_minute=False
        self.is_in_12s_home_wait=False         #如果回家以后距离下一次弹丸发放不足10s,就触发12s等待，拿到弹丸后打破等待
        self.home_wait_start_time = 0  # 记录进入12秒等待期的时间戳
    def condition(self):
        if self.condition_func(self):
            return True
        return False
    def _load_points(self):
        self.points = self.yaml.__getattr__(self.name)
        self.len = len(self.points)
    def init_dec(self):
        self.blackboard.dec_now = self.name
        self.point_now = self.points[0]     
        self.wait_begin = False
        while not self.nav.isTaskComplete():
            self.nav.cancelTask()
            self.blackboard.nav_status = NAV_STATUS.CANCELED


    def update(self):
        if not self.points:
            try:
                self._load_points()
            except KeyError:
                self.node.get_logger().warn(
                    f"yaml blackboard data for '{self.name}' is not ready yet"
                )
                return Status.FAILURE
        # 初始条件



        condition= self.condition()
        self.bullet_remain_last=self.blackboard.Referee.bullet_remaining_num_17mm
        if not condition:
            return Status.FAILURE
        



        # ---------------- START 若当前决策树不是本节点，则初始化本节点 ----------------
        if self.blackboard.dec_now != self.name:
            self.init_dec()
            self.blackboard.goal = self.point_now
            self.node.get_logger().info("%s: send goal x:%f y:%f"%("goto_home",self.point_now['x'],self.point_now['y']))


            return Status.SUCCESS
        # ---------------- END 若当前决策树不是本节点，则初始化本节点 ----------------
        
        # ---------------- START 若正在发布本节点导航点，继续导航 ----------------
        if self.blackboard.nav_status == NAV_STATUS.RUNNING:
            self.node.get_logger().info("导航正在进行中")
            return Status.SUCCESS
        # ---------------- END 若正在发布本节点导航点，继续导航 ----------------
        



        # ---------------- START 到点决策 ----------------

        # 分成4种情况
        # 1. 到达点位，reach_goal为true，则开启wait_begin;
        # 2. 未到达，继续发点
        # 3. wait_begin已经开启，时间未达到，直接继续发送当前点位
        # 4. wait_begin已经开启，时间达到，进入go_to_next尝试发送下一点位

        # 特殊情况
        # 当前在补给区，并且距离下一波发弹的时间小于10s,则等待12s


        if self.waiting == True:        
            if time.time() > self.wait_until:
              
                self.waiting = False
                self.blackboard.goal = self.point_pos_now
                self.node.get_logger().info("%s: send goal x:%f y:%f" % (self.name, self.point_pos_now['x'], self.point_pos_now['y']))
            else:
                self.node.get_logger().info("waiting ...")
        # 正常到达目标后等待
        elif self.blackboard.nav_status == NAV_STATUS.SUCCEEDED :

            
            self.waiting = True
            tmp = 1
            self.wait_until = time.time() + tmp
    
        # 默认情况：发送当前目标点
        else:
            self.blackboard.goal = self.point_pos_now
            self.node.get_logger().info("%s: send goal x:%f y:%f" % (self.name, self.point_pos_now['x'], self.point_pos_now['y']))

        return Status.SUCCESS
    
        # ---------------- END 到点决策 ----------------






        #开启等待后检查等待是否结束
        if self.waiting_for is not None:        
            if time.time() > self.wait_until:
              
                self.waiting_for = None
                self.blackboard.goal = self.point_now
                self.node.get_logger().info("%s: send goal x:%f y:%f" % (self.name, self.point_now['x'], self.point_now['y']))
                return Status.SUCCESS
            else:
                self.node.get_logger().info("waiting for %s..." % self.waiting_for)
                return Status.SUCCESS
        
        # # 检查是否需要进入12秒强制等待阶段
        # elif self.blackboard.Referee.stage_remain_time % 60 <= 10 and \
        #     self.blackboard.home_occupy != 0 and \
        #     not self.is_in_12s_home_wait:

        #     self.is_in_12s_home_wait = True
        #     self.waiting_for = "home_phase_12s"
        #     self.wait_until = time.time() + 12
        #     return Status.SUCCESS
        
        # 正常到达目标后等待
        elif self.blackboard.reach_goal:
            self.waiting_for = "normal"
            tmp = 3.0
            self.wait_until = time.time() + tmp
    
        # 默认情况：发送当前目标点
        else:
            self.blackboard.goal = self.point_now
            self.node.get_logger().info("%s: send goal x:%f y:%f" % (self.name, self.point_now['x'], self.point_now['y']))

        return Status.SUCCESS
