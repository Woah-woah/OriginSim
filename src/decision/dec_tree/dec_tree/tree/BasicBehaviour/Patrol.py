import py_trees
from rclpy.node import Node
from py_trees.common import Status
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import random
from ..parameter import NAV_STATUS




class Patrol(py_trees.behaviour.Behaviour):
    def __init__(self, name: str,  node:Node, nav: BasicNavigator, random=1, points_key=None):
        super().__init__(name)
        self.yaml = self.attach_blackboard_client(namespace="yaml")
        self.points_key = points_key if points_key is not None else name
        self.yaml.register_key(self.points_key,py_trees.common.Access.READ)
        # self.yaml.register_key("our_outpost",py_trees.common.Access.READ)
        # self.yaml.register_key("our_color",py_trees.common.Access.READ)Unpack
        # self.yaml.register_key("their_outpost",py_trees.common.Access.READ)
        self.blackboard = self.attach_blackboard_client()
        self.yaml.register_key('blood_limit',py_trees.common.Access.READ)
        self.yaml.register_key('blood_limit',py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal",py_trees.common.Access.WRITE)
        self.blackboard.register_key("dec_now",py_trees.common.Access.WRITE)
        self.blackboard.register_key("nav_status",py_trees.common.Access.READ)
        self.blackboard.register_key("nav_status",py_trees.common.Access.WRITE)
        self.blackboard.register_key("Referee",py_trees.common.Access.READ)



        self.points = []
        self.name = name
        self.len = 0
        self.point_pos_now = 0
        self.node = node
        self.blackboard.dec_now = None
        self.wait_until = 0           # 等待截止时间
        self.waiting = False
        self.random =random
        self.nav = nav
        self.first_time_init = True



    def _load_points(self):
        self.points = self.yaml.__getattr__(self.points_key)
        self.len = len(self.points)

    def init_dec(self):
        if not self.points:
            self._load_points()
        self.blackboard.dec_now = self.name
        if  self.random == 2 :
            enemy_pos_attr = f"enemy_{self.points_key}_pos"
            enemy_pos = int(getattr(self.blackboard.Referee, enemy_pos_attr))
            print((enemy_pos))
            self.point_pos_now = self.points[enemy_pos - 1] 
        else :
            self.point_pos_now = self.points[0]
        self.wait_begin = False
        self.end_time = 0
        random.seed(time.time())
        while not self.nav.isTaskComplete():
            self.nav.cancelTask()
            self.blackboard.nav_status = NAV_STATUS.CANCELED


    def go_to_next(self):
        if self.len == 1:
            return
        if self.random==1: # 随机巡逻
            tmp = random.choice(self.points)
            while tmp == self.point_pos_now:
                tmp = random.choice(self.points)
            self.point_pos_now = tmp
        elif self.random == 0: # 不随机巡逻
            self.point_pos_now=self.points[(self.points.index(self.point_pos_now)+1)%self.len]
        elif self.random == 2: # 取点巡逻
            return
    def update(self):
        if not self.points:
            try:
                self._load_points()
            except KeyError:
                self.node.get_logger().warn(
                    f"yaml blackboard data for '{self.name}' is not ready yet"
                )
                return Status.FAILURE

        
        # ---------------- START 若当前决策树不是本节点，则初始化本节点 ----------------
        if self.blackboard.dec_now != self.name:
            self.init_dec()
            self.blackboard.goal = self.point_pos_now
            self.node.get_logger().info("%s: send goal x:%f y:%f"%(self.name,self.point_pos_now['x'],self.point_pos_now['y']))

            if self.blackboard.dec_now == 'goto_outpost' or self.blackboard.dec_now == 'goto_mid':
                self.yaml.blood_limit = 201
            else :
                self.yaml.blood_limit = 201

            return Status.SUCCESS
        # ---------------- END 若当前决策树不是本节点，则初始化本节点 ----------------
        



        # ---------------- START 若正在发布本节点导航点，继续导航 ----------------
        if self.blackboard.nav_status == NAV_STATUS.RUNNING:
            self.node.get_logger().info("running")
            return Status.SUCCESS
        # ---------------- END 若正在发布本节点导航点，继续导航 ----------------
        
        

        # ---------------- START 到点决策 ----------------


        self.first_time_init  = False




            
        # 分成4种情况
        # 1. 检查，到达点位，reach_goal为true，则开启wait_begin;
        # 2. 未到达，继续发点
        # 3. wait_begin已经开启，时间未达到，直接继续发送当前点位
        # 4. wait_begin已经开启，时间达到，进入go_to_next尝试发送下一点位
        
        
        
        #开启等待后检查等待是否结束
        if self.waiting == True:        
            if time.time() > self.wait_until:
              
                self.waiting = False
                self.go_to_next()
                self.blackboard.goal = self.point_pos_now
                self.node.get_logger().info("%s: send goal x:%f y:%f" % (self.name, self.point_pos_now['x'], self.point_pos_now['y']))
            else:
                self.node.get_logger().info("waiting ...")
        # 正常到达目标后等待
        elif self.blackboard.nav_status == NAV_STATUS.SUCCEEDED :
            self.waiting = True
            tmp = 0.1
            self.wait_until = time.time() + tmp
    
        # 默认情况：发送当前目标点
        else:
            self.blackboard.goal = self.point_pos_now
            self.node.get_logger().info("%s: send goal x:%f y:%f" % (self.name, self.point_pos_now['x'], self.point_pos_now['y']))

        return Status.SUCCESS
    
        # ---------------- END 到点决策 ----------------
