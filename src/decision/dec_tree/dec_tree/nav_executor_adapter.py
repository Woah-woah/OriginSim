from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger


class NavExecutorAdapter:
    def __init__(self, node):
        self.node = node

        # nav executor 初始状态
        self.state = None
        self.goal_id = None

        self.goal_id_before_send = None  # 发送新目标前的执行器目标编号
        self.confirmed_goal_id = None    # 执行器已接受的新目标编号
        self.replan_future = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # 发送目标
        self.goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)

        # 接收导航状态
        self.state_sub = node.create_subscription(String, '/nav_executor/state', self._state_callback, qos)

        # 接收目标编号
        self.goal_id_sub = node.create_subscription(UInt64, '/nav_executor/goal_id', self._goal_id_callback, qos)

        # 取消导航
        self.cancel_client = node.create_client(Trigger, '/nav_executor/cancel')
        self.replan_client = node.create_client(Trigger, '/nav_executor/replan')


    def _state_callback(self, msg):
        self.state = msg.data

    def _goal_id_callback(self, msg):
        self.goal_id = msg.data

        # 新目标确认完成，旧目标编号就不再参与判断。
        if self.goal_id_before_send is not None and self.goal_id > self.goal_id_before_send:
            self.confirmed_goal_id = self.goal_id
            self.goal_id_before_send = None

            self.node.get_logger().info(f'导航目标已确认：goal_id={self.confirmed_goal_id}')


    def send_goal(self, goal: PoseStamped):
        if self.goal_id is None:
            self.node.get_logger().warn('未收到goal_id, 暂不发点')
            return False

        self.goal_id_before_send = self.goal_id
        self.confirmed_goal_id = None
        self.replan_future = None

        self.goal_pub.publish(goal)
        return True


    def get_current_state(self):
        if self.replan_future is not None:
            if not self.replan_future.done():
                return None

            future = self.replan_future
            self.replan_future = None

            try:
                response = future.result()
            except Exception as error:
                self.node.get_logger().error(f'重规划服务调用失败：{error}')
                return 'TERMINAL_FAILED'

            if not response.success:
                self.node.get_logger().error(f'重规划请求被拒绝：{response.message}')
                return 'TERMINAL_FAILED'

        if self.goal_id_before_send is not None:   # 还在等待新目标确认，不返回状态
            return None
        if self.confirmed_goal_id is None:         # 新目标未确认，不返回状态
            return None
        if self.goal_id != self.confirmed_goal_id:  # 当前反馈不是已确认的目标
            return None

        return self.state



    def cancel(self):
        if not self.cancel_client.service_is_ready():
            self.node.get_logger().warn('nav_executor/cancel 服务不可用')
            return None

        return self.cancel_client.call_async(Trigger.Request())



    def replan(self):
        if not self.replan_client.service_is_ready():
            self.node.get_logger().warn('nav_executor/replan 服务不可用')
            return False

        self.state = None
        self.replan_future = self.replan_client.call_async(Trigger.Request())   # 异步调用
        return True