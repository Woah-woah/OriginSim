import py_trees
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from referee_msg.msg import Referee
from rm_interfaces.msg import GimbalCmd,RegionArea
from std_msgs.msg import Bool,Int32


class TopicToBlackboardNode(Node):
    def __init__(self, qos_profile: QoSProfile | None = None):
        super().__init__('topic_to_blackboard_node')

        self.blackboard = py_trees.blackboard.Client(name='topic_to_blackboard')

        self.qos_profile = qos_profile or QoSProfile(depth=10)

        # 用于保存 subscription，防止被 Python 垃圾回收
        self.subscriptions_cache = []


        self.bind_topic_to_blackboard(
            msg_type=Referee,
            topic_name='/Referee',
            key='Referee',
            default_value=Referee(),
            qos_profile=self.qos_profile
        )

        self.bind_topic_to_blackboard(
            msg_type=GimbalCmd,
            topic_name='/serial/process_gimbal_aft_inv',
            key='auto_aim',
            default_value=GimbalCmd(),
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )

        self.bind_topic_to_blackboard(
            msg_type=Int32,
            topic_name='/region_int',
            key='region_int',
            default_value=-1,
            value_getter=lambda msg: msg.data,
            qos_profile=self.qos_profile
        )


        self.bind_topic_to_blackboard(
            msg_type=RegionArea,
            topic_name='/region_area',
            key='region_area',
            default_value=0,
            value_getter=lambda msg: msg.area_type,
            qos_profile=self.qos_profile
        )
    def bind_topic_to_blackboard(
        self,
        msg_type,
        topic_name: str,
        key: str,
        default_value=None,
        qos_profile: QoSProfile | None = None,
        value_getter=None
    ):
        """
        msg_type:
            ROS2 消息类型，例如 Referee、GimbalCmd、Bool。
        topic_name:
            订阅的话题名，例如 '/Referee'。
        key:
            写入 blackboard 的变量名，例如'Referee'、'auto_aim'。
        default_value:
            blackboard 的初始值。
        qos_profile:
            该 topic 使用的 QoS。
        value_getter:
            可选的数据提取函数。
            如果不填，则整个 msg 写入 blackboard。
            如果填写，例如 lambda msg: msg.data，则只写入 msg.data。
        """

        self.blackboard.register_key(
            key=key,
            access=py_trees.common.Access.WRITE
        )

        if default_value is not None:
            setattr(self.blackboard, key, default_value)

        if value_getter is None:
            value_getter = lambda msg: msg

        def callback(msg):
            value = value_getter(msg)
            setattr(self.blackboard, key, value)

        subscription = self.create_subscription(
            msg_type,
            topic_name,
            callback,
            qos_profile or self.qos_profile
        )

        # 保存 subscription，防止被释放
        self.subscriptions_cache.append(subscription)
