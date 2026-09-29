import random
import time

import rclpy
from rclpy.node import Node

from referee_msg.msg import Referee
from std_msgs.msg import Int8


class Panel_Publisher(Node):

    def __init__(self, node_name='control_panel_pub'):
        super().__init__(node_name)

        self.game_state_publisher_ = self.create_publisher(Referee, '/Referee', 10)

        # 订阅串口发来的状态切换请求
        self.request_state_sub_ = self.create_subscription(
            Int8,
            '/serial/request_state_change',
            self.request_state_change_callback,
            10
        )

        # 20Hz，既用于自动发布，也用于检查延迟状态切换
        self.timer_ = self.create_timer(0.05, self.timer_callback)

        self.game_state = Referee()
        self.reset_game_state()

        # 延迟切换相关
        self.pending_state_now = None
        self.pending_apply_time = None

    def reset_game_state(self):
        msg = self.game_state

        msg.remain_hp = 600
        msg.max_hp = 600
        msg.bullet_remaining_num_17mm = 400
        msg.bullet_cooling_speed = 40
        msg.shooter_heat_limit = 200
        msg.shooter_heat_now = 0
        msg.remain_energy = 100
        msg.health_state = 0
        msg.state_now = 0
        msg.stage_remain_time = 420
        msg.game_progress = 0
        msg.ally_1_robot_hp = 600
        msg.ally_2_robot_hp = 600
        msg.ally_3_robot_hp = 600
        msg.ally_4_robot_hp = 600
        msg.ally_outpost_hp = 1000
        msg.ally_base_hp = 5000
        msg.rfid_status = 0
        msg.event_type = 0
        msg.return_fortress = 0

    def request_state_change_callback(self, msg: Int8):
        requested_state = int(msg.data)

        current_state = int(self.game_state.state_now)

        # 1. 如果请求状态和当前已生效状态相同，不做任何事
        if requested_state == current_state:
            return

        # 2. 如果已经有一个相同的待切换状态，也不要重复刷新延迟
        if self.pending_state_now is not None and requested_state == int(self.pending_state_now):
            return

        # 3. 只有真正来了一个“新的不同状态请求”，才重新开始延迟
        delay = random.uniform(0.1, 1.0)
        self.pending_state_now = requested_state
        self.pending_apply_time = time.monotonic() + delay

        self.get_logger().info(
            f"Received new state change request: {requested_state}, "
            f"will apply after {delay:.3f}s"
        )
    def _apply_pending_state_if_needed(self):
        if self.pending_state_now is None or self.pending_apply_time is None:
            return

        now = time.monotonic()
        if now >= self.pending_apply_time:
            old_state = int(self.game_state.state_now)
            new_state = int(self.pending_state_now)

            self.game_state.state_now = new_state

            self.get_logger().info(
                f"Applied delayed state_now change: {old_state} -> {new_state}"
            )

            self.pending_state_now = None
            self.pending_apply_time = None

    def timer_callback(self):
        # 先处理延迟状态切换
        self._apply_pending_state_if_needed()

        # 比赛运行中才倒计时
        if self.game_state.game_progress == 4 and self.game_state.stage_remain_time > 0:
            self.game_state.stage_remain_time -= 1
            if self.game_state.stage_remain_time <= 0:
                self.game_state.stage_remain_time = 0
                self.game_state.game_progress = 5

        self.game_state_publisher_.publish(self.game_state)


def main(args=None):
    rclpy.init(args=args)
    node = Panel_Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()