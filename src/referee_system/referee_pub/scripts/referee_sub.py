#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from referee_msg.msg import Referee 
class Subscriber(Node):
    def __init__(self,name):
        super().__init__(name)
        self.subscriber = self.create_subscription(Referee,
                                                  "Referee",
                                                  self.sub_callback,
                                                   100)
 
    def sub_callback(self,message):
        print(f"""
[remain_hp] \t\t\t {message.remain_hp}
[max_hp]    \t\t\t {message.max_hp} 
[game_type]    \t\t\t {message.game_type} 
[game_progress]    \t\t {message.game_progress} 
[stage_remain_time]    \t\t {message.stage_remain_time} 
[coin_remaining_num]    \t {message.coin_remaining_num} 
[bullet_remaining_num_17mm]    \t {message.bullet_remaining_num_17mm} 
[red_1_hp]    \t\t\t {message.red_1_hp} 
[red_2_hp]    \t\t\t {message.red_2_hp} 
[red_3_hp]    \t\t\t {message.red_3_hp} 
[red_4_hp]    \t\t\t {message.red_4_hp} 
[red_5_hp]    \t\t\t {message.red_5_hp} 
[red_7_hp]    \t\t\t {message.red_7_hp} 
[red_outpost_hp]    \t\t {message.red_outpost_hp} 
[red_base_hp]    \t\t {message.red_base_hp} 
[blue_1_hp]    \t\t\t {message.blue_1_hp} 
[blue_2_hp]    \t\t\t {message.blue_2_hp} 
[blue_3_hp]    \t\t\t {message.blue_3_hp} 
[blue_4_hp]    \t\t\t {message.blue_4_hp} 
[blue_5_hp]    \t\t\t {message.blue_5_hp} 
[blue_7_hp]    \t\t\t {message.blue_7_hp} 
[blue_outpost_hp]    \t\t {message.blue_outpost_hp} 
[blue_base_hp]    \t\t {message.blue_base_hp} 
[rfid_status]    \t\t {message.rfid_status} 
[event_type]     \t\t {message.event_type} 
""")
 
 
def main(args=None):
    rclpy.init(args=args)
    node = Subscriber("listener")
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()