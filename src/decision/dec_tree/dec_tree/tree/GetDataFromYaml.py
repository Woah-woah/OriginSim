import py_trees
from ament_index_python.packages import get_package_share_directory
import yaml
from rclpy.node import Node
from py_trees.common import Status


class GetDataFromYaml(py_trees.behaviour.Behaviour):
    '''
    从指定的yaml文件中读取所有的信息\n
    存放在namespace为yaml的黑板上\n
    仅运行一次
    '''
    def __init__(self, name: str, yaml_name: str, node: Node):
        super().__init__(name)
        self.yaml_name = yaml_name
        self.open_yaml = False
        self.blackboard = self.attach_blackboard_client(namespace='yaml')
        self.node = node

    def update(self):
        if not self.open_yaml:
            try:
                path = get_package_share_directory("dec_tree") + "/config/" + self.yaml_name + ".yaml"
                with open(path, 'r') as file:
                    yaml_file = yaml.safe_load(file)
                self.node.get_logger().info("yaml文件导入成功: %s" %self.yaml_name)
            except:
                self.node.get_logger().info("无法读取yaml文件")
                return Status.FAILURE
            self.open_yaml = True

            try:
                global_config_path = get_package_share_directory("bringup") + "/params/global_config.yaml"
                with open(global_config_path, 'r') as file:
                    global_config = yaml.safe_load(file)
                self_color = str(global_config.get("self_color", "red")).lower()
                self.node.get_logger().info("global_config颜色读取成功: %s" % self_color)
            except:
                self.node.get_logger().error("无法读取bringup/params/global_config.yaml中的self_color")
                return Status.FAILURE



            blackboard_data = {
                "our_color": self_color,
                **yaml_file[self_color],
                "blood_limit": yaml_file["blood_limit"],
            }

            for key, value in blackboard_data.items():
                self._write_blackboard(key, value)
            

            return Status.SUCCESS
        else:
            return Status.SUCCESS
        
    def _write_blackboard(self, key: str, value):
        self.blackboard.register_key(
            key=key,
            access=py_trees.common.Access.WRITE
        )
        setattr(self.blackboard, key, value)

        