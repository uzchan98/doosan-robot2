import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

import yaml


class ConnectionNode(Node):
    def __init__(self):
        super().__init__('connection_node')
        
        # 파라미터 선언
        self.declare_parameter('name', 'dsr01')
        self.declare_parameter('rate', 100)
        self.declare_parameter('standby', 5000)
        self.declare_parameter('command', True)
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 12345)
        self.declare_parameter('mode', 'real')
        self.declare_parameter('model', 'm1013')
        self.declare_parameter('gripper', 'none')
        self.declare_parameter('mobile', 'none')
        
        self.my_parameter_ = self.get_parameter('mobile').value
        self.get_logger().info('mobile: %s' % self.my_parameter_)
        parameters = {}
        parameters['name'] = self.get_parameter('name').get_parameter_value().string_value
        parameters['rate'] = self.get_parameter('rate').get_parameter_value().integer_value
        parameters['standby'] = self.get_parameter('standby').get_parameter_value().integer_value
        parameters['command'] = self.get_parameter('command').get_parameter_value().bool_value
        parameters['host'] = self.get_parameter('host').get_parameter_value().string_value
        parameters['port'] = self.get_parameter('port').get_parameter_value().integer_value
        parameters['mode'] = self.get_parameter('mode').get_parameter_value().string_value
        parameters['model'] = self.get_parameter('model').get_parameter_value().string_value
        parameters['gripper'] = self.get_parameter('gripper').get_parameter_value().string_value
        parameters['mobile'] = self.get_parameter('mobile').get_parameter_value().string_value
        
        current_file_path = os.path.join(
            get_package_share_directory("dsr_hardware2"), "config"
        )
        run_script_path = os.path.join(
            get_package_share_directory("common2"), "bin"
        )
        os.makedirs(current_file_path, exist_ok=True)
        
            # exec(command)
            # print(command)
            # eval("{}/run_drcf.sh ".format(run_script_path), host, str(port), name)
        
        with open(os.path.join(current_file_path, 'parameters.yaml'), 'w') as file:
            yaml.dump(parameters, file)
        if parameters['mode'] == 'virtual':
            port = parameters['port']
            model = parameters['model']
            name = parameters['name']
            command = "{}/run_drcf.sh ".format(run_script_path) +" "+ str(port)+" "+ model +" " +name
            # import os
            self.get_logger().info('@@@@@@@@@@@@@@@@@@@@@@@@@@@ run_drcf')
            os.system(command)

def main(args=None):
    rclpy.init(args=args)
    node = ConnectionNode()
    rclpy.spin_once(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()