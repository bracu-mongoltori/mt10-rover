import rclpy
from rclpy.node import Node
import serial.tools.list_ports
import yaml

class PortFinderNode(Node):
    def __init__(self):
        super().__init__('port_finder_node')
        self.get_logger().info("Starting serial port search...")
        
        # Path to SBG config file
        self.config_path = '/home/mt10/mt10_ws/src/mt10_control/config/mt_sbg_config.yaml'
        
        # Find all ttyUSB ports
        ports = list(serial.tools.list_ports.grep(r'ttyUSB'))
        
        if ports:
            selected_port = ports[0].device  # Take first found USB port
            self.get_logger().info(f"Found SBG device at {selected_port}")
            self.update_config_file(selected_port)
        else:
            self.get_logger().error("No ttyUSB devices found!")

        # Shutdown the node after completing the task
        self.get_logger().info("Task completed - shutting down node")
        self.destroy_node()
        rclpy.shutdown()

    def update_config_file(self, port):
        try:
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Update port parameter
            config['/**']['ros__parameters']['uartConf']['portName'] = port
            
            with open(self.config_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            
            self.get_logger().info(f"Updated config file with port: {port}")
            
        except Exception as e:
            self.get_logger().error(f"Error updating config: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = PortFinderNode()
    
    try:
        rclpy.spin(node)  # Allow the node to execute its logic
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
