import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from mt_arm_interfaces.msg import ControlMsgs
from std_msgs.msg import Bool, String




class JoyInputReader(Node):
    def __init__(self):
        super().__init__('Control_Node')

        # Subcribers 
        self.subscription = self.create_subscription(Joy,'/joy',self.joy_callback,1)

        # Publisher
        self.teleop  = self.create_publisher(Twist, '/cmd_vel', 1)
        self.robotic_arm = self.create_publisher(ControlMsgs, '/robotic_arm', 1)
        self.arm_disarm = self.create_publisher(Bool, '/arm_disarm', 1)
        self.mode = self.create_publisher(String, '/mode', 1)
        
        
        
        # Initialization
        self.BUTTONS = {
                "A"      : 0,
                "B"      : 1,
                "X"      : 2,
                "Y"      : 3,
                "LB"     : 4,
                "RB"     : 5,
                "BACK"   : 6,
                "START"  : 7,
                "L_JOY"  : 8,
                "R_JOY"  : 9
                }

        self.AXES = {
            "L_H" : 0,
            "L_V" : 1,
            "R_H" : 3,
            "R_V" : 4,
            "LT"  : 2,
            "RT"  : 5,
            "D_H" : 6,
            "D_V" : 7

        }
        
        # Arm Disarm  
        self.ARM = Bool()
        self.ARM.data = False
        
        # Message
        # Robotica Arm Message
        self.robotic_arm_msg = ControlMsgs()
        self.robotic_arm_msg.base      = 0.0
        self.robotic_arm_msg.link1     = 0.0
        self.robotic_arm_msg.link2     = 0.0
        self.robotic_arm_msg.d1        = 0.0
        self.robotic_arm_msg.d2        = 0.0
        self.robotic_arm_msg.claw      = 0.0
        # Teleop Message
        self.teleop_msg = Twist()
        self.teleop_msg.linear.x = 0.0
        self.teleop_msg.angular.z = 0.0
        
        # Default Scale
        self.default_scale = 100.0
        self.d1_d2_scale = 100.0
        self.trigger = False

    def joy_callback(self, msg):
        # Check if the control is enabled
        self.sudo_control(msg)


                
    def sudo_control(self, msg):
        
        # Change the mode
        if msg.axes[self.AXES['LT']] != 1 and msg.axes[self.AXES['RT']] != 1:
            self.mode_msg = String()
            self.mode_msg.data = "AUTO"
            self.mode.publish(self.mode_msg)
            self.get_logger().info("AUTO")
        elif msg.axes[self.AXES['RT']] != 1:
            self.mode_msg = String()
            self.mode_msg.data = "MANUAL"
            self.mode.publish(self.mode_msg)
            self.get_logger().info("MANUAL")
        elif msg.axes[self.AXES['LT']] != 1:
            self.mode_msg = String()
            self.mode_msg.data = "SIMPLE"
            self.mode.publish(self.mode_msg)
            self.get_logger().info("SIMPLE")


        
        # Arm and Disarm the ROVER
        if msg.buttons[self.BUTTONS['START']] == 1:
            self.ARM.data = True
            self.arm_disarm.publish(self.ARM)
            self.get_logger().info("ARM")
        if msg.buttons[self.BUTTONS['BACK']] == 1:
            self.ARM.data = False
            self.arm_disarm.publish(self.ARM)
            self.get_logger().info("DISARM")


        # Claw control
        # Close -- Open
        # OPEN
        if msg.buttons[self.BUTTONS['LB']] == 1:
            self.robotic_arm_msg.claw = msg.buttons[self.BUTTONS['LB']] * 100.0
            if self.robotic_arm_msg.claw > 100:
                self.robotic_arm_msg.claw = 100.0
            self.get_logger().info("Claw Opening")
        # CLOSE
        elif msg.buttons[self.BUTTONS['RB']] == 1:
            self.robotic_arm_msg.claw = msg.buttons[self.BUTTONS['RB']] * -100.0
            if self.robotic_arm_msg.claw < -100:
                self.robotic_arm_msg.claw = -100.0
            self.get_logger().info("Claw CLosing")
        else:
            self.robotic_arm_msg.claw = 0.0
        

        # D1 and D2 Control
        # D1
        if msg.buttons[self.BUTTONS['Y']] == 1:
            self.robotic_arm_msg.d1 = 100.0
            self.get_logger().info("Ender Vector UP")
        elif msg.buttons[self.BUTTONS['A']] == 1:
            self.robotic_arm_msg.d1 = -100.0
            self.get_logger().info("Ender Vector Down")
        else:
            self.robotic_arm_msg.d1 = 0.0
        
        # D2
        if msg.buttons[self.BUTTONS['B']] == 1:
            self.robotic_arm_msg.d2 = 100.0
            self.get_logger().info("Ender Vector right")
        elif msg.buttons[self.BUTTONS['X']] == 1:
            self.robotic_arm_msg.d2 = -100.0
            self.get_logger().info("Ender Vector left")
        else:
            self.robotic_arm_msg.d2 = 0.0


        # Link 2 Control
        if msg.axes[self.AXES['D_V']] !=  0:
            self.robotic_arm_msg.link2 = msg.axes[self.AXES['D_V']] * self.default_scale
            self.get_logger().info("Link2 UP")
        else:
            self.robotic_arm_msg.link2 = 0.0
        

        # LINK 1 Control
        if msg.axes[self.AXES['R_V']] != 0:
            self.robotic_arm_msg.link1 = msg.axes[self.AXES['R_V']] * self.default_scale
            self.get_logger().info("Link1 Moves")
        else:
            self.robotic_arm_msg.link1 = 0.0


        # Base Control
        if msg.axes[self.AXES['D_H']] != 0.0:
            self.robotic_arm_msg.base = -msg.axes[self.AXES['D_H']] * self.default_scale
            self.get_logger().info("Base Moves")
        else:
            self.robotic_arm_msg.base = 0.0

        # Control the ROVER
        self.teleop_msg.linear.x = -msg.axes[self.AXES['L_V']] * self.default_scale
        self.teleop_msg.angular.z = msg.axes[self.AXES['R_H']] * self.default_scale
          
        # Publish
        self.robotic_arm.publish(self.robotic_arm_msg)
        self.teleop.publish(self.teleop_msg)
        
        
        

def main(args=None):
    rclpy.init(args=args)
    joy_input_reader = JoyInputReader()
    rclpy.spin(joy_input_reader)
    joy_input_reader.destroy_node()
    rclpy.shutdown()
try:
    if __name__ == '__main__':
        main()
except KeyboardInterrupt:
    sys.exit(0)