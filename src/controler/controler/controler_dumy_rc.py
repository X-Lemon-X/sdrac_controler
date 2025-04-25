import rclpy
from rclpy.node import Node
import cantools
import can
from sensor_msgs.msg import JointState 
from sensor_msgs.msg import Joy
from  std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue, DiagnosticStatus, DiagnosticArray
import os

class ControlerDumyRC(Node):
  def __init__(self):
    super().__init__('cdumy_rc_node')
    self.publisher_joints = self.create_publisher(JointState, '/controls/sdrac/joint_control', 10)


    self.sub_joint_states = self.create_subscription(
      JointState,
      '/controls/sdrac/joint_states',
      self.joint_states_callback,
      10
    )

    self.sub_joint_states = self.create_subscription(
      Joy,
      '/joy',
      self.joy_callback,
      10
    )
    self.buttons_previous = [0,0,0,0,0,0,0,0,0,0]
    self.mode_pos = True
    self.publisher_control_mode_callback = self.create_publisher(String,'/controls/sdrac/control_mode',10)

  def joint_states_callback(self, msg: JointState):
    pass

  def check_if_button_pressed(self, buttons, index):
    pressed = False
    if buttons[index] != self.buttons_previous[index] and buttons[index] == 1:
      pressed = True

    self.buttons_previous[index] = buttons[index]
    return pressed

  def buttons_handler(self, buttons):
    msg = None
    if self.check_if_button_pressed(buttons, 2):
      msg = String()
      msg.data = "velocity_control"
      self.mode_pos = False
      self.publisher_control_mode_callback.publish(msg)
      self.get_logger().info(f"Change control mode to: \"{msg.data}\"")
    
    if self.check_if_button_pressed(buttons, 3):
      msg = String()
      msg.data = "position_control"
      self.mode_pos = True
      self.publisher_control_mode_callback.publish(msg)
      self.get_logger().info(f"Change control mode to: \"{msg.data}\"")
    
    if self.check_if_button_pressed(buttons, 4):
      msg = String()
      self.mode_pos = False
      msg.data = "torque_control"
      self.publisher_control_mode_callback.publish(msg)
      self.get_logger().info(f"Change control mode to: \"{msg.data}\"")

    
    self.buttons_previous = buttons

  def joy_callback(self, msg: Joy):
    if len(msg.axes) != 6:
      self.get_logger().error(f"joy message not enought axis: \"{len(msg.axes)}\" expected 6")
      return
    msg_joint = JointState()
    msg_joint.header.frame_id = "joint_control"
    msg_joint.header.stamp = self.get_clock().now().to_msg()


    if self.mode_pos:
      velocity = [0.5] * 6
      positons = [0.0] * 6
      # for axis in msg.axes:
      #   positons.append(axis)
      positons[0] = msg.axes[0]
      positons[1] = msg.axes[1]
      positons[2] = msg.axes[2]
      positons[3] = msg.axes[3]
      positons[4] = msg.axes[4]
      positons[5] = msg.axes[5]
    else:
      velocity = []
      positons = [0.0] * 6
      for axis in msg.axes:
        velocity.append(axis)
    
    msg_joint.velocity = velocity 
    msg_joint.effort = velocity
    msg_joint.position = positons
    # self.get_logger().info(f"joy message: \"{msg_joint}\"")
    self.buttons_handler(msg.buttons)
    self.publisher_joints.publish(msg_joint)


    pass
    # self.publisher_joints.publish(msg)
    
    
def main(args=None):
  rclpy.init(args=args)

  can_publsiher = ControlerDumyRC()

  rclpy.spin(can_publsiher)
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  can_publsiher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()