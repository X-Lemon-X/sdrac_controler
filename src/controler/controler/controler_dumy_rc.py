import rclpy
from rclpy.node import Node
import cantools
import can
from sensor_msgs.msg import JointState 
from sensor_msgs.msg import Joy
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

  def joint_states_callback(self, msg: JointState):
    pass

  def joy_callback(self, msg: Joy):
    if len(msg.axes) != 6:
      self.get_logger().error(f"joy message not enought axis: \"{len(msg.axes)}\" expected 6")
      return
    msg_joint = JointState()
    msg_joint.header.frame_id = "joint_control"
    msg_joint.header.stamp = self.get_clock().now().to_msg()
    velocity = []
    for axis in msg.axes:
      velocity.append(axis)
    msg_joint.velocity = velocity 
    msg_joint.effort = []
    positons = [ 0 for _ in range(5)]
    msg_joint.position = positons
    self.get_logger().info(f"joy message: \"{msg_joint}\"")
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