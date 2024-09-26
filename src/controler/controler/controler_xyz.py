import rclpy
from rclpy.node import Node
import cantools
import can
from sensor_msgs.msg import JointState 
from sensor_msgs.msg import Joy
from diagnostic_msgs.msg import KeyValue, DiagnosticStatus, DiagnosticArray
import os

class ControlerXYZ(Node):
  def __init__(self):
    super().__init__('can_node')
    self.publisher_joints = self.create_publisher(JointState, '/controls/sdrac/joint_control', 10)


    self.sub_joint_states = self.create_subscription(
      JointState,
      '/controls/sdrac/joint_states',
      self.joint_states_callback,
      10
    )

    self.sub_joint_states = self.create_subscription(
      JointState,
      '/controls/sdrac/joy',
      self.joy_callback,
      10
    )

  def joint_states_callback(self, msg: JointState):
    pass

  def joy_callback(self, msg: Joy):
    pass
    # self.publisher_joints.publish(msg)
    
    
def main(args=None):
  rclpy.init(args=args)

  can_publsiher = ControlerXYZ()

  rclpy.spin(can_publsiher)
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  can_publsiher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()