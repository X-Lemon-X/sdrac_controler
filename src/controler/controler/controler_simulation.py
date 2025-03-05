import rclpy
from rclpy.node import Node
import cantools
import can
from sensor_msgs.msg import JointState 
from diagnostic_msgs.msg import KeyValue, DiagnosticStatus, DiagnosticArray
import os


class ControlerTranslatorNode(Node):
  # This class is a translation of th veclotiy or position send to /controls/sdrac/joint_control
  # to equivaleun on specific joints that is selnd to /controls/sdrac/joint_seters
  # why becouse of the use of differential drive
  #
  # message send on /controls/sdrac/joint_control
  # is has 6 values velocity for each joint hovwver this are not the same as the joints bercouse of the differential drive
  #
  # 
  def __init__(self):
    super().__init__('can_node')
    self.publisher_joints_geters = self.create_publisher(JointState, '/controls/sdrac/joint_geters', 10)
    self.sub_joint_geters = self.create_subscription(
      JointState,
      '/controls/sdrac/joint_seters',
      self.joint_seters_callback,
      10
    )
    
    self.target_pos = [0.0] * 6
    self.velocity = [0.0] * 6
    self.position = [0.0] * 6
    self.time_diff = self.get_clock().now().to_msg().nanosec
    self.timer_geters = self.create_timer(0.04, self.timer_geters_callback)

  def timer_geters_callback(self):
    # This function is called every 0.05s and sends the message to /controls/sdrac/joint_geters
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    # tt = self.get_clock().now().to_msg().nanosec
    # time_diff = tt - self.time_diff
    # self.time_diff = tt
    # time_diff = time_diff / 1.0e9
    
    # positions = self.position
    # for i in range(6):
    #   ts = self.target_pos[i]
    #   if ts < positions[i]:
    #     positions[i] -= abs(self.velocity[i]) * time_diff
    #   else:
    #     positions[i] += abs(self.velocity[i]) * time_diff

    #   if abs(positions[i]-self.target_pos[i]) <= 1e-4:
    #     positions[i] = self.target_pos[i]
    # self.position = positions
    # self.get_logger().info(f"Position: {self.position}")
    
    
    msg.position = self.target_pos
    msg.velocity = self.velocity
    self.publisher_joints_geters.publish(msg)

  def joint_seters_callback(self, msg: JointState):
    # This function is called when a message is received on /controls/sdrac/joint_geters
    # The message is then translated to the corresponding joints and send to /controls/sdrac/joint_seters
    # The message is also send to /controls/sdrac/joint_states
    self.velocity = msg.velocity
    self.target_pos = msg.position
    
    
def main(args=None):
  rclpy.init(args=args)

  can_publsiher = ControlerTranslatorNode()

  rclpy.spin(can_publsiher)
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  can_publsiher.drc_nodeestroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()