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
    self.publisher_joints_seters = self.create_publisher(JointState, '/controls/sdrac/joint_seters', 10)
    self.publisher_joints_states = self.create_publisher(JointState, '/controls/sdrac/joint_states', 10)
    self.publisher_joints_states_gui = self.create_publisher(JointState, '/joint_states', 10)

    self.sub_joint_control= self.create_subscription(
      JointState,
      '/controls/sdrac/joint_control',
      self.joint_control_callback,
      10
    )
    
    self.sub_joint_geters = self.create_subscription(
      JointState,
      '/controls/sdrac/joint_geters',
      self.joint_geters_callback,
      10
    )

  def translate_to_diff_drive(self, roll_4, roll_5, roll_6):
    # This function is used to translate the velocity of the robot to the velocity of the joints
    # 3 jouitn forming differential drive
    out_5 = (roll_5 + roll_6)/2
    out_6 = (roll_6 - roll_5)/2
    # corection fot the 4th joint
    out_4 = roll_4
    # one of this wil most likelky be + and the other - but that depend from the setup of the robot 
    out_5 = out_5 + out_4
    out_6 = out_6 + out_4
    return out_4, out_5, out_6

  def tranlate_from_diff_drive_to(self, in_4, in_5, in_6):
    #  inferse of the   translate_reading_for_diff_drive
    #  this function is used to translate the velocity of the joints to the velocity of the robot 
    rot_4 = in_4
    # corection fot the 4th joint that rotates the 5th and 6th joint
    in_5 = in_5 + rot_4
    in_6 = in_6 + rot_4
    # inverse of differential drive
    rot_5 = (in_6 - in_5 )/2 
    rot_6 = (in_5 + in_6)/2
    return rot_4, rot_5, rot_6
  
  def joint_control_callback(self, msg: JointState):
    # This function is called when a message is received on /controls/sdrac/joint_control
    # The message is then translated to the corresponding joints and send to /controls/sdrac/joint_seters
    # The message is also send to /controls/sdrac/joint_states
    # self.get_logger().info(f"joint_control_callback: \"{msg}\"")
    joint_1_position = msg.position[0]
    joint_2_position = msg.position[1]
    joint_3_position = msg.position[2]
    joint_4_position = msg.position[3]
    joint_5_position = msg.position[4]
    joint_6_position = msg.position[5]
    # Translate the velocity of the robot to the velocity of the joints
    joint_4_position, joint_5_position, joint_6_position = self.translate_to_diff_drive(joint_4_position, joint_5_position, joint_6_position)

    # map velocity to position
    joint_1_velocity = msg.velocity[0]  
    joint_2_velocity = msg.velocity[1]
    joint_3_velocity = msg.velocity[2]
    joint_4_velocity = msg.velocity[3]
    joint_5_velocity = msg.velocity[4]
    joint_6_velocity = msg.velocity[5]
    # Translate the velocity of the robot to the velocity of the joints
    # joint_4_velocity, joint_5_velocity, joint_6_velocity = self.translate_to_diff_drive(joint_4_velocity, joint_5_velocity, joint_6_velocity)
    reponse = JointState()
    reponse.header.stamp = self.get_clock().now().to_msg()
    positions = [
      joint_1_position,
      joint_2_position,
      joint_3_position,
      joint_4_position,
      joint_5_position,
      joint_6_position
    ]

    velocity = [
      joint_1_velocity,
      joint_2_velocity,
      joint_3_velocity,
      joint_4_velocity,
      joint_5_velocity,
      joint_6_velocity
    ]

    reponse.position = positions
    reponse.velocity = velocity
    reponse.effort = msg.effort
    reponse.name = msg.name
    # self.get_logger().info(f"joint_control_callback: \"{reponse}\"")
    self.publisher_joints_seters.publish(reponse)

  def joint_geters_callback(self, msg: JointState):
    # This function is called when a message is received on /controls/sdrac/joint_geters
    # The message is then translated to the corresponding joints and send to /controls/sdrac/joint_seters
    # The message is also send to /controls/sdrac/joint_states
    joint_4_position = msg.position[3]
    joint_5_position = msg.position[4]
    joint_6_position = msg.position[5]
    # Translate the velocity of the robot to the velocity of the joints
    j4_p, j5_p, j6_p = self.tranlate_from_diff_drive_to(joint_4_position, joint_5_position, joint_6_position)

    # map velocity to position
    joint_4_velocity = msg.velocity[3]
    joint_5_velocity = msg.velocity[4]
    joint_6_velocity = msg.velocity[5]
    # Translate the velocity of the robot to the velocity of the joints
    j4_v, j5_v, j6_v = self.tranlate_from_diff_drive_to(joint_4_velocity, joint_5_velocity, joint_6_velocity)
    names = [
      "Rev1",
      "Rev2",
      "Rev3",
      "Rev4",
      "Rev5",
      "Rev6"
    ]
    reponse = JointState()
    reponse.header.stamp = self.get_clock().now().to_msg()
    reponse.position = msg.position
    reponse.velocity = msg.velocity
    reponse.effort = msg.effort
    reponse.name = names
    reponse.position[3] = j4_p
    reponse.position[4] = j5_p
    reponse.position[5] = j6_p
    reponse.velocity[3] = j4_v
    reponse.velocity[4] = j5_v
    reponse.velocity[5] = j6_v
    self.publisher_joints_states.publish(reponse)
    # self.get_logger().info(f"joint_control_callback: \"{reponse}\"")


    # Publish the message to the GUI
    reponse = JointState()
    reponse.header.stamp = self.get_clock().now().to_msg()
    reponse.position = msg.position
    reponse.velocity = []
    reponse.effort = []
    reponse.name = names
    reponse.position.reverse()
    reponse.velocity.reverse()
    reponse.name.reverse()

    self.publisher_joints_states_gui.publish(reponse)

    
    
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