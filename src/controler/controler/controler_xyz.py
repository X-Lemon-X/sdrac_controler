import rclpy
from rclpy.node import Node
import cantools
import can
from sensor_msgs.msg import JointState 
from sensor_msgs.msg import Joy
from  std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue, DiagnosticStatus, DiagnosticArray
import os
import numpy
from .sim import Kinematic6axisModel
import sys
from enum import Enum

class ControlMode(Enum):
  POSITION = 1
  VELOCITY = 2
  TARQUE = 3

class ControlerXYZ(Node):
  def __init__(self):
    super().__init__('can_node')
    tcp_x = 0
    tcp_y = 0
    tcp_z = 0
    link_1 = 145
    link_2 = 375
    link_3 = 375
    link_4 = 100
    self.model_pos = Kinematic6axisModel(link_1, link_2, link_3, link_4)
    self.model_inverse = Kinematic6axisModel(link_1, link_2, link_3, link_4)
    self.model_pos.set_tcp_vector(tcp_x, tcp_y, tcp_z)
    self.model_inverse.set_tcp_vector(tcp_x, tcp_y, tcp_z)

    self.increase_value_pos_param = self.declare_parameter('increase_value_pos', 50.0)
    self.increase_value_rot_param = self.declare_parameter('increase_value_rot', 0.5)
    self.default_velocity_param = self.declare_parameter('default_velocity', 0.5)

    # self.publisher_joints = self.create_publisher(JointState, '/controls/sdrac/joint_control', 10)
    self.publisher_joints_seters = self.create_publisher(JointState, '/controls/sdrac/joint_seters', 10)
    self.publisher_joints_states = self.create_publisher(JointState, '/controls/sdrac/joint_states', 10)
    self.publisher_joints_states_gui = self.create_publisher(JointState, '/joint_states', 10)
    self.publisher_control_mode_callback = self.create_publisher(String,'/controls/sdrac/control_mode',10)
    self.publisher_joints_control = self.create_publisher(JointState, '/controls/sdrac/joint_control', 10)
    
    self.sub_joint_geters = self.create_subscription(
      JointState,
      '/controls/sdrac/joint_geters',
      self.joint_geters_callback,
      10
    )

    self.first_pos_callculated = False
    self.pos_x_target = 0.0
    self.pos_y_target = 0.0
    self.pos_z_target = 0.0
    self.rot_roll_target = 0.0
    self.rot_pitch_target = 0.0
    self.rot_yaw_target = 0.0
    self.q1 = 0.0
    self.q2 = 0.0
    self.q3 = 0.0
    self.q4 = 0.0
    self.q5 = 0.0
    self.q6 = 0.0

    self.time_prev = self.get_clock().now().nanoseconds
    self.sub_joint_states = self.create_subscription(
      Joy,
      '/joy',
      self.joy_callback,
      10
    )

    self.status_subscription = self.create_subscription(
      DiagnosticArray,
      '/diagnostics/sdrac/status',
      self.status_callback,
      10
    )
    self.write_control_mode = self.create_timer(20, self.set_control_mode)
    self.control_mode = ControlMode.VELOCITY
    self.buttons_previous = [0,0,0,0,0,0,0,0]

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

  def translate_angles_from_robot_to_model(self, angles):
    angles = numpy.mod(angles, 2*numpy.pi)

    q1 = -angles[0]
    q2 = -angles[1] - numpy.pi/2
    q3 = -angles[2] + numpy.pi/2
    q4 = angles[3]
    q5 = angles[4]
    q6 = angles[5]
    # q4 no correction
    # q5 no correction 
    # q6 no correction
    return float(q1), float(q2), float(q3), float(q4), float(q5), float(q6)

  def translate_angles_from_model_to_robot(self, q1, q2, q3, q4, q5, q6):
    q1 = 2* numpy.pi -q1
    q2 = -q2 - numpy.pi/2
    q3 = -q3 + numpy.pi/2
    # q4 = q4
    # q5 = q5
    # q6 = q6
    # q4 no correction
    # q5 no correction 
    # q6 no correction
    q1 = numpy.mod(q1, 2*numpy.pi)
    q2 = numpy.mod(q2, 2*numpy.pi)
    q3 = numpy.mod(q3, 2*numpy.pi)
    q4 = numpy.mod(q4, 2*numpy.pi)
    q5 = numpy.mod(q5, 2*numpy.pi)
    q6 = numpy.mod(q6, 2*numpy.pi)
    return q1, q2, q3, q4, q5, q6

  def set_control_mode(self,mode):
    msg_conmtrol_mode = String()
    msg_conmtrol_mode.data = "position_control"
    self.publisher_control_mode_callback.publish(msg_conmtrol_mode)

  def pick_closest_value(self,values, solutions):
    distance_min = 1e10
    retun_value = ()
    i = 0
    for s in solutions:
      dist = 0
      for a in range(len(values)):
        dist += abs(values[a] - s[a])
      if dist <= distance_min:
        distance_min = dist
        retun_value = s
      i += 1
      # self.get_logger().info(f"Solution distance:{dist} {i}/{len(solutions)}: {s}")
    return retun_value, i

  def pick_shortest_path(self, angle_target, angle_current):
    dist = abs(angle_target - angle_current)
    dist2 =abs( ( angle_target - 2*numpy.pi) - angle_current)
    if dist2 < dist:
      angle = angle_target - 2*numpy.pi
    else:
      angle = angle_target
    return angle

  def optimize_rotations(self, q1, q2, q3, q4, q5, q6):
    cq1, cq2, cq3, cq4, cq5, cq6 = self.translate_angles_from_robot_to_model([self.q1, self.q2, self.q3, self.q4, self.q5, self.q6])
    q1 = self.pick_shortest_path(q1, cq1)
    q2 = self.pick_shortest_path(q2, cq2)
    q3 = self.pick_shortest_path(q3, cq3)
    q4 = self.pick_shortest_path(q4, cq4)
    q5 = self.pick_shortest_path(q5, cq5)
    q6 = self.pick_shortest_path(q6, cq6)
    return q1, q2, q3, q4, q5, q6

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
      self.control_mode = ControlMode.VELOCITY
    
    if self.check_if_button_pressed(buttons, 3):
      msg = String()
      msg.data = "position_control"
      self.mode_pos = True
      self.publisher_control_mode_callback.publish(msg)
      self.get_logger().info(f"Change control mode to: \"{msg.data}\"")
      self.control_mode = ControlMode.POSITION
    
    if self.check_if_button_pressed(buttons, 4):
      msg = String()
      self.mode_pos = False
      msg.data = "torque_control"
      self.publisher_control_mode_callback.publish(msg)
      self.get_logger().info(f"Change control mode to: \"{msg.data}\"")
      self.control_mode = ControlMode.TARQUE

    self.buttons_previous = buttons

  def xyz_control(self,msg: Joy):
    q1, q2, q3, q4, q5, q6 = self.translate_angles_from_robot_to_model([self.q1, self.q2, self.q3, self.q4, self.q5, self.q6])
    qs1, qs2,qs3,qs4,qs5,qs6 = self.translate_angles_from_model_to_robot(q1, q2, q3, q4, q5, q6)
    qa1, qa2, qa3, qa4, qa5, qa6 = self.translate_angles_from_robot_to_model([qs1, qs2, qs3, qs4, qs5, qs6])
    self.get_logger().info(f"""
                           Model angles: q1: {self.q1}, q2: {self.q2}, q3: {self.q3}, q4: {self.q4}, q5: {self.q5}, q6: {self.q6}
                           Robot angles:q1: {q1},  q2: {q2},  q3:{q3}, q4: {q4}, q5: {q5}, q6: {q6}
                          Model angles: q1: {qs1}, q2: {qs2}, q3:{qs3}, q4: {qs4}, q5: {qs5}, q6: {qs6}
                                   RA : q1: {qa1}, q2:{qa2},  q3:{qa3}, q4:{qa4},  q5:{qa5},  q6:{qa6}""")


    time_diff = ( self.get_clock().now().nanoseconds - self.time_prev)/1e9
    self.time_prev = self.get_clock().now().nanoseconds

    self.model_pos.set_angles(q1, q2, q3, q4, q5, q6)
    sol = self.model_pos.get_euler_zyz_angles_solutions()
    x ,y, z = self.model_pos.get_cordinates()
    roll, pitch, yaw = sol[0]

    if not self.first_pos_callculated:
      s, index = self.pick_closest_value([0,0,0],sol)
      roll, pitch, yaw = s
      self.get_logger().info("First position callculated")
      self.pos_x_target = x
      self.pos_y_target = y
      self.pos_z_target = z
      self.get_logger().info(f"Model position First: x: {x}, y: {y}, z: {z}, roll: {roll}, pitch: {pitch}, yaw: {yaw}")
      self.rot_roll_target = roll
      self.rot_pitch_target = pitch
      self.rot_yaw_target = yaw
      self.first_pos_callculated = True

    # roll,pitch,yaw = self.pick_closest_value((self.rot_roll_target, self.rot_pitch_target, self.rot_yaw_target), sol)
    roll, pitch, yaw = sol[0]
    # self.rot_roll_target = roll
    # self.rot_pitch_target = pitch
    # self.rot_yaw_current = yaw

    increment_pos = self.increase_value_pos_param.get_parameter_value().double_value
    increment_rot = self.increase_value_rot_param.get_parameter_value().double_value
    self.pos_x_target += msg.axes[0] * increment_pos * time_diff
    self.pos_y_target += msg.axes[1] * increment_pos * time_diff
    self.pos_z_target += msg.axes[2] * increment_pos * time_diff
    self.rot_roll_target += msg.axes[3] * increment_rot * time_diff
    self.rot_pitch_target += msg.axes[4] * increment_rot * time_diff
    self.rot_yaw_target += msg.axes[5] * increment_rot * time_diff
    # Decrease the number of crapy positions
    self.pos_x_target = round(self.pos_x_target, 1)
    self.pos_y_target = round(self.pos_y_target, 1)
    self.pos_z_target = round(self.pos_z_target, 1)
    self.rot_roll_target = round(self.rot_roll_target, 4)
    self.rot_pitch_target = round(self.rot_pitch_target, 4)
    self.rot_yaw_target = round(self.rot_yaw_target, 4)


    # Calculate the inverse kinematic model
    self.model_inverse.set_pos_and_rot(self.pos_x_target, self.pos_y_target, self.pos_z_target, self.rot_roll_target, self.rot_pitch_target, self.rot_yaw_target)
    solutions = self.model_inverse.get_inverse_kinematics_solutions()
    if len(solutions) != 0 and len(solutions) != 2:
      upper_solutions = solutions[len(solutions)//2:]
      s , index= self.pick_closest_value([q1, q2, q3, q4, q5, q6], upper_solutions)
      for a in solutions:
        self.get_logger().info(f"Solution: {a}")
      self.get_logger().info(f"Index: {index}")
      # sols = len(solutions)
      # if len(solutions) != 1:
      #   sols = (sols // 2)
      # else:
      #   sols = 0
      # sols = 0
      q1, q2, q3, q4, q5, q6 = s  

    q1, q2, q3, q4, q5, q6 = self.translate_angles_from_model_to_robot(q1, q2, q3, q4, q5, q6)
    sq1 = round(q1,3)
    sq2 = round(q2,3)
    sq3 = round(q3,3)
    sq4 = round(q4,3)
    sq5 = round(q5,3)
    sq6 = round(q6,3)
    # q1, q2, q3, q4, q5, q6 = self.optimize_rotations(q1, q2, q3, q4, q5, q6)

    self.get_logger().info(
      f"""
        Current X: {x}, Y: {y}, Z: {z}, Roll: {round(roll,4)}, Pitch: {round(pitch,4)}, Yaw: {round(yaw,4)} Euler: {len(sol)} 
        Current rot: q1: {round(self.q1,3)}, q2: {round(self.q2,3)}, q3: {round(self.q3,3)}, q4: {round(self.q4,3)}, q5: {round(self.q5,3)}, q6: {round(self.q6,3)}
        Target X: {self.pos_x_target}, Y: {self.pos_y_target}, Z: {self.pos_z_target}, Roll: {self.rot_roll_target}, Pitch: {self.rot_pitch_target}, Yaw: {self.rot_yaw_target}
        Target rot: q1: {round(q1,3)}, q2: {round(q2,3)}, q3: {round(q3,3)}, q4: {round(q4,3)}, q5: {round(q5,3)}, q6: {round(q6,3)}
        Target Inv rot: q1: {sq1}, q2: {sq2}, q3: {sq3}, q4: {sq4}, q5: {sq5}, q6: {sq6}
        Inverse kinematic solutions: {len(solutions)} { "NO SOLUTIONS" if len(solutions) == 0 else "" }
      """)
    

    velocity =  float(self.default_velocity_param.get_parameter_value().double_value)
    q1 = float(q1)
    q2 = float(q2)
    q3 = float(q3)
    q4 = float(q4)
    q5 = float(q5)
    q6 = float(q6)
    msg_joint = JointState()
    msg_joint.header.frame_id = "joint_control"
    msg_joint.header.stamp = self.get_clock().now().to_msg()

    # q4, q5, q6 = self.translate_to_diff_drive(q4, q5, q6)
    positions = [ q1, q2, q3, q4, q5, q6]
    velocities = [velocity, velocity, velocity, velocity, velocity, velocity]
    efforts = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    msg_joint.position = positions
    msg_joint.velocity = velocities
    msg_joint.effort = efforts
    self.publisher_joints_seters.publish(msg_joint)

  def velocity_control(self,msg: Joy):
    msg_joint = JointState()
    msg_joint.header.frame_id = "joint_control"
    msg_joint.header.stamp = self.get_clock().now().to_msg()
    velocity = []
    positons = [0.0] * 6
    for axis in msg.axes:
      velocity.append(axis)
    
    msg_joint.velocity = velocity 
    msg_joint.effort = velocity
    msg_joint.position = positons
    # self.get_logger().info(f"joy message: \"{msg_joint}\"")
    self.publisher_joints_seters.publish(msg_joint)

  def joy_callback(self, msg: Joy):
    if len(msg.axes) != 6:
      self.get_logger().error(f"joy message not enought axis: \"{len(msg.axes)}\" expected 6")
      return

    self.buttons_handler(msg.buttons)

    if self.control_mode == ControlMode.POSITION:
      self.xyz_control(msg)
      return
    
    if self.control_mode == ControlMode.VELOCITY:
      self.velocity_control(msg)
      return

    if self.control_mode == ControlMode.TARQUE:
      self.get_logger().warning("Torque control not implemented")
      return


  def status_callback(self, msg: DiagnosticArray):
    ms = msg.status[0]
    if ms.name == "konarm_connection":
      self.first_pos_callculated = False if ms.level == DiagnosticStatus.ERROR else self.first_pos_callculated
    pass

  def joint_geters_callback(self, msg: JointState):
    # This function is called when a message is received on /controls/sdrac/joint_geters
    # The message is then translated to the corresponding joints and send to /controls/sdrac/joint_seters
    # The message is also send to /controls/sdrac/joint_states
    joint_4_position = msg.position[3]
    joint_5_position = msg.position[4]
    joint_6_position = msg.position[5]
    # Translate the velocity of the robot to the velocity of the joints
    # j4_p, j5_p, j6_p = self.tranlate_from_diff_drive_to(joint_4_position, joint_5_position, joint_6_position)
    j4_p = joint_4_position
    j5_p = -joint_5_position
    j6_p = joint_6_position

    # map velocity to position
    joint_4_velocity = msg.velocity[3]
    joint_5_velocity = msg.velocity[4]
    joint_6_velocity = msg.velocity[5]
    # Translate the velocity of the robot to the velocity of the joints
    # j4_v, j5_v, j6_v = self.tranlate_from_diff_drive_to(joint_4_velocity, joint_5_velocity, joint_6_velocity)
    j4_v = joint_4_velocity
    j5_v = -joint_5_velocity
    j6_v = joint_6_velocity
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
    # self.publisher_joints_states.publish(reponse)
    self.q1 = msg.position[0]
    self.q2 = msg.position[1]
    self.q3 = msg.position[2]
    self.q4 = msg.position[3]
    self.q5 = msg.position[4]
    self.q6 = msg.position[5]

    # calcualte kinematic model
    q1, q2, q3, q4, q5, q6 = self.translate_angles_from_robot_to_model(reponse.position)
    q1 = float(q1)
    q2 = float(q2)
    q3 = float(q3)
    q4 = float(q4)
    q5 = float(q5)
    q6 = float(q6)
    self.model_pos.set_angles(q1, q2, q3, q4, q5, q6)
    # x,y,z = self.model_pos.get_cordinates()
    # roll, pitch, yaw = self.model_pos.get_euler_zyz_angles_solutions()
    # self.get_logger().info(f"Model position: x: {x}, y: {y}, z: {z}, roll: {roll}, pitch: {pitch}, yaw: {yaw}")

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

  can_publsiher = ControlerXYZ()

  rclpy.spin(can_publsiher)
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  can_publsiher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()