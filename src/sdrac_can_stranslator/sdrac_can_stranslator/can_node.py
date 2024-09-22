import rclpy
from rclpy.node import Node
import cantools
import can
from sensor_msgs.msg import JointState 
from diagnostic_msgs.msg import KeyValue, DiagnosticStatus, DiagnosticArray
import os

class CanPublisher(Node):
  def __init__(self):
    super().__init__('can_node')
    self.publisher_joints = self.create_publisher(JointState, '/controls/joint_state', 10)
    self.publisher_status = self.create_publisher(DiagnosticArray, '/diagnostics/status', 10)
    self.publisher_errors = self.create_publisher(DiagnosticArray, '/diagnostics/errors', 10)

    self.sub_joint_set_callback = self.create_subscription(
      JointState,
      '/controls/joint_seters',
      self.joint_set_callback,
      10
    )
    
    self.python_file_dir = os.path.dirname(os.path.realpath(__file__))
    self.can_db_file = f'/home/lemonx/it/sdrac_controler/src/sdrac_can_stranslator/sdrac_can_stranslator/ariadna_constants/can_messages/output/can.dbc'

    self.can_time_out = 0.006
    self.can_interface_name = 'can0'
    self.can_bitrate = 100000
    self.can_bus = None
    self.konarms_can_messages = {}
    self.konarms_can_decode_functions = {}
    self.konarms_can_messages_id_to_msg = {}
    self.konarms_can_messages_id_to_number = {}
    self.konarm_axes = 6
    self.start_can()

    self.period_read_pos = 1/50
    self.period_read_error = 1/2
    self.period_read_state = 1/2
    self.period_read_can = 1/60
    self.timer_tc_can_read_get_pos = self.create_timer(self.period_read_pos, self.tc_can_read_pos)
    self.timer_tc_can_read_get_state = self.create_timer(self.period_read_state, self.tc_can_read_state)
    self.timer_tc_can_read_get_error = self.create_timer(self.period_read_error, self.tc_can_read_error)
    # self.timer_tc_can_read_can = self.create_timer(self.period_read_can, self.read_can_callback)
    # Set default values to 
    self.states = []
    self.positions = []
    self.velocities = []
    self.errors = []
    self.set_default_values()

    self.start_can()


  def set_default_values(self):
    self.states = [ {'status':'emergency_stop'} for _ in self.get_axis_range() ]
    self.positions = [ float('nan') for _ in self.get_axis_range() ]
    self.velocities = [ float('nan') for _ in self.get_axis_range() ]
    self.errors = [ self.errors_empty() for _ in self.get_axis_range() ]

  # def __exit__(self):
  #   self.stop_can()

  def __del__(self):
    self.stop_can()

  def get_axis_range(self) -> range:
    return range(1, self.konarm_axes + 1)

  def start_can(self):
    if not os.path.exists(self.can_db_file):
      msg = f'Can db file not found: {self.can_db_file}'
      self.get_logger().error(msg)
      raise FileNotFoundError(msg)
    
    self.can_db = cantools.database.load_file(self.can_db_file)
    self.can_bus = can.interface.Bus(self.can_interface_name, bustype='socketcan', bitrate=self.can_bitrate)
    self.konarms_can_messages = {}
    self.konarms_can_messages_id_to_msg = {}
    for i in self.get_axis_range():
      self.konarms_can_messages[f'konarm_{i}_status'] = self.can_db.get_message_by_name(f'konarm_{i}_status')
      self.konarms_can_messages[f'konarm_{i}_set_pos'] = self.can_db.get_message_by_name(f'konarm_{i}_set_pos')
      self.konarms_can_messages[f'konarm_{i}_get_pos'] = self.can_db.get_message_by_name(f'konarm_{i}_get_pos')
      self.konarms_can_messages[f'konarm_{i}_get_errors'] = self.can_db.get_message_by_name(f'konarm_{i}_get_errors')
      self.konarms_can_decode_functions[self.konarms_can_messages[f'konarm_{i}_status'].frame_id] = self.decode_status
      self.konarms_can_decode_functions[self.konarms_can_messages[f'konarm_{i}_get_pos'].frame_id] = self.decode_get_pos
      self.konarms_can_decode_functions[self.konarms_can_messages[f'konarm_{i}_get_errors'].frame_id] = self.decode_get_errors

    for frame in self.konarms_can_messages.values():
      self.konarms_can_messages_id_to_msg[frame.frame_id] = frame
      # Extract the number from the message name (e.g., 'konarm_1_status' -> 1)
      number = int(frame.name.split('_')[1])
      self.konarms_can_messages_id_to_number[frame.frame_id] = number

  def stop_can(self):
    if self.can_bus is not None:
      self.can_bus.shutdown()

  def errors_empty(self):
    return {
      'temp_engine_overheating': 'fault',
      'temp_driver_overheating': 'fault',
      'temp_board_overheating': 'fault',
      'temp_engine_sensor_disconnect': 'fault',
      'temp_driver_sensor_disconnect': 'fault',
      'temp_board_sensor_disconnect': 'fault',
      'encoder_arm_disconnect': 'fault',
      'encoder_motor_disconnect': 'fault',
      'board_overvoltage': 'fault',
      'board_undervoltage': 'fault',
      'can_disconnected': 'fault',
      'can_error': 'fault',
      'controler_motor_limit_position': 'fault',
    }

  def decode_status(self, msg):
    if msg is None:
      return
    number = self.konarms_can_messages_id_to_number[msg.arbitration_id]
    decoded = self.konarms_can_messages_id_to_msg[msg.arbitration_id].decode(msg.data)
    self.states[number - 1] = decoded
  
  def decode_get_pos(self, msg):
    if msg is None:
      return
    number = self.konarms_can_messages_id_to_number[msg.arbitration_id]
    decoded = self.konarms_can_messages_id_to_msg[msg.arbitration_id].decode(msg.data)
    self.positions[number - 1] = decoded['position']
    self.velocities[number - 1] = decoded['velocity']
  
  def decode_get_errors(self, msg):
    if msg is None:
      return
    number = self.konarms_can_messages_id_to_number[msg.arbitration_id]
    decoded = self.konarms_can_messages_id_to_msg[msg.arbitration_id].decode(msg.data)
    self.errors[number - 1] = decoded
  
  def read_can(self):
    try:
      msg = self.can_bus.recv(self.can_time_out)
      if msg is None:
        return
      # Check if the message is unknown  
      if msg.arbitration_id not in self.konarms_can_decode_functions:
        self.get_logger().warning(f"Received unnknown message on can: {self.can_interface_name}: {msg}")
        return
      # Decode the message with the appropriate function
      self.konarms_can_decode_functions[msg.arbitration_id](msg)
      frame = self.konarms_can_messages_id_to_msg[msg.arbitration_id]
      self.get_logger().info(f"Received: {frame.frame_id} {frame.name}")
  
    except can.exceptions.CanOperationError as e:
      self.get_logger().error(f"Error RX can:{self.can_interface_name} {e}")
    return 

  def can_send(self, msg_name_short:str,_data:list=None,_is_remote_frame:bool=False):
    for i in self.get_axis_range():
      try:
        msg = self.konarms_can_messages[f'konarm_{i}_{msg_name_short}']
        if _data is not None:
          _data = msg.encode(_data)
        
        msg_send = can.Message(arbitration_id=msg.frame_id,data=_data,is_remote_frame=_is_remote_frame,is_extended_id=msg.is_extended_frame)
        
        self.can_bus.send(msg_send)
        if _is_remote_frame:
          self.read_can()
      except can.exceptions.CanOperationError as e:
        self.get_logger().error(f"Error TX can:{self.can_interface_name} {e}")    

  def read_can_callback(self):
    self.read_can()


  def tc_can_read_pos(self):
    self.can_send('get_pos',_is_remote_frame=True)
    # self.read_can()
    ros_msg = JointState()
    ros_msg.header.stamp = self.get_clock().now().to_msg()
    ros_msg.header.frame_id = 'joint_state'
    # ros_msg.name = 'konarmgetpos'
    ros_msg.position = self.positions
    ros_msg.velocity = self.velocities
    self.publisher_joints.publish(ros_msg)

  def tc_can_read_state(self):
    self.can_send('status',_is_remote_frame=True)
    # self.read_can()
    ros_msg = DiagnosticArray()
    ros_msg.header.stamp = self.get_clock().now().to_msg()
    
    id = 0
    for state in self.states:
      status = DiagnosticStatus()
      status.name = f'konarm_{id}_status'
      if(state['status'] == 'ok'):
        status.level = DiagnosticStatus.OK
      else:
        status.level = DiagnosticStatus.ERROR
      status.message = str(state['status'])
      ros_msg.status.append(status)
      id += 1

    self.publisher_status.publish(ros_msg)

  def tc_can_read_error(self):
    self.can_send('get_errors',_is_remote_frame=True)
    # self.read_can()
    ros_msg = DiagnosticArray()
    ros_msg.header.stamp = self.get_clock().now().to_msg()
    ros_msg.header.frame_id = 'konarm_errors'
    id = 0
    for errors in self.errors:
      status = DiagnosticStatus()
      status.name = f'konarm_{id}_get_errors'
      status.message = 'errors_list'
      status.level = DiagnosticStatus.OK
      for key in errors.keys():
        kv = KeyValue()
        kv.key = key
        kv.value = str(errors[key])
        status.values.append(kv)
        if kv.value == 'fault':
          status.level = DiagnosticStatus.ERROR
      ros_msg.status.append(status)
      id += 1
    self.publisher_errors.publish(ros_msg)

  def joint_set_callback(self, msg:JointState):
    if len(msg.position) != self.konarm_axes:
      self.get_logger().error(f"Invalid number of positions: {len(msg)}")
      return
    if len(msg.velocity) != self.konarm_axes:
      self.get_logger().error(f"Invalid number of velocities: {len(msg)}")
      return
    
    for i in self.get_axis_range():
      self.can_send('set_pos',
                    {
                      "position" : msg.position[i - 1],
                      "velocity" : msg.velocity[i - 1]
                    })  
      

def main(args=None):
  rclpy.init(args=args)

  can_publsiher = CanPublisher()

  rclpy.spin(can_publsiher)
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  can_publsiher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()