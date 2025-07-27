import os
import subprocess
import can
import time


class CanHat:
  r"""
    script to automatically activate and setup CANable-MKS interface on linux machines.
    

    Parameters
    ==========
    name_of_can_interface : str
        Name of the CAN interface to init the can with.
    
    bitrate : int
        Bitrate of the CAN interface. in bits per second.
        available bitrates are: 1000000, 800000, 500000, 250000, 125000, 100000, 50000, 20000, 10000

    logger : logging.Logger
      ros2 logger object

    Examples
    ========
        >>> from can_device import CanHat
        >>> can_device = CanHat()
        >>> can_device.init_can_interface()

    Tips
    ====
    - You can run this script with sudo to avoid permission errors. or the script will ask you to enter your password.
    - or just run thsi script directly it will probably work.
    >>> python3 can_device.py
  """

  RED = '\033[0;31m'
  GREEN = '\033[0;32m'
  BLUE = '\033[0;34m'
  ORANGE = '\033[0;33m'
  NC = '\033[0m' 

  def __init__(self, name_of_can_interface:str="can0", bitrate:int=1000000, logger=None):
    ## Class to set up CAN interface
    # @param name_of_can_interface: Name of the CAN interface
    # @param bitrate: Bitrate of the CAN interface
    
    self.vendor_id = "16d0"
    self.product_id = "117e"
    self.vendor_enc = "Openlight\x20Labs"
    self.name_of_can_interface = name_of_can_interface
    self.bitrate = bitrate
    self.logger = logger

    self.map_bitrate_to_parameter = {
      1000000: "s8",
      800000: "s7",
      500000: "s6",
      250000: "s5",
      125000: "s4",
      100000: "s3",
      50000: "s2",
      20000: "s1",
      10000: "s0",
    }

    if bitrate not in self.map_bitrate_to_parameter:
      raise ValueError(f"Unsupported bitrate: \"{bitrate}\" \nsupported bitrates are: {self.map_bitrate_to_parameter.keys()}")

  def check_if_can_interface_up(self) -> bool:
    result = subprocess.run(['ip', 'addr', 'show', self.name_of_can_interface], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return "UP" in str(result.stdout)
  
  def log_info(self, msg, color=None):
    if self.logger:
      self.logger.info(msg)
    else:
      print(f"{color if color else ''}{msg}{self.NC}")
  
  def log_error(self, msg, color=None):
    if self.logger:
      self.logger.error(msg)
    else:
      print(f"{color if color else ''}{msg}{self.NC}")

  def init_can_interface(self):
    # Check if CAN interface is UP
    if self.check_if_can_interface_up():
      self.log_info("Can is already UP!", self.ORANGE)
    else:  
      self.__check_if_program_is_installed("slcand", "can-utils")
      self.__check_if_program_is_installed("ifconfig", "net-tools")
      self.__can_up()
    # return can.interface.Bus(self.name_of_can_interface, interface='socketcan', bitrate=self.bitrate)

  def __check_if_program_is_installed(self, program, package):
    result = subprocess.run(['which', program], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode != 0:
      self.log_error(f"Error program: \"{program}\" is not installed.", self.RED)
      subprocess.run(['sudo', 'apt-get', 'install', package, '-y'])
    else:
      self.log_info(f"Program: \"{program}\" is installed", self.BLUE)

  def __can_set_up_interface(self, device_path):
    ret = subprocess.run(['sudo', 'slcand', '-o', '-c', f'-{self.map_bitrate_to_parameter[self.bitrate]}', device_path, self.name_of_can_interface])
    if ret.returncode == 0:
      self.log_info("Can interface configurated successfully!", self.GREEN)
      return
    msg = "Error setting up can interface"
    self.log_error(msg, self.RED)
    raise ValueError(msg)

  def __can_up_device(self):
    ret = subprocess.run(['sudo', 'ip', 'link', 'set', 'dev', self.name_of_can_interface, 'up', 'type', 'can', 'bitrate', str(self.bitrate)])
    if ret.returncode == 0:
      self.log_info("Can interface set up successfully!", self.GREEN)
      return
    self.log_error("Error setting up can interface", self.RED)
    self.log_info("Trying to set up can interface in compatibility mode", self.ORANGE)
    ret = subprocess.run(['sudo', 'ip', 'link', 'set', 'up', self.name_of_can_interface])
    if ret.returncode == 0:
      self.log_info("Can interface set up successfully!", self.GREEN)
      return
    msg = "Error setting up can interface in compatibility mode"
    self.log_error(msg, self.RED)
    raise ValueError(msg)

  def __can_config_txqueuelen(self):
    result = subprocess.run(['sudo', 'ifconfig', self.name_of_can_interface, 'txqueuelen', '1000'])
    if result.returncode != 0:
      self.log_error("Error setting txqueuelen", self.RED)
      raise ValueError("Error setting txqueuelen")
    else:
      self.log_info("Can interface txqueuelen set successfully!", self.GREEN)

  def __get_can_device_path(self):
    devs = [dev for dev in os.listdir('/dev') if 'tty' in dev]
    device_path = None
    # Iterate over all devices
    for dev in devs:
      device_info = subprocess.run(['udevadm', 'info', f'/dev/{dev}'], stdout=subprocess.PIPE).stdout.decode()
      venid = self.vendor_id if f"ID_VENDOR_ID={self.vendor_id}" in device_info else ""
      prodid = self.product_id if f"ID_MODEL_ID={self.product_id}" in device_info else ""
      if not venid or not prodid:
        continue
      device_path = f"/dev/{dev}"
      break
    return device_path

  def __can_up(self):
    device_path = self.__get_can_device_path()
    if not device_path:
      self.log_error("No can-hat found", self.RED)
    else:
      self.log_info(f"Can-hat found on: {device_path}", self.GREEN)
    self.__can_set_up_interface(device_path)
    time.sleep(0.1)
    self.__can_up_device()
    time.sleep(0.1)
    self.__can_config_txqueuelen()
    wait_time = 0.1
    self.log_info(f"CAN waiting {wait_time} seconds for can-hat to start working!", self.GREEN)
    time.sleep(wait_time)
  
if __name__ == "__main__":
  can_device = CanHat()
  can_device.init_can_interface()
