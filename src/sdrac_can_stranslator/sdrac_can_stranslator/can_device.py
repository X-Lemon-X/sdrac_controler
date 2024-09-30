import os
import subprocess

class CanDevice:
  RED = '\033[0;31m'
  GREEN = '\033[0;32m'
  BLUE = '\033[0;34m'
  ORANGE = '\033[0;33m'
  NC = '\033[0m'  # No Color

  def __init__(self, name_of_can_interface:str="can0", bitrate:int=1000000):
    self.vendor_id = "16d0"
    self.product_id = "117e"
    self.vendor_enc = "Openlight\x20Labs"
    self.name_of_can_interface = name_of_can_interface
    self.bitrate = bitrate

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
    
  def check_if_program_is_installed(self, program, package):
    result = subprocess.run(['which', program], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode != 0:
      print(f"{self.RED}Error program: \"{program}\" is not installed.{self.NC}")
      subprocess.run(['sudo', 'apt-get', 'install', package, '-y'])
    else:
      print(f"{self.BLUE} Program: \"{program}\" is installed{self.NC}")

  def __can_set_up_interface(self,device_path):
    ret = subprocess.run(['sudo', 'slcand', '-o', '-c', f'-{self.map_bitrate_to_parameter[self.bitrate]}', device_path, self.name_of_can_interface])
    if ret.returncode == 0:
      print(f"{self.GREEN}Can interface configurated successfully!{self.NC}")
      return 
    print(f"{self.RED}Error setting up can interface{self.NC}")
  
  def __can_up_device(self):
    ret = subprocess.run(['sudo', 'ip', 'link', 'set', 'dev', self.name_of_can_interface, 'up', 'type', 'can', 'bitrate', str(self.bitrate)])
    if ret.returncode == 0:
      print(f"{self.GREEN}Can interface set up successfully!{self.NC}")
      return
    print(f"{self.RED}Error setting up can interface{self.NC}")
    print(f"{self.ORANGE}Trying to set up can interface in compatibility mode{self.NC}")
    ret = subprocess.run(['sudo', 'ip', 'link', 'set', 'up', self.name_of_can_interface])
    if ret.returncode == 0:
      print(f"{self.GREEN}Can interface set up successfully!{self.NC}")
      return
    print(f"{self.RED}Error setting up can interface in compatibility mode {self.NC}")

  def __can_config_txqueuelen(self):
    result = subprocess.run(['sudo', 'ifconfig', self.name_of_can_interface, 'txqueuelen', '1000'])
    if result.returncode != 0:
      print(f"{self.RED}Error setting txqueuelen {self.NC}")
    else:
      print(f"{self.GREEN}Can intrface txqueuelen set successfully!{self.NC}")

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
      print(f"{self.RED}No can-hat found{self.NC}")
      return
    else:
      print(f"{self.GREEN}Can-hat found on: {device_path}{self.NC}")
    self.__can_set_up_interface(device_path)
    self.__can_up_device()
    self.__can_config_txqueuelen()

  def setup_can(self):
    # Check if CAN interface is UP
    result = subprocess.run(['ip', 'addr', 'show', self.name_of_can_interface], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode == 0:
      print(f"{self.ORANGE}Can is already UP!{self.NC}")
      return

    self.check_if_program_is_installed("slcand", "can-utils")
    self.check_if_program_is_installed("ifconfig", "net-tools")
    self.__can_up()
    

if __name__ == "__main__":
  can_device = CanDevice()
  can_device.setup_can()
