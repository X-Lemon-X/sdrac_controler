import os
import sys
from rclpy.node import Node

# Import the receiver_ros module from the Pilot_6_axis.ros package
sys.path.append(os.path.join(os.path.dirname(__file__), 'Pilot_6_axis', 'ros'))
from .Pilot_6_axis.ros import receiver_ros

def main():
  receiver_ros.main()

if __name__ == "__main__":
  main()