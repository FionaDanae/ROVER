import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class RoverArm(Node):

    def __init__(self):
        super().__init__('rover_arm')
        self.ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1)

        self.sub = self.create_subscription(
            String,
            'arm_cmd',
            self.callback,
            10)

    def callback(self,msg):
        self.ser.write((msg.data + "\n").encode())

def main():
    rclpy.init()
    node = RoverArm()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
