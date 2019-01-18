import serial
import rospy
from geometry_msgs.msg import Twist

def mapRange(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

class TurtleGlove:
    def __init__(self, serial):
        self.serial = serial
        rospy.init_node('TurtleGlove', anonymous=True)
        self.twist_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.twist_msg = Twist()
        self.data = []

    def publish(self):
        # add code for turtlebot !!
        self.twist_msg.linear.x = self.data[0] * -1
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0 
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = self.data[1]
        self.twist_publisher.publish(self.twist_msg)

    def getData(self):
        raw_datas = str(self.serial.readline()).strip('\n\r').split(',')
        self.data = []
        for raw_data in raw_datas:
            value = mapRange(float(raw_data), -90, 90, -1, 1)
            if value > -0.1 and value < 0.1: 
                value = 0
            elif value < -0.7 or value > 0.7:
                value = 0
            self.data.append(value)

if __name__ == '__main__':
    try:
        with serial.Serial('/dev/ttyACM0', 115200, timeout=2) as service:
            glove = TurtleGlove(service)
            while(True):
                try:

