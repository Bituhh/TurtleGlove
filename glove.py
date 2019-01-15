import serial
import rospy
from geometry_msgs.msg import Twist

class TurtleGlove:
    def __init__(self, serial):
        self.serial = serial
        rospy.init_node('TurtleGlove', anonymous=True)
        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist_msg = Twist()
        self.doSomething()

    def doSomething(self):
        # add code for turtlebot !!
        self.twist_msg.linear.x = 50
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0 
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0
        self.twist_publisher.publish(self.twist_msg)
        
        print(self.serial.readline())

if __name__ == '__main__':
    with serial.Serial('/dev/ttyACM0', 115200, timeout=2) as service:
        glove = TurtleGlove(service)
        
        while ( True ) :
            try:
                glove.doSomething()
            except RuntimeError: 
                print(RuntimeError)
                break


