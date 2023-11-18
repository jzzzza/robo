import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from enum import Enum, auto
import time

class State(Enum):
    TO_THE_FIRST_WALL = auto()
    ROTATING = auto()
    TO_THE_SECOND_WALL = auto()
    STOP = auto()

class Tb3(Node):
    ROBOT_WIDTH = 0.287
    TOLERANCE = 0.01
    ROTATION_SPEED_PERCENT = 30  # Angular velocity for rotation
    FORWARD_SPEED_PERCENT = 50   # Linear velocity for moving forward
    ROTATION_TIME = 3            # Time to rotate (seconds)
    rotation_start_time = None   # To keep track of rotation start time

    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        self.state = State.TO_THE_FIRST_WALL

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        MAX_LIN_VEL = 0.26  # m/s
        MAX_ANG_VEL = 1.82  # rad/s

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)

    def scan_callback(self, msg):
        front_distance = msg.ranges[0]
        error = front_distance - self.ROBOT_WIDTH / 2 - self.TOLERANCE

        if self.state == State.TO_THE_FIRST_WALL:
            if error > 0:
                self.vel(self.FORWARD_SPEED_PERCENT, 0)
            else:
                self.vel(0, 0)
                self.state = State.ROTATING
                self.rotation_start_time = time.time()

        elif self.state == State.ROTATING:
            current_time = time.time()
            if current_time - self.rotation_start_time < self.ROTATION_TIME:
                self.vel(0, self.ROTATION_SPEED_PERCENT)
            else:
                self.vel(0, 0)
                self.state = State.TO_THE_SECOND_WALL

        elif self.state == State.TO_THE_SECOND_WALL:
            # Assuming the second wall is straight ahead after rotation
            if error > 0:
                self.vel(self.FORWARD_SPEED_PERCENT, 0)
            else:
                self.vel(0, 0)
                self.state = State.STOP

        elif self.state == State.STOP:
            self.vel(0, 0)

def main(args=None):
    rclpy.init(args=args)
    tb3 = Tb3()
    print('waiting for messages...')

    try:
        rclpy.spin(tb3)
    except KeyboardInterrupt:
        pass

    tb3.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

