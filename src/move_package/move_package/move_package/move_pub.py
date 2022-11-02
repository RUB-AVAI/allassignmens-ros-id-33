import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import keyboard


class GoForward(Node):

    def forward(self, lin, ang):
        self.get_logger().info("Moving - lin : {} ang : {}".format(lin,ang))
        
        # Twist is a datatype for velocity
        move_cmd = Twist()
        
        # let's go forward 
        move_cmd.linear.x = lin
        
        # let's turns
        move_cmd.angular.z = ang
        
        # publish the velocity
        self.move_publisher.publish(move_cmd)

    def move_callback(self):
        if keyboard.is_pressed("left arrow"):
            self.forward(0, 1)
        elif keyboard.is_pressed("right arrow"):
            self.forward(0, -1)
        elif keyboard.is_pressed("down arrow"):
            self.forward(-0.1, 0)
        elif keyboard.is_pressed("up arrow"):
            self.forward(0.1, 0)
        elif keyboard.is_pressed("space"):
            self.forward(0, 0)

    def __init__(self, node_name: str = "GoForward"):
        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        super().__init__(node_name)
        self.move_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

	    # as long as you haven't ctrl + c keeping doing...
        self.timer_ = self.create_timer(1, self.move_callback)


def main():
    rclpy.init()
    node = GoForward()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
