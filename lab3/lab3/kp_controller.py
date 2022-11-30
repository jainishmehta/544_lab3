from interfaces.srv import StartAndEnd
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
import rclpy
from rclpy.node import Node


class KpController(Node):

    def __init__(self):
        super().__init__('kp_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(TFMessage, '/tf', self.update_pose, 10)
        self.srv = self.create_service(StartAndEnd, 'start_and_end', self.start_controller)
        self.tf_to_odom_x = None
        self.tf_to_odom_y = None
        self.curr_x = None
        self.curr_y  =None

    def start_controller(self, req, resp):
        print(req.start.x)
        print(req.end.x)
        directions = astar(maze, (req.start.x, req.start.y), (req.end.x, req.end.y))
        resp.status = True
        return resp
    
    def update_pose(self, msg: TFMessage):
        for tf in msg.transforms:
            # Find the base footprint TF
            if tf.child_frame_id == "base_footprint":
                if not self.tf_to_odom_x:
                    # Update offset to tranform into odom frame
                    self.tf_to_odom_x = tf.transform.translation.x
                    self.tf_to_odom_y = tf.transform.translation.y
                # Update position
                self.curr_x = tf.transform.translation.x - self.tf_to_odom_x
                self.curr_y = tf.transform.translation.y - self.tf_to_odom_y
                # Output current pose
                self.get_logger().info("Current position: x: %.2f y: %.2f" % (self.curr_x, self.curr_y))
                break


def main():
    rclpy.init()

    controller = KpController()

    rclpy.spin(controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
