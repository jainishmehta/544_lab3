from interfaces.srv import StartAndEnd
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
import rclpy
import numpy as np

from rclpy.node import Node
import math
import numpy as np
import cv2



class Pose():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        # save unit vector as heading
        self.heading = [math.cos(theta), math.sin(theta)]

    def __str__(self) -> str:
        return f"x: {self.x} y: {self.y} theta: {self.theta*180/math.pi}"

class KpController(Node):
    def __init__(self):
        super().__init__('kp_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(TFMessage, '/tf', self.update_pose, 10)
        self.timer = self.create_timer(0.05, self.kp_controller)
        self.srv = self.create_service(StartAndEnd, 'start_and_end', self.start_controller)
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.create_costmap, 10)
        self.created_map = False
        self.tf_to_odom_x = None
        self.tf_to_odom_y = None
        self.curr_pose = Pose()
        # Node has pixel coordinates as positions
        # (end point) -> (2nd last point) -> ... -> (start point)
        self.path_nodes = []

        # Resolution of the map (currently arbituary)
        self.resolution = 0.05
        # Map width and height
        self.map_width = 310
        self.map_height = 195
        # Bottom left coordinate wrt to odom frame
        self.origin_x = -5.75
        self.origin_y = -3.72

        # Error threshold
        self.error_threshold = 0.05

        # Constant linear vel
        self.lin_vel = 0.1

        # Kp controller constant
        self.kp = 3
        self.maze = None

    def start_controller(self, req, resp):
        print(req.start.x)
        print(req.end.x)
        resp.status = True
        return resp

    def kp_controller(self):
        ang_vel = 0
        if len(self.path_nodes)>0:
            # Get last node
            target_node = self.path_nodes[-1]
            target_px_x, target_px_y = target_node.position
            target_odom_x, target_odom_y = self.pixel_to_cartesian(target_px_x, target_px_y)
            
            # Check if tolerance is met
            if (self.euclidean_distance(target_odom_x, target_odom_y, self.curr_x, self.curr_y) < self.error_threshold):
                # Remove the node, and continue for the next node on next iteration
                self.path_nodes.pop()
            else:
                # does not meet tolerance, calculate error in heading
                desired_heading = self.get_desired_heading(target_odom_x, target_odom_y)
                theta_error = math.acos(np.dot(desired_heading, self.curr_pose.heading))
                # determine direction of error
                direction = 1 if np.cross(self.curr_pose.heading, desired_heading)>0 else -1
                if abs(theta_error) > 0.01:
                    ang_vel = theta_error*direction*self.kp
                msg = Twist()
                msg.linear.x = self.lin_vel
                msg.angular.z = ang_vel
                self.publisher.publish(msg)
        else:
            # no nodes, stop sending signal
            msg = Twist()
            self.publisher.publish(msg)
    
    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt(pow(x1-x2,2)+pow(y1-y2,2))

    def pixel_to_cartesian(self, px_x, px_y):
        odom_x = (px_x*self.resolution)+self.origin_x
        odom_y = (self.map_height*self.resolution+self.origin_y)-(px_y*self.resolution)
        return odom_x, odom_y

    def get_desired_heading(self, target_x, target_y):
        delta_x = target_x-self.curr_pose.x
        delta_y = target_y-self.curr_pose.y
        desired_heading = math.atan2(delta_y, delta_x)
        return [math.cos(desired_heading), math.sin(desired_heading)]

    def update_pose(self, msg: TFMessage):
        for tf in msg.transforms:
            # Find the base footprint TF
            if tf.child_frame_id == "base_footprint":
                if not self.tf_to_odom_x:
                    # Update offset to tranform into odom frame
                    self.tf_to_odom_x = tf.transform.translation.x
                    self.tf_to_odom_y = tf.transform.translation.y
                # Update position
                self.curr_pose.x = tf.transform.translation.x - self.tf_to_odom_x
                self.curr_pose.y = tf.transform.translation.y - self.tf_to_odom_y

                # Update heading 
                q_x = tf.transform.rotation.x
                q_y = tf.transform.rotation.y
                q_z = tf.transform.rotation.z
                q_w = tf.transform.rotation.w

                siny_cosp = 2 * (q_w * q_z + q_x * q_y)
                cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
                # curr_theta ranges from -pi to pi
                self.curr_pose.theta = math.atan2(siny_cosp, cosy_cosp)
                # Output current pose
                self.get_logger().info("Current Position: " + str(self.curr_pose))
                break

    def create_costmap(self, msg: OccupancyGrid):
        if (not self.maze):
            # Save data from costmap topic
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin = msg.info.origin
            data = np.array(msg.data)

            # Reshape array according to width and height
            reshaped_data = np.reshape(data, (height, width)).astype('float32')

            # Determine new image dimentions for downscaling
            resize_factor = 0.2 / resolution
            new_width = int(width / resize_factor)
            new_height = int(height / resize_factor)

            # Rescale and flip the data
            rescaled_data = cv2.resize(reshaped_data, (new_width, new_height))
            flipped_data = cv2.flip(rescaled_data, 0)
            
            self.maze = flipped_data

            cv2.imwrite("testimg.png", flipped_data)        


def main():
    rclpy.init()

    controller = KpController()

    rclpy.spin(controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()