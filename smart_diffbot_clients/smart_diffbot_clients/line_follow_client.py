import rclpy
import os
import time
import sys

from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class LineFollowClient(Node):

    def __init__(self):

        # Start node 
        super().__init__('line_follow_client')

    def start(self, robot_name):

        # Start action client
        self.nav_client = ActionClient(self, NavigateToPose, robot_name+'/navigate_to_pose')

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()

        goal_msg.behavior_tree = os.path.join(
            get_package_share_directory('smart_diffbot_navigation'), 
            'behavior_trees', 'navigate_line_following.xml')

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = -8.0
        pose_msg.pose.position.y = 4.0
        goal_msg.pose = pose_msg

        self.get_logger().info('Waiting for Nav2 action server to come online...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server available, sending line following goal...')
        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 server')
            return

        self.get_logger().info('Goal accepted by Nav2 server, executing... ')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info(f"Done!")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Navigating... Distance remaining: {int(feedback.distance_remaining)} meters", 
                               throttle_duration_sec=1)
        time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    line_follow_client = LineFollowClient()

    # Defaults
    robot_name = ""

    # Handle any command line inputs 
    if len(sys.argv) > 1 :
        robot_name = sys.argv[1]

    # Start client
    line_follow_client.start(robot_name=robot_name)

    # Send action request to Nav2 
    line_follow_client.send_goal()

    # Spin (so the node won't shut down after the goal is sent)
    rclpy.spin(line_follow_client)


if __name__ == '__main__':
    main()