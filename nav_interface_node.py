import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
class FlexibleNavigator(Node):
    def __init__(self):
        super().__init__('flexible_navigator')
        self.navigator = TurtleBot4Navigator()
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        self.subscription  # prevent unused variable warning
    def goal_pose_callback(self, msg):
        self.get_logger().info('Received goal pose, navigating to the position')
        if not self.navigator.getDockedStatus():
            self.navigator.undock()
        # Navigate to the received goal pose
        self.navigator.startToPose(msg)
    def check_nav2_active_and_set_initial(self):
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before initializing pose')
            self.navigator.dock()
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
def main():
    rclpy.init()
    flexible_navigator = FlexibleNavigator()
    flexible_navigator.check_nav2_active_and_set_initial()
    # Spin to keep the script from exiting until this node is stopped
    rclpy.spin(flexible_navigator)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
