import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_ros_planning_interface import MoveGroupInterface


class EndEffectorControl(Node):
    def __init__(self):
        super().__init__('end_effector_control')
        self.move_group = MoveGroupInterface("manipulator", "robot_description", namespace='/')

    def move_to_position(self, target_pose):
        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.go(wait=True)
        if plan:
            self.get_logger().info("Motion executed successfully")
        else:
            self.get_logger().info("Failed to plan and execute motion")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = EndEffectorControl()

        # Define the end effector pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.5  # Define your desired x position
        target_pose.pose.position.y = 0.3  # Define your desired y position
        target_pose.pose.position.z = 0.6  # Define your desired z position
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0

        # Move end effector to the desired position
        node.move_to_position(target_pose)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

