#!/usr/bin/env python3
"""Example action client for PlanAndExecute action.

Demonstrates planning with optional execution.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from arm_msgs.action import PlanAndExecute


class PlanAndExecuteClient(Node):
    def __init__(self):
        super().__init__('plan_execute_client_example')
        self._action_client = ActionClient(
            self,
            PlanAndExecute,
            '/planner_server/plan_and_execute'
        )

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Connected to action server')

    def send_goal(self, goal, execute=False):
        """Send a goal to the action server."""
        goal.execute = execute

        self.get_logger().info(f'Sending goal (execute={execute})...')

        send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return None

        self.get_logger().info('Goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return result_future.result().result

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Stage: {feedback.stage}, '
            f'Progress: {feedback.percent_complete:.1f}%, '
            f'Status: {feedback.status_message}'
        )

    def plan_to_pose(self, x, y, z, execute=False):
        """Plan to a pose target with optional execution."""
        goal = PlanAndExecute.Goal()
        goal.target_type = PlanAndExecute.Goal.TARGET_TYPE_POSE

        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.w = 1.0

        goal.target_pose = target_pose
        goal.planner_id = "RRTConnectkConfigDefault"
        goal.planning_time = 5.0
        goal.num_planning_attempts = 3
        goal.velocity_scaling_factor = 0.1
        goal.acceleration_scaling_factor = 0.1
        goal.execute = execute

        result = self.send_goal(goal, execute)

        if result and result.success:
            self.get_logger().info(
                f'Action completed successfully! '
                f'Planning time: {result.planning_time:.3f}s, '
                f'Waypoints: {result.waypoints_count}'
            )
            if execute:
                if result.execution_success:
                    self.get_logger().info('Trajectory executed successfully!')
                else:
                    self.get_logger().error(
                        f'Execution failed: {result.execution_message}'
                    )
        else:
            self.get_logger().error(f'Action failed: {result.message if result else "No result"}')

        return result

    def plan_to_joints(self, joint_values, execute=False):
        """Plan to joint configuration with optional execution."""
        goal = PlanAndExecute.Goal()
        goal.target_type = PlanAndExecute.Goal.TARGET_TYPE_JOINT
        goal.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        goal.joint_values = joint_values
        goal.planner_id = "RRTConnectkConfigDefault"
        goal.planning_time = 5.0
        goal.velocity_scaling_factor = 0.1
        goal.acceleration_scaling_factor = 0.1
        goal.execute = execute

        result = self.send_goal(goal, execute)

        if result and result.success:
            self.get_logger().info(
                f'Action completed! Waypoints: {result.waypoints_count}'
            )
            if execute and result.execution_success:
                self.get_logger().info('Trajectory executed!')

        return result


def main(args=None):
    rclpy.init(args=args)

    client = PlanAndExecuteClient()

    print("\n=== Plan and Execute Action Examples ===\n")

    # Example 1: Plan only (no execution)
    print("Example 1: Plan to pose (no execution)")
    client.plan_to_pose(x=0.3, y=0.2, z=0.5, execute=False)

    print("\n" + "="*50 + "\n")

    # Example 2: Plan and execute
    print("Example 2: Plan to joint configuration and execute")
    joint_values = [0.0, -0.3, 0.3, 0.0, 0.3, 0.0]
    client.plan_to_joints(joint_values, execute=True)

    print("\nAll examples completed!")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
