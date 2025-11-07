#!/usr/bin/env python3
"""Example client for the PlanMotion service.

Demonstrates how to plan to joint targets, pose targets, and Cartesian paths.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from arm_msgs.srv import PlanMotion


class PlannerClient(Node):
    def __init__(self):
        super().__init__('planner_client_example')
        self.client = self.create_client(PlanMotion, '/planner_server/plan_motion')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for planner service...')

        self.get_logger().info('Connected to planner service')

    def plan_to_joint_target(self, joint_names, joint_values):
        """Plan to a joint configuration."""
        request = PlanMotion.Request()
        request.target_type = PlanMotion.Request.TARGET_TYPE_JOINT
        request.joint_names = joint_names
        request.joint_values = joint_values
        request.planner_id = "RRTConnectkConfigDefault"
        request.planning_time = 5.0
        request.num_planning_attempts = 1
        request.velocity_scaling_factor = 0.1
        request.acceleration_scaling_factor = 0.1

        self.get_logger().info(f'Planning to joint target: {joint_values}')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            self.get_logger().info(
                f'Planning succeeded! Time: {response.planning_time:.3f}s, '
                f'Waypoints: {response.waypoints_count}, '
                f'Planner: {response.planner_used}'
            )
        else:
            self.get_logger().error(f'Planning failed: {response.message}')

        return response

    def plan_to_pose_target(self, x, y, z, frame_id="base_link"):
        """Plan to a Cartesian pose."""
        request = PlanMotion.Request()
        request.target_type = PlanMotion.Request.TARGET_TYPE_POSE

        target_pose = PoseStamped()
        target_pose.header.frame_id = frame_id
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.w = 1.0  # Identity quaternion

        request.target_pose = target_pose
        request.planner_id = "RRTConnectkConfigDefault"
        request.planning_time = 5.0
        request.num_planning_attempts = 3
        request.velocity_scaling_factor = 0.1
        request.acceleration_scaling_factor = 0.1

        self.get_logger().info(f'Planning to pose: [{x}, {y}, {z}]')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            self.get_logger().info(
                f'Planning succeeded! Time: {response.planning_time:.3f}s, '
                f'Waypoints: {response.waypoints_count}'
            )
        else:
            self.get_logger().error(f'Planning failed: {response.message}')

        return response

    def plan_cartesian_path(self, waypoints_xyz, frame_id="base_link"):
        """Plan a Cartesian path through waypoints."""
        request = PlanMotion.Request()
        request.target_type = PlanMotion.Request.TARGET_TYPE_CARTESIAN

        # Create waypoints
        waypoints = []
        for (x, y, z) in waypoints_xyz:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            waypoints.append(pose)

        request.waypoints = waypoints
        request.cartesian_max_step = 0.01  # 1cm steps
        request.cartesian_jump_threshold = 0.0  # No jump threshold
        request.velocity_scaling_factor = 0.1
        request.acceleration_scaling_factor = 0.1

        self.get_logger().info(f'Planning Cartesian path with {len(waypoints)} waypoints')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            self.get_logger().info(
                f'Planning succeeded! Time: {response.planning_time:.3f}s, '
                f'Waypoints: {response.waypoints_count}'
            )
        else:
            self.get_logger().error(f'Planning failed: {response.message}')

        return response

    def plan_deterministic(self, joint_names, joint_values, seed=12345):
        """Plan with deterministic mode (reproducible results)."""
        request = PlanMotion.Request()
        request.target_type = PlanMotion.Request.TARGET_TYPE_JOINT
        request.joint_names = joint_names
        request.joint_values = joint_values
        request.planner_id = "RRTConnectkConfigDefault"
        request.planning_time = 5.0
        request.deterministic = True
        request.random_seed = seed
        request.velocity_scaling_factor = 0.1
        request.acceleration_scaling_factor = 0.1

        self.get_logger().info(f'Planning with deterministic seed: {seed}')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            self.get_logger().info(
                f'Deterministic planning succeeded! Waypoints: {response.waypoints_count}'
            )
        else:
            self.get_logger().error(f'Planning failed: {response.message}')

        return response


def main(args=None):
    rclpy.init(args=args)

    client = PlannerClient()

    print("\n=== Planner Client Examples ===\n")

    # Example 1: Plan to joint target
    print("Example 1: Planning to joint configuration")
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    joint_values = [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]
    client.plan_to_joint_target(joint_names, joint_values)

    print("\n" + "="*50 + "\n")

    # Example 2: Plan to pose target
    print("Example 2: Planning to Cartesian pose")
    client.plan_to_pose_target(x=0.3, y=0.2, z=0.5)

    print("\n" + "="*50 + "\n")

    # Example 3: Plan Cartesian path
    print("Example 3: Planning Cartesian path")
    waypoints = [
        (0.3, 0.0, 0.5),
        (0.3, 0.1, 0.5),
        (0.3, 0.1, 0.6),
    ]
    client.plan_cartesian_path(waypoints)

    print("\n" + "="*50 + "\n")

    # Example 4: Deterministic planning
    print("Example 4: Deterministic planning")
    client.plan_deterministic(joint_names, joint_values, seed=42)

    print("\nAll examples completed!")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
