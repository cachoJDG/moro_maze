"""MORO local planner: drive the robot along the global path using forward
simulation (DWA-style), publishing velocity commands directly to /cmd_vel.

Pipeline per control cycle (MORO cookbook, "Putting everything together"):
  1. Locate the robot (tf map -> base_link).
  2. Select the current goal pose on the global path.
  3. Express the goal relative to the robot (homogeneous transforms).
  4. Generate valid control signals (vt, wt).
  5. Forward-simulate each control (forward kinematics + PT2) and score it.
  6. Pick the lowest-cost control and publish it to /cmd_vel.
  7. Publish the best trajectory and the relative goal for visualization.
On reaching the last goal pose, a (0, 0) command is sent to stop the robot.
"""

import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from moro_maze.control_utils import (
    PT2Block,
    evaluateControls,
    generateControls,
    goal_relative_to_robot,
)


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner_node')
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('trajectory_topic', '/local_trajectory')
        self.declare_parameter('goal_topic', '/current_goal')
        self.declare_parameter('control_rate', 5.0)        # [Hz] control loop
        self.declare_parameter('sim_ts', 0.2)              # [s] sim sampling time
        self.declare_parameter('horizon', 12)              # forward-sim steps
        self.declare_parameter('goal_tolerance', 0.25)     # [m] advance threshold
        self.declare_parameter('final_goal_tolerance', 0.20)
        # Velocity / acceleration limits for control generation.
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('w_min', -1.4)
        self.declare_parameter('w_max', 1.4)
        self.declare_parameter('v_acc', 0.1)
        self.declare_parameter('w_acc', 1.0)
        self.declare_parameter('v_step', 0.02)
        self.declare_parameter('w_step', 0.1)
        # PT2 robot dynamics model.
        self.declare_parameter('pt2_T', 0.05)
        self.declare_parameter('pt2_D', 0.8)
        # Cost weights. q_theta is 0 by default: for following maze waypoints we
        # care about reaching each POSITION, not its orientation. A non-zero
        # q_theta makes the robot rotate in place at corners (the 90 deg heading
        # error, squared, dominates the position error) instead of driving on.
        self.declare_parameter('q_pos', 1.0)
        self.declare_parameter('q_theta', 0.0)
        self.declare_parameter('r_v', 0.05)
        self.declare_parameter('r_w', 0.02)

        self.global_path = []
        self.current_goal_index = 0
        self.last_control = np.array([0.0, 0.0])
        self.finished = False

        q_pos = float(self.get_parameter('q_pos').value)
        q_theta = float(self.get_parameter('q_theta').value)
        self.Q = np.diag([q_pos, q_pos, q_theta])
        self.R = np.diag([
            float(self.get_parameter('r_v').value),
            float(self.get_parameter('r_w').value),
        ])

        ts = float(self.get_parameter('sim_ts').value)
        self.robot_pt2 = PT2Block(
            ts=ts,
            T=float(self.get_parameter('pt2_T').value),
            D=float(self.get_parameter('pt2_D').value),
        )

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = ReliabilityPolicy.RELIABLE

        self.cmd_pub = self.create_publisher(
            TwistStamped, self.get_parameter('cmd_vel_topic').value, 10)
        self.traj_pub = self.create_publisher(
            Path, self.get_parameter('trajectory_topic').value, 10)
        self.goal_pub = self.create_publisher(
            PoseStamped, self.get_parameter('goal_topic').value, 10)

        self.create_subscription(
            Path, self.get_parameter('global_path_topic').value,
            self.path_callback, path_qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        control_rate = max(0.5, float(self.get_parameter('control_rate').value))
        self.create_timer(1.0 / control_rate, self.control_loop)
        self.get_logger().info('local_planner_node alive')

    # --- path handling ------------------------------------------------------

    def path_callback(self, msg):
        # Convert the global path (PoseStamped list, map frame) into
        # (x, y, theta) goal poses. Theta is taken from the heading toward the
        # next waypoint so the robot finishes each leg facing the way it drives.
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not points:
            return

        path = []
        for index, (x, y) in enumerate(points):
            if index < len(points) - 1:
                nx, ny = points[index + 1]
                theta = math.atan2(ny - y, nx - x)
            elif path:
                theta = path[-1][2]
            else:
                theta = 0.0
            path.append((x, y, theta))

        self.global_path = path
        self.current_goal_index = 0
        self.finished = False
        self.last_control = np.array([0.0, 0.0])
        self.get_logger().info(
            f'local planner received global path: goals={len(self.global_path)}')

    # --- localization (map -> base_link) ------------------------------------

    def localise_robot(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

        q = trans.transform.rotation
        # Yaw from quaternion (z-axis rotation only).
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            yaw,
        ])

    # --- main control loop --------------------------------------------------

    def control_loop(self):
        if self.finished or not self.global_path:
            return

        robot_pose = self.localise_robot()
        if robot_pose is None:
            return

        # 2. Select the current goal, advancing over already-reached waypoints.
        goal_tol = float(self.get_parameter('goal_tolerance').value)
        final_tol = float(self.get_parameter('final_goal_tolerance').value)
        last_index = len(self.global_path) - 1

        while self.current_goal_index < last_index:
            gx, gy, _ = self.global_path[self.current_goal_index]
            if math.hypot(gx - robot_pose[0], gy - robot_pose[1]) < goal_tol:
                self.current_goal_index += 1
            else:
                break

        goal_pose = self.global_path[self.current_goal_index]

        # Stop once the final goal is reached.
        if self.current_goal_index == last_index:
            dist_final = math.hypot(goal_pose[0] - robot_pose[0],
                                    goal_pose[1] - robot_pose[1])
            if dist_final < final_tol:
                self.stop_robot()
                self.finished = True
                self.get_logger().info(
                    'local planner: final goal reached, robot stopped')
                return

        # 3. Goal relative to the robot.
        rel_goal = goal_relative_to_robot(robot_pose, goal_pose)

        # 4. Generate candidate controls.
        controls = generateControls(
            self.last_control,
            v_min=float(self.get_parameter('v_min').value),
            v_max=float(self.get_parameter('v_max').value),
            w_min=float(self.get_parameter('w_min').value),
            w_max=float(self.get_parameter('w_max').value),
            v_acc=float(self.get_parameter('v_acc').value),
            w_acc=float(self.get_parameter('w_acc').value),
            v_step=float(self.get_parameter('v_step').value),
            w_step=float(self.get_parameter('w_step').value),
        )
        if len(controls) == 0:
            return

        # 5. Forward-simulate and score.
        ts = float(self.get_parameter('sim_ts').value)
        horizon = int(self.get_parameter('horizon').value)
        costs, trajectories = evaluateControls(
            controls, self.robot_pt2, rel_goal, horizon, ts, Q=self.Q, R=self.R)

        # 6. Pick the lowest-cost control.
        best_idx = int(np.argmin(costs))
        best_control = controls[best_idx]

        # Advance the real dynamic state with the applied linear velocity.
        self.robot_pt2.update(float(best_control[0]))
        self.last_control = best_control

        # 7. Publish command, trajectory and relative goal.
        self.publish_cmd(best_control)
        self.publish_trajectory(trajectories[best_idx])
        self.publish_goal(rel_goal)

    # --- ROS publishing -----------------------------------------------------

    def publish_cmd(self, control):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(control[0])
        msg.twist.angular.z = float(control[1])
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.last_control = np.array([0.0, 0.0])
        self.publish_cmd(self.last_control)

    def publish_trajectory(self, trajectory):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        for pose in trajectory:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(pose[0])
            ps.pose.position.y = float(pose[1])
            ps.pose.orientation.z = math.sin(pose[2] / 2.0)
            ps.pose.orientation.w = math.cos(pose[2] / 2.0)
            msg.poses.append(ps)
        self.traj_pub.publish(msg)

    def publish_goal(self, rel_goal):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = float(rel_goal[0])
        msg.pose.position.y = float(rel_goal[1])
        msg.pose.orientation.z = math.sin(rel_goal[2] / 2.0)
        msg.pose.orientation.w = math.cos(rel_goal[2] / 2.0)
        self.goal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
