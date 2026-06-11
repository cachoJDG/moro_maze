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
    # --- Tuning constants (hardcoded; kept simple, no ROS parameters) -------
    GLOBAL_PATH_TOPIC = '/global_path'
    CMD_VEL_TOPIC = '/cmd_vel'
    TRAJECTORY_TOPIC = '/local_trajectory'
    GOAL_TOPIC = '/current_goal'

    CONTROL_RATE = 5.0          # [Hz] control loop
    SIM_TS = 0.2                # [s] forward-simulation sampling time
    HORIZON = 12                # forward-sim steps
    GOAL_TOLERANCE = 0.25        # [m] advance to the next waypoint
    # Tighter radius for the LAST waypoint than for the intermediate ones: a
    # large radius makes the robot declare "arrived" while still ~0.20 m short
    # and stop inside the maze, so it never clears the exit. Keep it small so it
    # drives right up to the final goal.
    FINAL_GOAL_TOLERANCE = 0.10  # [m] stop radius at the last waypoint
    # Once on the final leg, keep driving for up to this long to let the robot
    # finish exiting, then stop anyway so the run terminates cleanly even if it
    # cannot quite settle inside the tight radius.
    FINAL_GOAL_GRACE_SEC = 10.0  # [s] extra time at the last waypoint

    # Velocity / acceleration limits for control generation.
    V_MIN = 0.0
    V_MAX = 0.22
    W_MIN = -1.4
    W_MAX = 1.4
    V_ACC = 0.1
    W_ACC = 1.0
    V_STEP = 0.02
    W_STEP = 0.1

    # PT2 robot dynamics model.
    PT2_T = 0.05
    PT2_D = 0.8

    # Cost weights. Q_THETA is 0: for following maze waypoints we care about
    # reaching each POSITION, not its orientation. A non-zero Q_THETA makes the
    # robot rotate in place at corners (the 90 deg heading error, squared,
    # dominates the position error) instead of driving on.
    Q_POS = 1.0
    Q_THETA = 0.0
    R_V = 0.05
    R_W = 0.02

    def __init__(self):
        super().__init__('local_planner_node')

        self.global_path = []
        self.current_goal_index = 0
        self.last_control = np.array([0.0, 0.0])
        self.finished = False
        self.final_leg_start = None  # time we first reached the last waypoint

        self.Q = np.diag([self.Q_POS, self.Q_POS, self.Q_THETA])
        self.R = np.diag([self.R_V, self.R_W])

        self.robot_pt2 = PT2Block(ts=self.SIM_TS, T=self.PT2_T, D=self.PT2_D)

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = ReliabilityPolicy.RELIABLE

        self.cmd_pub = self.create_publisher(TwistStamped, self.CMD_VEL_TOPIC, 10)
        self.traj_pub = self.create_publisher(Path, self.TRAJECTORY_TOPIC, 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.GOAL_TOPIC, 10)

        self.create_subscription(
            Path, self.GLOBAL_PATH_TOPIC, self.path_callback, path_qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        control_rate = max(0.5, self.CONTROL_RATE)
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
        self.final_leg_start = None
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
        goal_tol = self.GOAL_TOLERANCE
        final_tol = self.FINAL_GOAL_TOLERANCE
        last_index = len(self.global_path) - 1

        while self.current_goal_index < last_index:
            cur = self.global_path[self.current_goal_index]
            nxt = self.global_path[self.current_goal_index + 1]
            d_cur = math.hypot(cur[0] - robot_pose[0], cur[1] - robot_pose[1])
            d_nxt = math.hypot(nxt[0] - robot_pose[0], nxt[1] - robot_pose[1])
            # Advance when the waypoint is reached, OR when we have already driven
            # past it (the next waypoint is now closer). Without the overshoot
            # check the robot can stall chasing a waypoint that ended up behind it
            # after passing it at speed.
            if d_cur < goal_tol or d_nxt < d_cur:
                self.current_goal_index += 1
            else:
                break

        goal_pose = self.global_path[self.current_goal_index]

        # Stop once the final goal is reached. On the final leg we (a) use the
        # tighter FINAL_GOAL_TOLERANCE so the robot drives close enough to clear
        # the exit, and (b) grant a grace period: keep driving for up to
        # FINAL_GOAL_GRACE_SEC to finish exiting, then stop regardless so the run
        # ends cleanly even if it cannot settle inside the tight radius.
        if self.current_goal_index == last_index:
            now = self.get_clock().now()
            if self.final_leg_start is None:
                self.final_leg_start = now
            elapsed = (now - self.final_leg_start).nanoseconds * 1e-9

            dist_final = math.hypot(goal_pose[0] - robot_pose[0],
                                    goal_pose[1] - robot_pose[1])
            if dist_final < final_tol:
                self.stop_robot()
                self.finished = True
                self.get_logger().info(
                    f'local planner: final goal reached (dist={dist_final:.3f} m), '
                    'robot stopped')
                return
            if elapsed > self.FINAL_GOAL_GRACE_SEC:
                self.stop_robot()
                self.finished = True
                self.get_logger().info(
                    f'local planner: final-goal grace ({self.FINAL_GOAL_GRACE_SEC:.0f}s) '
                    f'elapsed at dist={dist_final:.3f} m, robot stopped')
                return

        # 3. Goal relative to the robot.
        rel_goal = goal_relative_to_robot(robot_pose, goal_pose)

        # 4. Generate candidate controls.
        controls = generateControls(
            self.last_control,
            v_min=self.V_MIN, v_max=self.V_MAX,
            w_min=self.W_MIN, w_max=self.W_MAX,
            v_acc=self.V_ACC, w_acc=self.W_ACC,
            v_step=self.V_STEP, w_step=self.W_STEP,
        )
        if len(controls) == 0:
            return

        # 5. Forward-simulate and score.
        costs, trajectories = evaluateControls(
            controls, self.robot_pt2, rel_goal, self.HORIZON, self.SIM_TS,
            Q=self.Q, R=self.R)

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
