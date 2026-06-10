"""Local-planning utilities for the MORO maze-control assignment.

Implements the four sections of the MORO cookbook's local planner:
  * pose <-> homogeneous transform conversions (Determine Movement Goal),
  * generateControls (Generate Valid Control Signals),
  * forwardKinematics + PT2Block + costFn + evaluateControls
    (Simulate Outcome of Control Signals).

forwardKinematics and PT2Block are taken from the MORO Project Cookbook
("Simulate Outcome of Control Signals"), which in turn references the
class 4 exercise and Thrun, Probabilistic Robotics. They are reproduced
here as permitted by the cookbook (used like any other cited source).
"""

import copy
import math

import numpy as np


# --- Determine Movement Goal Relative to Robot ------------------------------

def pose2tf_mat(pose):
    """(x, y, theta) -> 2D homogeneous transform matrix.

    Layout is [ R(theta) | t ; 0 0 1 ], i.e. a 2x2 rotation in the top-left and
    the translation (x, y) in the last column. Expressing poses as matrices lets
    us compose/invert them with plain matrix multiplication (see
    goal_relative_to_robot).
    """
    x, y, theta = pose
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return np.array([
        [cos_t, -sin_t, x],
        [sin_t,  cos_t, y],
        [0.0,    0.0,   1.0],
    ], dtype=np.float64)


def tf_mat2pose(tf_mat):
    """2D homogeneous transform matrix -> (x, y, theta).

    x, y are read straight from the translation column; theta is recovered from
    the rotation block with atan2(sin, cos) = atan2(R[1,0], R[0,0]).
    """
    x = tf_mat[0, 2]
    y = tf_mat[1, 2]
    theta = math.atan2(tf_mat[1, 0], tf_mat[0, 0])
    return np.array([x, y, theta], dtype=np.float64)


def goal_relative_to_robot(robot_pose, goal_pose):
    """Express goal_pose (map frame) relative to robot_pose (map frame).

    Both transforms point map->frame, so the map->robot transform is
    inverted before multiplication: rel = inv(T_robot) @ T_goal.
    """
    tf_robot = pose2tf_mat(robot_pose)
    tf_goal = pose2tf_mat(goal_pose)
    rel = np.linalg.inv(tf_robot) @ tf_goal
    return tf_mat2pose(rel)


# --- Generate Valid Control Signals -----------------------------------------

def generateControls(last_control,
                     v_min=0.0, v_max=0.22, w_min=-1.4, w_max=1.4,
                     v_acc=0.1, w_acc=1.0, v_step=0.02, w_step=0.1):
    """Valid (vt, wt) candidates near the last applied control.

    The robot's inertia means the next command may not deviate too far from
    the last one, so the linear/angular velocities are sampled within
    [last +/- acc], clipped to the absolute [min, max] limits.
    """
    last_v, last_w = float(last_control[0]), float(last_control[1])

    v_lo = max(v_min, last_v - v_acc)
    v_hi = min(v_max, last_v + v_acc)
    w_lo = max(w_min, last_w - w_acc)
    w_hi = min(w_max, last_w + w_acc)

    eps = 1e-9
    v_values = np.arange(v_lo, v_hi + eps, v_step)
    w_values = np.arange(w_lo, w_hi + eps, w_step)
    if v_values.size == 0:
        v_values = np.array([np.clip(last_v, v_min, v_max)])
    if w_values.size == 0:
        w_values = np.array([np.clip(last_w, w_min, w_max)])

    controls = np.array([[v, w] for v in v_values for w in w_values], dtype=np.float64)
    return controls


# --- Simulate Outcome of Control Signals ------------------------------------

def forwardKinematics(control, lastPose, dt, dtype=np.float64):
    """Mobile robot forward kinematics (MORO cookbook; see Thrun,
    Probabilistic Robotics)."""
    if not isinstance(lastPose, np.ndarray):
        lastPose = np.array(lastPose, dtype=dtype)
    assert lastPose.shape == (3,), \
        "Wrong pose format. Pose must be [x, y, theta]"
    if not isinstance(control, np.ndarray):
        control = np.array(control)
    assert control.shape == (2,), \
        "Wrong control format. Control must be [vt, wt]"

    vt, wt = control
    # Avoid division by zero when omega is zero (straight-line motion).
    if wt == 0:
        wt = np.finfo(dtype).tiny
    # Exact integration of the unicycle model for a constant (vt, wt) over dt: the
    # robot follows a circular arc of radius R = vt/wt. The pose update below is the
    # closed-form solution of x' = v cos(theta), y' = v sin(theta), theta' = w.
    vtwt = vt / wt
    _, _, theta = lastPose
    return lastPose + np.array([
        -vtwt * np.sin(theta) + vtwt * np.sin(theta + (wt * dt)),
        vtwt * np.cos(theta) - vtwt * np.cos(theta + (wt * dt)),
        wt * dt,
    ], dtype=dtype)


class PT2Block:
    """Discrete PT2 block (Tustin approximation) modelling the robot's
    linear-velocity dynamics/inertia. (MORO cookbook; class 4 exercise.)"""

    def __init__(self, T=0, D=0, kp=1, ts=0, bufferLength=3):
        self.k1, self.k2, self.k3, self.k4, self.k5, self.k6 = 0, 0, 0, 0, 0, 0
        self.e = [0 for _ in range(bufferLength)]
        self.y = [0 for _ in range(bufferLength)]
        if ts != 0:
            self.setConstants(T, D, kp, ts)

    def setConstants(self, T, D, kp, ts):
        self.k1 = 4 * T ** 2 + 4 * D * T * ts + ts ** 2
        self.k2 = 2 * ts ** 2 - 8 * T ** 2
        self.k3 = 4 * T ** 2 - 4 * D * T * ts + ts ** 2
        self.k4 = kp * ts ** 2
        self.k5 = 2 * kp * ts ** 2
        self.k6 = kp * ts ** 2

    def update(self, e):
        self.e = [e] + self.e[:len(self.e) - 1]
        self.y = [0] + self.y[:len(self.y) - 1]
        e, y = self.e, self.y
        y[0] = (e[0] * self.k4 + e[1] * self.k5 + e[2] * self.k6
                - y[1] * self.k2 - y[2] * self.k3) / self.k1
        return y[0]


def default_state_weight():
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.5]], dtype=np.float64)


def default_control_weight():
    return np.array([[0.1, 0.0],
                     [0.0, 0.1]], dtype=np.float64)


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def costFn(pose, goalpose, control, Q=None, R=None):
    """Weighted quadratic cost of a (simulated) pose + control vs the goal.

    cost = e^T Q e + u^T R u, with e = |pose - goal| (theta error wrapped to
    [-pi, pi] first) and u = |control|. Lower is better; 0 == on the goal.
    """
    if Q is None:
        Q = default_state_weight()
    if R is None:
        R = default_control_weight()

    pose = np.asarray(pose, dtype=np.float64)
    goalpose = np.asarray(goalpose, dtype=np.float64)

    # State error e = |pose - goal|. The heading component is wrapped to [-pi, pi]
    # first so that, e.g., an error of 350 deg counts as -10 deg (small), not large.
    error = pose - goalpose
    error[2] = _wrap_angle(error[2])
    e = np.abs(error)
    u = np.abs(np.asarray(control, dtype=np.float64))

    # Quadratic form: e^T Q e penalises being far from the goal (Q weights x/y/theta),
    # u^T R u penalises control effort (R weights linear/angular velocity).
    return float(e.T @ (Q @ e) + u.T @ (R @ u))


def evaluateControls(controls, robotModelPT2, goalpose, horizon, ts,
                     Q=None, R=None):
    """Forward-simulate each control candidate over `horizon` steps and
    accumulate its cost. Returns (costs, trajectories), aligned with controls.

    Each candidate starts from the robot frame origin [0, 0, 0]; the linear
    velocity is passed through a copy of the current PT2 dynamic state so the
    robot's inertia is respected.
    """
    controls = np.asarray(controls, dtype=np.float64)
    costs = np.zeros(len(controls), dtype=np.float64)
    trajectories = [[] for _ in controls]

    for ctrl_idx, control in enumerate(controls):
        forward_pt2 = copy.deepcopy(robotModelPT2)
        forward_pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        v_t, w_t = control

        for _ in range(horizon):
            v_t_dynamic = forward_pt2.update(v_t)
            forward_pose = forwardKinematics([v_t_dynamic, w_t], forward_pose, ts)
            costs[ctrl_idx] += costFn(forward_pose, goalpose, control, Q, R)
            trajectories[ctrl_idx].append(forward_pose.copy())

    return costs, trajectories
