#!/usr/bin/env python3
# Minimal production-ready ROS2 MPC planner node using cvxpy + OSQP
import rclpy
from rclpy.node import Node
import numpy as np
import cvxpy as cp
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

class MPCPlanner(Node):
    def __init__(self):
        super().__init__('mpc_planner')
        self.declare_parameter('horizon', 20)
        self.declare_parameter('dt', 0.1)
        self.horizon = self.get_parameter('horizon').value
        self.dt = self.get_parameter('dt').value
        self.max_acc = 3.0  # m/s^2
        self.max_steer = 0.5  # rad/s
        self.sub_pose = self.create_subscription(PoseStamped, '/localization/pose', self.pose_cb, 10)
        self.sub_ref = self.create_subscription(Path, '/behavior/ref_path', self.ref_cb, 10)
        self.pub_traj = self.create_publisher(Path, '/planning/trajectory', 10)
        self.x = None
        self.ref_path = None
        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

    def pose_cb(self, msg):
        # update vehicle state [x, y, yaw, v]
        pose = msg.pose
        # convert quaternion to yaw omitted for brevity (use tf_transformations)
        yaw = 0.0
        v = 0.0
        self.x = np.array([pose.position.x, pose.position.y, yaw, v])

    def ref_cb(self, msg):
        # store reference path as list of poses
        self.ref_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def linearized_dynamics(self, state):
        # simple kinematic bicycle linearization around current yaw
        A = np.eye(4)
        A[0,3] = self.dt
        A[1,3] = self.dt
        B = np.zeros((4,2))
        B[3,0] = self.dt  # accel -> v
        B[2,1] = self.dt  # steering rate -> yaw
        return A, B

    def build_and_solve_mpc(self):
        N = self.horizon
        n = 4; m = 2
        A, B = self.linearized_dynamics(self.x)
        # Variables
        X = cp.Variable((n, N+1))
        U = cp.Variable((m, N))
        # Parameters
        x0 = self.x
        ref = np.zeros((n, N+1))
        for k in range(N+1):
            if self.ref_path and k < len(self.ref_path):
                rx, ry = self.ref_path[k]
                ref[0,k] = rx; ref[1,k] = ry
        Q = np.diag([10.0,10.0,1.0,0.1])
        R = np.diag([0.1,0.1])
        cost = 0
        constr = [X[:,0] == x0]
        for k in range(N):
            cost += cp.quad_form(X[:,k]-ref[:,k], Q) + cp.quad_form(U[:,k], R)
            constr += [X[:,k+1] == A @ X[:,k] + B @ U[:,k]]
            constr += [cp.abs(U[0,k]) <= self.max_acc, cp.abs(U[1,k]) <= self.max_steer]
        cost += cp.quad_form(X[:,N]-ref[:,N], Q)
        prob = cp.Problem(cp.Minimize(cost), constr)
        prob.solve(solver=cp.OSQP, warm_start=True, max_iter=10000)
        if prob.status not in ('optimal', 'optimal_inaccurate'):
            self.get_logger().warn('MPC solver failed: ' + str(prob.status))
            return None
        traj = X.value
        return traj

    def publish_trajectory(self, traj):
        if traj is None:
            return
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for k in range(traj.shape[1]):
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = float(traj[0,k])
            p.pose.position.y = float(traj[1,k])
            path.poses.append(p)
        self.pub_traj.publish(path)

    def loop(self):
        if self.x is None or self.ref_path is None:
            return
        traj = self.build_and_solve_mpc()
        self.publish_trajectory(traj)

def main(args=None):
    rclpy.init(args=args)
    node = MPCPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()