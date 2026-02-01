#!/usr/bin/env python3
# Turtlebot code from ME274A

from turtle import width
import numpy as np
import typing as T
import rclpy                    # ROS2 client library
from numpy import linalg
from rclpy.node import Node     # ROS2 node baseclass
from scipy.interpolate import splev, splrep
import matplotlib.pyplot as plt

from asl_tb3_lib.navigation import BaseNavigator
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.tf_utils import quaternion_to_yaw
from asl_tb3_lib.navigation import TrajectoryPlan
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
from asl_tb3_lib.grids import snap_to_grid, StochOccupancyGrid2D

class AStar(object):
    """Represents a motion planning problem to be solved using A*"""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1):
        self.statespace_lo = statespace_lo         # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = statespace_hi         # state space upper bound (e.g., [5, 5])
        self.occupancy = occupancy                 # occupancy grid (a DetOccupancyGrid2D object)
        self.resolution = resolution               # resolution of the discretization of state space (cell/m)
        self.x_offset = x_init                     
        self.x_init = self.snap_to_grid(x_init)    # initial state
        self.x_goal = self.snap_to_grid(x_goal)    # goal state

        self.closed_set = set()    # the set containing the states that have been visited
        self.open_set = set()      # the set containing the states that are condidate for future expension

        self.est_cost_through = {}  # dictionary of the estimated cost from start to goal passing through state (often called f score)
        self.cost_to_arrive = {}    # dictionary of the cost-to-arrive at state from start (often called g score)
        self.came_from = {}         # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_cost_through[self.x_init] = self.distance(self.x_init,self.x_goal)

        self.path = None        # the final path as a list of states

    def is_free(self, x):
        """
        Checks if a give state x is free, meaning it is inside the bounds of the map and
        is not inside any obstacle.
        Inputs:
            x: state tuple
        Output:
            Boolean True/False
        Hint: self.occupancy is a DetOccupancyGrid2D object, take a look at its methods for what might be
              useful here
        """
        ########## Code starts here ##########
        x = np.asarray(x)
        if np.all(x >= self.statespace_lo) and np.all(x <= self.statespace_hi):
            return self.occupancy.is_free(x)
        return False
        # # check if x is within the state space bounds
        # if not (self.statespace_lo[0] <= x[0] <= self.statespace_hi[0] and self.statespace_lo[1] <= x[1] <= self.statespace_hi[1]):
        #     return False
        # check if x is in a free space according to the occupancy grid
        # if not self.occupancy.is_free(x):
        #     return False
        # return True

        # raise NotImplementedError("is_free not implemented")
        ########## Code ends here ##########

    def distance(self, x1, x2):
        """
        Computes the Euclidean distance between two states.
        Inputs:
            x1: First state tuple
            x2: Second state tuple
        Output:
            Float Euclidean distance

        HINT: This should take one line. Tuples can be converted to numpy arrays using np.array().
        """
        ########## Code starts here ##########
        # compute the Euclidean distance between x1 and x2
        return np.linalg.norm(np.array(x1) - np.array(x2))

        # compute the L1 norm (Manhattan distance) between x1 and x2
        # return np.linalg.norm(np.array(x1) - np.array(x2), ord=1)

        # compute the L-infinity norm (Chebyshev distance) between x1 and x2
        # return np.linalg.norm(np.array(x1) - np.array(x2), ord=np.inf)

        # raise NotImplementedError("distance not implemented")
        ########## Code ends here ##########

    def snap_to_grid(self, x):
        """ Returns the closest point on a discrete state grid
        Input:
            x: tuple state
        Output:
            A tuple that represents the closest point to x on the discrete state grid
        """
        return (
            self.resolution * round((x[0] - self.x_offset[0]) / self.resolution) + self.x_offset[0],
            self.resolution * round((x[1] - self.x_offset[1]) / self.resolution) + self.x_offset[1],
        )

    def get_neighbors(self, x):
        """
        Gets the FREE neighbor states of a given state x. Assumes a motion model
        where we can move up, down, left, right, or along the diagonals by an
        amount equal to self.resolution.
        Input:
            x: tuple state
        Ouput:
            List of neighbors that are free, as a list of TUPLES

        HINTS: Use self.is_free to check whether a given state is indeed free.
               Use self.snap_to_grid (see above) to ensure that the neighbors
               you compute are actually on the discrete grid, i.e., if you were
               to compute neighbors by adding/subtracting self.resolution from x,
               numerical errors could creep in over the course of many additions
               and cause grid point equality checks to fail. To remedy this, you
               should make sure that every neighbor is snapped to the grid as it
               is computed.
        """
        neighbors = []
        ########## Code starts here ##########
        # Define the possible movements (8 directions)
        movements = [
            (self.resolution, 0),   # right
            (-self.resolution, 0),  # left
            (0, self.resolution),   # up
            (0, -self.resolution),  # down
            (self.resolution, self.resolution),    # up-right
            (self.resolution, -self.resolution),   # down-right
            (-self.resolution, self.resolution),   # up-left
            (-self.resolution, -self.resolution)   # down-left
        ]
        for move in movements:
            neighbor = self.snap_to_grid((x[0] + move[0], x[1] + move[1]))
            if self.is_free(neighbor):
                neighbors.append(neighbor)
        return neighbors
        # raise NotImplementedError("get_neighbors not implemented")
        ########## Code ends here ##########

    def find_best_est_cost_through(self):
        """
        Gets the state in open_set that has the lowest est_cost_through
        Output: A tuple, the state found in open_set that has the lowest est_cost_through
        """
        return min(self.open_set, key=lambda x: self.est_cost_through[x])

    def reconstruct_path(self):
        """
        Use the came_from map to reconstruct a path from the initial location to
        the goal location
        Output:
            A list of tuples, which is a list of the states that go from start to goal
        """
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))

    def plot_path(self, fig_num=0, show_init_label=True):
        """Plots the path found in self.path and the obstacles"""
        if not self.path:
            return

        self.occupancy.plot(fig_num)

        solution_path = np.asarray(self.path)
        plt.plot(solution_path[:,0],solution_path[:,1], color="green", linewidth=2, label="A* solution path", zorder=10)
        plt.scatter([self.x_init[0], self.x_goal[0]], [self.x_init[1], self.x_goal[1]], color="green", s=30, zorder=10)
        if show_init_label:
            plt.annotate(r"$x_{init}$", np.array(self.x_init) + np.array([.2, .2]), fontsize=16)
        plt.annotate(r"$x_{goal}$", np.array(self.x_goal) + np.array([.2, .2]), fontsize=16)
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)

        plt.axis([0, self.occupancy.width, 0, self.occupancy.height])

    def plot_tree(self, point_size=15):
        # plot_line_segments([(x, self.came_from[x]) for x in self.open_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
        # plot_line_segments([(x, self.came_from[x]) for x in self.closed_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
        # px = [x[0] for x in self.open_set | self.closed_set if x != self.x_init and x != self.x_goal]
        # py = [x[1] for x in self.open_set | self.closed_set if x != self.x_init and x != self.x_goal]
        # plt.scatter(px, py, color="blue", s=point_size, zorder=10, alpha=0.2)
        print("Tree plotting not implemented")

    def solve(self):
        """
        Solves the planning problem using the A* search algorithm. It places
        the solution as a list of tuples (each representing a state) that go
        from self.x_init to self.x_goal inside the variable self.path
        Input:
            None
        Output:
            Boolean, True if a solution from x_init to x_goal was found

        HINTS:  We're representing the open and closed sets using python's built-in
                set() class. This allows easily adding and removing items using
                .add(item) and .remove(item) respectively, as well as checking for
                set membership efficiently using the syntax "if item in set".
        """
        ########## Code starts here ##########
        while self.open_set:
            current = self.find_best_est_cost_through()
            if current == self.x_goal:
                self.path = self.reconstruct_path()
                return True

            self.open_set.remove(current)
            self.closed_set.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closed_set:
                    continue

                tentative_g_score = self.cost_to_arrive[current] + self.distance(current, neighbor)

                if neighbor not in self.open_set:
                    self.open_set.add(neighbor)
                elif tentative_g_score >= self.cost_to_arrive.get(neighbor, float('inf')):
                    continue

                self.came_from[neighbor] = current
                self.cost_to_arrive[neighbor] = tentative_g_score
                self.est_cost_through[neighbor] = tentative_g_score + self.distance(neighbor, self.x_goal)
        # raise NotImplementedError("solve not implemented")
        ########## Code ends here ##########

#----------------------------------------------------------------------------------------------------------
class Navigator(BaseNavigator):
    def __init__(self, kpx: float = 6.0, kpy: float = 6.0, kdx: float = 3.0, kdy: float = 3.0,
                 V_max: float = 0.5, om_max: float = 1.0) -> None:
        # call the parent's init method
        super().__init__()
        # set the proportional control gain
        self.kp = 2.0
        # set the constants for trajectory tracking
        self.kpx = kpx
        self.kpy = kpy
        self.kdx = kdx
        self.kdy = kdy
        self.V_max = V_max
        # self.om_max = om_max
        self.V_PREV_THRES = 0.0001
    
    def reset(self) -> None:
        """ Reset internal variables for trajectory tracking controller """
        self.t_prev = 0.0
        self.V_prev = 0.0
        self.om_prev = 0.0
        
    # override the compute_heading_control() method from BaseHeadingController
    def compute_heading_control(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        # compute the heading error
        heading_error = wrap_angle(goal.theta - state.theta)

        # required angular velocity
        angular_velocity = self.kp * heading_error
        # create a new TurtleBotControl message
        control_msg = TurtleBotControl()
        control_msg.omega = angular_velocity

        return control_msg
    
    def compute_trajectory_tracking_control(self, state: TurtleBotState, plan: TrajectoryPlan, t: float) -> TurtleBotControl:
        """ Compute control target using a trajectory tracking controller

        Args:
            state (TurtleBotState): current robot state
            plan (TrajectoryPlan): planned trajectory
            t (float): current timestep

        Returns:
            TurtleBotControl: control command
        """

        dt = t - self.t_prev
        traj = plan.desired_state(t)

        # I want to calculate the below values with scipy.interpolate.splev to sample from spline parameters given by TrajectoryPlan
        x_d = traj.x
        xd_d = splev(t, plan.path_x_spline, der=1)
        xdd_d = splev(t, plan.path_x_spline, der=2)
        y_d = traj.y
        yd_d = splev(t, plan.path_y_spline, der=1)
        ydd_d = splev(t, plan.path_y_spline, der=2)

        ########## Code starts here ##########
        # avoid singularity
        if abs(self.V_prev) < self.V_PREV_THRES:
            self.V_prev = self.V_PREV_THRES

        xd = self.V_prev*np.cos(state.theta)
        yd = self.V_prev*np.sin(state.theta)

        # compute virtual controls
        u = np.array([xdd_d + self.kpx*(x_d-state.x) + self.kdx*(xd_d-xd),
                      ydd_d + self.kpy*(y_d-state.y) + self.kdy*(yd_d-yd)])

        # compute real controls
        J = np.array([[np.cos(state.theta), -self.V_prev*np.sin(state.theta)],
                          [np.sin(state.theta), self.V_prev*np.cos(state.theta)]])
        a, om = linalg.solve(J, u)
        V = self.V_prev + a*dt
        ########## Code ends here ##########

        # apply control limits (NOTE NOT USED HERE)
        # V = np.clip(V, -self.V_max, self.V_max)
        # om = np.clip(om, -self.om_max, self.om_max)

        # save the commands that were applied and the time
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        control = TurtleBotControl()
        control.v = V
        control.omega = om

        return control

    def compute_trajectory_plan(self, state: TurtleBotState, goal: TurtleBotState, occupancy: StochOccupancyGrid2D, resolution: float, horizon: float) -> T.Optional[TrajectoryPlan]:
        """ Compute a trajectory plan using A* and cubic spline fitting
        
        Args:
            state (TurtleBotState): state
            goal (TurtleBotState): goal
            occupancy (StochOccupancyGrid2D): occupancy
            resolution (float): resolution
            horizon (float): horizon

        Returns:
            T.Optional[TrajectoryPlan]:
        """
        
        x_init = (state.x, state.y)
        x_goal = (goal.x, goal.y)

        astar = AStar(statespace_lo=(state.x-horizon,state.y-horizon), statespace_hi=(state.x+horizon,state.y+horizon), x_init=x_init, x_goal=x_goal, occupancy=occupancy, resolution=resolution)
        if not astar.solve() or len(astar.path) < 4:
            print("No valid path found")
            return None
        self.reset()
        path = np.array(astar.path)

        # path time 
        v_desired = 0.15
        spline_alpha = 0.05
        ts = [0.0]
        path = np.array(path)
        for i in range(len(path)-1):
            dist = linalg.norm(path[i+1]-path[i])
            dt = dist / v_desired
            ts.append(ts[-1] + dt)
        ts = np.array(ts)

        x_path = splrep(ts, path[:,0], k=3, s=spline_alpha)
        y_path = splrep(ts, path[:,1], k=3, s=spline_alpha)

        # code from Cole
        plan = TrajectoryPlan(
            path = path,
            path_x_spline=x_path,
            path_y_spline=y_path,
            duration = ts[-1]
        )
        
        return plan

if __name__ == "__main__":
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    rclpy.shutdown()
