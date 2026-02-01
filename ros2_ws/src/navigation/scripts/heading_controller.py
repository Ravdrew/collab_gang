#!/usr/bin/env python3
# Turtlebot heading control

import numpy as np
import rclpy
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

class HeadingController(BaseHeadingController):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        # self.kp = 2.0
        self.declare_parameter("kp", 2.0)
        
    @property
    def kp(self) -> float:
        return self.get_parameter("kp").value
        
    def compute_control_with_goal(self, current_state: TurtleBotState, desired_state: TurtleBotState) -> TurtleBotControl:
        heading_error = wrap_angle(desired_state.theta - current_state.theta)
        w = self.kp * heading_error
        return TurtleBotControl(omega=w)

if __name__=="__main__":
    rclpy.init()
    HC = HeadingController("HeadingController")
    rclpy.spin(HC)
    rclpy.shutdown()
