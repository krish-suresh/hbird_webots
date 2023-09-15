import numpy as np
from math import sin, cos
from .utils import HBParams, angle_wrap
from hbird_msgs.msg._waypoint import Waypoint
from hbird_msgs.msg._state import State
class Controller3D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, hbparams, pid_gains, dt=0):
        """
        Inputs:
        - hbparams (HBarams dataclass):             model parameter class for the drone
        - pid_gains (dict):                         pid gains

        N.B. pid_gains is a dictionary structure where the keys are 'kp_x', 'kd_z', etc.
        """
        self.params : HBParams = hbparams

        # set control gains here
        self.pid_gains = pid_gains


    def compute_commands(self, setpoint : Waypoint, state : State):
        """
        Inputs:
        - setpoint (Waypoint):      the desired control setpoint
        - state (State):            the current state of the system
        Returns:
        - U (np.array):     array of control inputs {u1-u4}

        """
        U = np.array([0.,0.,0.,0.])
        # your code here

        U[0] = self.params.mass * (self.pid_gains["kd_z"]*(-state.velocity.z) + self.pid_gains["kp_z"]*(setpoint.position.z - state.position.z) + self.params.g)
        ẍ_c = self.pid_gains["kd_x"]*(-state.velocity.x) + self.pid_gains["kp_x"]*(setpoint.position.x - state.position.x)
        ÿ_c = self.pid_gains["kd_y"]*(-state.velocity.y) + self.pid_gains["kp_y"]*(setpoint.position.y - state.position.y)
        phi_d = (1.0/self.params.g)*(ẍ_c*sin(setpoint.heading) - ÿ_c*cos(setpoint.heading))
        theta_d = (1.0/self.params.g)*(ẍ_c*cos(setpoint.heading) + ÿ_c*sin(setpoint.heading))
        psi_d = setpoint.heading
        U[1] =  self.pid_gains["kd_p"]*(-state.angular_velocity.x) + self.pid_gains["kp_phi"]*(phi_d - state.orientation.x)
        U[2] =  self.pid_gains["kd_q"]*(-state.angular_velocity.y) + self.pid_gains["kp_theta"]*(theta_d - state.orientation.y)
        U[3] =  self.pid_gains["kd_r"]*(-state.angular_velocity.z) + self.pid_gains["kp_psi"]*(angle_wrap(psi_d - state.orientation.z))
        print (f"Orientation: {state.orientation.x:.2f},{state.orientation.y:.2f},{state.orientation.z:.2f}")
        print (f"Position: {state.position.x:.2f},{state.position.y:.2f},{state.position.z:.2f}. U: {['{0:0.2f}'.format(i) for i in U]}")
        return U