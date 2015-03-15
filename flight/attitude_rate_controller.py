'''


    Three degree of freedom attitude-rate controller. This is appropriate for
     a multirotor modeled as a rigid body with inertia.


'''
__author__ = 'kenz'

from math import pi
import numpy as np
from collections import namedtuple


class AttitudeRateController(object):
    def __init__(self):
        PIDStruct = namedtuple("PIDStruct", "Kp Ki Kd")

        # Outer loop
        self.omega_x_PID = PIDStruct(Kp = 0.2, Ki = 0.1, Kd = 0.001)
        self.omega_y_PID = PIDStruct(Kp = 0.2, Ki = 0.1, Kd = 0.001)
        self.omega_z_PID = PIDStruct(Kp = 0.2, Ki = 0.1, Kd = 0.001)

        # Reset the controller, e.g. integrators, variance matrices, etc...
        self.reset_controller()


    def reset_controller(self):
        self.accum_angular_velocity = np.array([0,0,0])

    def update_setpoint(self, omega_setpoint_R):
        self.omega_setpoint_R = omega_setpoint_R


    def get_control(self, omega_hat, delT):

        # Calculate errors
        omega_x_error = self.omega_setpoint_R[0] - omega_hat[0]
        omega_y_error = self.omega_setpoint_R[1] - omega_hat[1]
        omega_z_error = self.omega_setpoint_R[2] - omega_hat[2]

        # Integrate inner loop
        self.accum_angular_velocity = self.accum_angular_velocity  + np.array([omega_x_error, omega_y_error, omega_z_error]) * \
                                        np.array([self.omega_x_PID.Ki, self.omega_y_PID.Ki, self.omega_z_PID.Ki]) * delT

        # Apply some kind of integrator anti-windup here
        # .
        # .
        # .

        torque_x = self.accum_angular_velocity[0] + (omega_x_error  * self.omega_x_PID.Kp) # + (omega_x_dot_error  * self.omega_x_PID.Kd)
        torque_y = self.accum_angular_velocity[1] + (omega_y_error  * self.omega_y_PID.Kp) # + (omega_y_dot_error  * self.omega_y_PID.Kd)
        torque_z = self.accum_angular_velocity[2] + (omega_z_error  * self.omega_z_PID.Kp) # + (omega_z_dot_error  * self.omega_z_PID.Kd)

        # Form control vector. It is torques about x-, y- and z- axes, and forces on x-, y- and z-axes
        u = np.array([torque_x, torque_y, torque_z])

        return u
