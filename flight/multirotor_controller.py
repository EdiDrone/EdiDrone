'''


    Multirotor controller. This is appropriate for
     a multirotor modeled as a rigid body with inertia.

'''
__author__ = 'kenz'

# System libraries
from math import pi
import numpy as np
from collections import namedtuple

# Local libraries
from attitude_rate_controller import AttitudeRateController


class MultirotorController(object):
    def __init__(self, controller_type = 'PID'):

        self.DEG2RAD = pi / 180
        self.RAD2DEG = pi / 180

        # Declare all class variables
        self.setpoint_roll_R  = 0
        self.pitch_setpoint_D = 0
        self.setpoint_yaw_RPS = 0
        self.thrust_normalized = 0

        self.accum_attitude         = np.array([0,0,0])
        self.accum_angular_velocity = np.array([0,0,0])

        # Create PID
        PIDStruct = namedtuple("PIDStruct", "Kp Ki Kd")

        # Outer loop
        self.roll_outer_loop_PID  = PIDStruct(Kp = 5, Ki = 0, Kd = 0)
        self.pitch_outer_loop_PID = PIDStruct(Kp = 5, Ki = 0, Kd = 0)
        self.yaw_outer_loop_PID   = PIDStruct(Kp = 1, Ki = 0, Kd = 0)

        self.x_outer_loop_PID = PIDStruct(Kp = 0.3, Ki = 0, Kd = 0)
        self.y_outer_loop_PID = PIDStruct(Kp = 0.3, Ki = 0, Kd = 0)
        self.z_outer_loop_PID = PIDStruct(Kp = 0.5, Ki = 0, Kd = 0)

        # Inner loop
        self.attitude_rate_controller = AttitudeRateController('PID')
        self.attitude_rate_controller.omega_x_inner_loop_PID = PIDStruct(Kp = 0.2, Ki = 0.1, Kd = 0.001)
        self.attitude_rate_controller.omega_y_inner_loop_PID = PIDStruct(Kp = 0.2, Ki = 0.1, Kd = 0.001)
        self.attitude_rate_controller.omega_z_inner_loop_PID = PIDStruct(Kp = 0.2, Ki = 0.1, Kd = 0.001)

        self.vx_inner_loop_PID = PIDStruct(Kp = 0.08, Ki = 0.008, Kd = 0.001)
        self.vy_inner_loop_PID = PIDStruct(Kp = 0.08, Ki = 0.008, Kd = 0.001)
        self.vz_inner_loop_PID = PIDStruct(Kp = 0.80, Ki = 0.100, Kd = 0.001)

        # Reset the controller, e.g. integrators, variance matrices, etc...
        self.reset_controller()


    def reset_controller(self):
        self.accum_attitude         = np.array([0,0,0])
        self.accum_angular_velocity = np.array([0,0,0])

    def update_setpoint(self, roll_setpoint_D, pitch_setpoint_D, yaw_setpoint_DPS, thrust_setpoint):
        self.setpoint_roll_R  = roll_setpoint_D * self.DEG2RAD
        self.pitch_setpoint_D = pitch_setpoint_D * self.DEG2RAD
        self.setpoint_yaw_RPS = yaw_setpoint_DPS * self.DEG2RAD
        self.thrust_normalized = thrust_setpoint

    def get_control(self, attitude_hat_R, omega_hat_RPS, delT):
        # Roll-pitch attitude error
        roll_error_R  = self.setpoint_roll_R  - attitude_hat_R[0]
        pitch_error_R = self.setpoint_pitch_R - attitude_hat_R[1]

        # Integrate outer loop
        self.accum_attitude = self.accum_attitude + np.array([roll_error_R, pitch_error_R, yaw_error_R]) * \
                                    np.array([self.roll_outer_loop_PID.Ki, self.pitch_outer_loop_PID.Ki, self.yaw_outer_loop_PID.Ki]) * delT

        # Apply some kind of integrator anti-windup here
        # .
        # .
        # .

        # Inner loop setpoints
        # NOTE: THIS IS NOT RIGHT. The derivative of attitude is not angular rate. There is a skew-symmetric matrix that needs to be
        # taken into account, but isn't here (because I don't expect it to matter at the low absolute angles at which we will work
        omega_x_setpoint = self.accum_attitude[0] + (roll_error_R  * self.roll_outer_loop_PID.Kp)  #+ (roll_dot_error  * self.roll_outer_loop_PID.Kd)
        omega_y_setpoint = self.accum_attitude[1] + (pitch_error_R * self.pitch_outer_loop_PID.Kp) #+ (pitch_dot_error * self.pitch_outer_loop_PID.Kd)
        omega_z_setpoint = self.setpoint_yaw_RPS

        # Inner loop ===============================================================================================
        # Send setpoints to inner-loop attitude rate controller
        self.attitude_rate_controller.update_setpoint(np.array([omega_x_setpoint, omega_y_setpoint, omega_z_setpoint]))

        # Rate inner loop
        moment_xyz = self.attitude_rate_controller.get_control(omega_hat_RPS, delT)

        # print "Attitude setpoint: " + repr(self.setpoint_roll_R)  + ", " + repr(self.setpoint_pitch_R)  + ", " + repr(self.setpoint_yaw_RPS)
        # print "Attitude actual  : " + repr(attitude_hat_R[0])  + ", " + repr(attitude_hat_R[1])  + ", " + repr(attitude_hat_R[2])
        # print "Omega setpoint:    " + repr(omega_x_setpoint)  + ", " + repr(omega_y_setpoint)  + ", " + repr(omega_z_setpoint)
        # print "Omega actual:      " + repr(omega_hat_RPS[0])  + ", " + repr(omega_hat_RPS[1])  + ", " + repr(omega_hat_RPS[2])

        # print "Position setpoint (NED): " + repr(self.setpoint_x)  + ", " + repr(self.setpoint_y)  + ", " + repr(self.setpoint_z)
        # print "Position actual (NED)  : " + repr(position_hat[0])  + ", " + repr(position_hat[1])  + ", " + repr(position_hat[2])
        # print "Velocity setpoint (NED): " + repr(vx_setpoint)  + ", " + repr(vy_setpoint)  + ", " + repr(vz_setpoint)
        # print "Velocity actual (NED)  : " + repr(velocity_hat[0])  + ", " + repr(velocity_hat[1])  + ", " + repr(velocity_hat[2])
        # print "Force desired (NED)    : " + repr(force_x)  + ", " + repr(force_y)  + ", " + repr(force_z)
        # print ""

        # Form control vector. It is torques about x-, y- and z- axes, and forces on x-, y- and z-axes
        u = np.array([moment_xyz[0], moment_xyz[1], moment_xyz[2], self.thrust_normalized])

        return u
