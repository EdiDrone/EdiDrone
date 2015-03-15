__author__ = 'kenz'

'''


    Multirotor controller. This is appropriate for
     a multirotor modeled as a rigid body with inertia.


'''
__author__ = 'kenz'

from math import pi
import numpy as np
from geometry import rotate
from collections import namedtuple
from attitude_rate_controller import AttitudeRateController


class MultirotorController(object):
    def __init__(self):
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
        self.vz_inner_loop_PID = PIDStruct(Kp = 0.8, Ki = 0.1, Kd = 0.001)

        self.setpoint_yaw_R = 0

        self.setpoint_x = 0
        self.setpoint_y = 0
        self.setpoint_z = 0

        self.feed_forward_D = 0

        # Reset the controller, e.g. integrators, variance matrices, etc...
        self.reset_controller()


    def reset_controller(self):
        self.accum_attitude         = np.array([0,0,0])
        self.accum_angular_velocity = np.array([0,0,0])
        self.accum_position         = np.array([0,0,0])
        self.accum_velocity         = np.array([0,0,0])

    def set_feed_forward_Z_force(self, mass, gravity):
        self.feed_forward_D = -mass * gravity

    def update_setpoint(self, yaw_setpoint_D, position_setpoint):
        self.setpoint_yaw_R   = (((yaw_setpoint_D + 180) % 360) - 180) * pi/180

        self.setpoint_x = position_setpoint[0]
        self.setpoint_y = position_setpoint[1]
        self.setpoint_z = position_setpoint[2]


    def get_control(self, attitude_hat_R, omega_hat, position_hat, velocity_hat, Rot_body_from_ned, delT):
        # Outer loop ===============================================================================================
        # Position error
        x_error = self.setpoint_x - position_hat[0]
        y_error = self.setpoint_y - position_hat[1]
        z_error = self.setpoint_z - position_hat[2]

        # Integrate outer loop
        self.accum_position = self.accum_position + np.array([x_error, y_error, z_error]) * \
                                    np.array([self.x_outer_loop_PID.Ki, self.y_outer_loop_PID.Ki, self.z_outer_loop_PID.Ki]) * delT

        # Apply some kind of integrator anti-windup here
        # .
        # .
        # .


        # Outer-middle loop setpoints
        vx_setpoint = self.accum_position[0] + (x_error * self.x_outer_loop_PID.Kp) #+ (x_dot_error * self.x_outer_loop_PID.Kd)
        vy_setpoint = self.accum_position[1] + (y_error * self.y_outer_loop_PID.Kp) #+ (y_dot_error * self.y_outer_loop_PID.Kd)
        vz_setpoint = self.accum_position[2] + (z_error * self.z_outer_loop_PID.Kp) #+ (z_dot_error * self.z_outer_loop_PID.Kd)

        # Middle loop ==============================================================================================
        # Velocity Middle loop errors
        vx_error = vx_setpoint - velocity_hat[0]
        vy_error = vy_setpoint - velocity_hat[1]
        vz_error = vz_setpoint - velocity_hat[2]

        print "position error   : " + str(x_error) + ", " + str(y_error) + ", " + str(z_error)
        print "velocity setpoint: " + str(vx_setpoint) + ", " + str(vy_setpoint) + ", " + str(vz_setpoint)
        print "velocity actual  : " + str(velocity_hat[0]) + ", " + str(velocity_hat[1]) + ", " + str(velocity_hat[2])
        print "velocity error   : " + str(vx_error) + ", " + str(vy_error) + ", " + str(vz_error)


        self.accum_velocity = self.accum_velocity + np.array([vx_error, vy_error, vz_error]) * \
                                np.array([self.vx_inner_loop_PID.Ki, self.vy_inner_loop_PID.Ki, self.vz_inner_loop_PID.Ki]) * delT

        # Apply some kind of integrator anti-windup here
        # .
        # .
        # .

        # Inner-middle loop setpoints
        setpoint_pitch_R =  self.accum_velocity[0] + (vx_error  * self.vx_inner_loop_PID.Kp) # + (vx_dot_error  * self.vx_inner_loop_PID.Kd)
        setpoint_roll_R =  self.accum_velocity[1] + (vy_error  * self.vy_inner_loop_PID.Kp) # + (vy_dot_error  * self.vy_inner_loop_PID.Kd)
        force_z =  self.accum_velocity[2] + (vz_error  * self.vz_inner_loop_PID.Kp) # + (vz_dot_error  * self.vz_inner_loop_PID.Kd)

        setpoint_pitch_R = -setpoint_pitch_R

        # Inner-middle loop ========================================================================================

        # Attitude error
        roll_error_R  = setpoint_roll_R - attitude_hat_R[0]
        pitch_error_R = setpoint_pitch_R - attitude_hat_R[1]
        yaw_error_R   = ((self.setpoint_yaw_R - attitude_hat_R[2] + pi) % (2*pi)) - pi # This is really outer loop stuff, but it looks nicer here
        # print "yaw error: " + str(yaw_error_R * )


        # Integrate middle loop
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
        omega_z_setpoint = self.accum_attitude[2] + (yaw_error_R   * self.yaw_outer_loop_PID.Kp)   #+ (yaw_dot_error   * self.yaw_outer_loop_PID.Kd)


        # Inner loop ===============================================================================================
        # Send setpoints to inner-loop attitude rate controller
        self.attitude_rate_controller.update_setpoint(np.array([omega_x_setpoint, omega_y_setpoint, omega_z_setpoint]))
        # Rate inner loop
        moment_xyz = self.attitude_rate_controller.get_control(omega_hat, delT)

        # Choose between adjusting throttle for tilt angle
        if 1:
            # Add the feed-forward thrust required to keep the vehicle hovering at a constant altitude
            thrust_z = force_z + self.feed_forward_D

            # Rotate NED forces into body frame
            v_body = np.dot(Rot_body_from_ned, np.array([0, 0, 1]))

            # Form control vector. It is torques about x-, y- and z- axes, and forces on x-, y- and z-axes
            u = np.array([moment_xyz[0], moment_xyz[1], moment_xyz[2], thrust_z/v_body[2]])
        else:
            # Form control vector. It is moments about x-, y- and z- axes, and force on the z-axis
            u = np.array([moment_xyz[0], moment_xyz[1], moment_xyz[2], force_z + self.feed_forward_D])

        return u
