#System library imports
import time
import sys
import os
import numpy
import subprocess
from subprocess import *
from acclHandler import *
from multiprocessing import Process
from threading import Thread, Lock

# Local library imports
import edison_servo_pwm
import map_channel_input
import receive_from_nodejs

from attitude_rate_controller import AttitudeRateController

numMotors = 4
control_in_rate = True


motors = [edison_servo_pwm.EdisonServoPWM(ordinal_channel=i, pwm_lower_bound_us=1000,
                                   pwm_upper_bound_us=2000, frequency=400) for i in range(numMotors)]

mixer_matrix = np.array([[0, 42, 42, 127],
    [-63,  42, -42, 127],
    [-63, -42,  42, 127],
    [  0, -42, -42, 127],
    [ 64, -42,  42, 127],
    [ 64,  42, -42, 127]]) / 127.0

gyro_refresh_rate = 500.0 # The speed at which we are reading the gyrometer, in [Hz]

delT = 1.0/gyro_refresh_rate

attitude_rate_controller = AttitudeRateController()

setpoint_rpy_R = np.array([0,0,0])

desired_response = np.array([0,0,0,0])
DEG2RAD = pi/180

if 0:
    # Start thread to monitor accelerometer data
    accelHandlerProcess=Process(target=handleAccelerometerSensorData, args=mutex)
    accelHandlerProcess.start()
    accelHandlerProcess.join()

# Open the client to listen to Node.js
client = serve_socket(8383)

while True:

    try:
        # Read sensor data. All floats
        # Gyro: Angle of velocity along x, y & z axes. 3 values
        gyroSensorData = None
        while not gyroSensorData:
            gyroSensorData = receiveSensorData()

        # Accelerometer: Reading along the x, y & z axes of accelerometer data.  3 values
        # TBD: Wait till all data received
        accelSensorData = None

        if control_in_rate == True:
            omega_hat = gyroSensorData

            attitude_rate_controller.update_setpoint(setpoint_rpy_R)

            # The PID values should be tuned so that this stays well within the [-1,1] range for the motor outputs
            moment_xyz = attitude_rate_controller.get_control(omega_hat, delT)

            desired_response[0] = moment_xyz[0]
            desired_response[1] = moment_xyz[1]
            desired_response[2] = moment_xyz[2]

            # Get the thrust directly
            desired_response[3] = setpoint_rpyt_D[3]

        else:
            #==========================================================
            # This is where the attitude estimator will go. But for the moment, the estimator is not ready, so go with
            # rate mode.
            #==========================================================
            attitude_hat_R = 0 #functionFromJacob(???)

            # Send updated setpoint to control
            hexacontroller.update_setpoint(attitude_hat_R, omega_hat_RPS, setpoint_rp_R, setpoint_yaw_RPS, delT)

            # Poll dynamics controller
            requested_dynamics = hexacontroller.get_control(np.array([roll_R, pitch_R, yaw_R]), omega_body, Rot_body_from_ned, delT)

        # Calculate the normalized thrusts required to achieve the desired dynamics response
        thrusts = np.dot(mixer_matrix, desired_response)

        # Make sure that any negative thrusts are driven all the way to -1. This is for safety
        for i in range(numMotors):
            if thrusts[i] <= 0:
                thrusts[i] = -1

        # Map actuator values to motor output PWM
        for i in range(numMotors):
            pwm_command = map_channel_to_output(input_value=thrusts[i], min=1000, neutral=1100, max=2000)
            motors[i].output(pwm_command)

        #====================================================
        # Update setpoint from remote control. Do this last because it ensures the loop connection is the fastest
        # possible between the gyros and the motors
        if 1:
            setpoint_rpyt_D = np.array(0, 0, 0, .5)
        else:
            setpoint_rpyt_D = receive_from_nodejs.receiveFloats(client, 4)

        # Convert setpoint deg into radians, and scale
        if control_in_rate == True:
            setpoint_rpy_R = np.array(setpoint_rpyt_D[0], setpoint_rpyt_D[1], setpoint_rpyt_D[2]) * DEG2RAD * 150
        else:
            setpoint_rpy_R = np.array(setpoint_rpyt_D[0], setpoint_rpyt_D[1], setpoint_rpyt_D[2]) * DEG2RAD * 30
        #====================================================

    except Exception:
        # End safely by setting all motors to 0 thrust
        for i in range(numMotors):
            motors[i].output(1000)

        # Inform and exit on exception
        exit(0)