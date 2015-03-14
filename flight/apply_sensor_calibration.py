'''
Apply simple bias and scale corrections to sensor input
'''
def apply_gyrometer_calibration(input_value, bias, scale):
    output_value = bias * input_value - scale

def apply_accelerometer_calibration(input_value, bias, scale):
    output_value = bias * input_value - scale
