'''
Create a servo class which configures an output servo channel by the minimum and maximum pulse width, as well as
frequency.
'''

__author__ = 'kenz'

import mraa # Intel support for IoT, https://github.com/intel-iot-devkit/mraa

class EdisonServoPWM(object):

    def __init__(self, ordinal_channel, pwm_lower_bound_us=1000, pwm_upper_bound_us=2000, frequency=50):

        # Map ordinal channels to Edison output pins. NOTE: This is a problematic mapping because Edison has shields,
        # some of which have jumpers allowing the reordering of the PWMs on the hardware board. Therefore, this is, at
        # best, a guess of how the user has set up the board. It supposes that the Edison is connected to the "Intel
        # Edison kit for Arduino", and that the original jumpers have been left on, in their original positions. Thus,
        # this connects GP12_PWM0 to pin3, GP13_PWM1 to pin4, GP182_PWM2 to pin4, GP183_PWM3 to pin4
        if ordinal_channel == 0:
            self.channel = 3
        elif ordinal_channel == 1:
            self.channel = 4
        elif ordinal_channel == 2:
            self.channel = 7
        elif ordinal_channel == 3:
            self.channel = 8
        else:
            return None

        self.pwm_lower_bound_us = pwm_lower_bound_us
        self.pwm_upper_bound_us = pwm_upper_bound_us
        self.frequency = frequency

        # Calculate PWM period as a function of servo frequency
        self.period_us = int(round(1e6/self.frequency))

        # Check that the input period is physically possible. If not, return
        if self.period_us > self.pwm_upper_bound_us:
            return None

        # Set channel
        self.x = mraa.Pwm(self.channel)

        # Set period
        self.x.period_us(self.period_us)

        # Activate PWM output
        self.x.enable(True)

    '''
    Convert a desired pulse_width into a duty cycle at the preset frequency
    '''
    def output(self, pulse_width_us):
        if pulse_width_us > self.pwm_upper_bound_us:
            pulse_width_us = self.pwm_upper_bound_us
        if pulse_width_us < self.pwm_lower_bound_us:
            pulse_width_us = self.pwm_lower_bound_us

        # Output the signal as the converse of the duty cycle. This is because the servo signal is pulled low during
        # the transmission
        self.x.write(1 - pulse_width_us/self.period_us)

