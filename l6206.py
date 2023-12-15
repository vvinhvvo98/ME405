"""!
@file l6206.py
This file implement a class that control DC motor using PWM signals.

@author Vinh Vo
@author Quinn Stephens
@date   2023-Dec-11 
"""

from pyb import Pin, Timer

class L6206:
    """!
    A class for controlling L6206 motor driver using PWM signals.
    """
    """
    Attributes:
        SLP (Pin): Sleep pin to enable or disable the motor driver.
        DIR (Pin): Direction pin to set the rotation direction of the motor.
        PWM (TimerChannel): PWM channel to control the motor speed.

    Methods:
        __init__(self, tim, channel, SLP, DIR, PWM): Initializes the L6206 instance.
        set_duty(self, duty): Sets the duty cycle for the motor speed and direction.
        enable(self): Enables the motor driver.
    """


    def __init__(self, tim, channel, SLP, DIR, PWM):
        """!
        Initializes a new instance of the L6206 motor driver class.

        @param tim (Timer): The timer object for PWM generation.
        @param channel (int): The channel number on the timer for PWM.
        @param SLP (str): The pin name for the sleep control.
        @param DIR (str): The pin name for the direction control.
        @param PWM (str): The pin name for PWM signal output.
        """
        
        """!
        Example:
          @code
              '''! This code sample is used to drive 2 DC motors from ROMI (L) and (R)'''
              
              # Initiate 2 DC motor with appropriate timer
              tim_R = Timer(4, freq=20000)
              tim_L = Timer(4, freq=20000)
              mot_R = L6206(tim_R, 2, Pin.cpu.A2, Pin.cpu.A10, Pin.cpu.B7)
              mot_L = L6206(tim_L, 1, Pin.cpu.B3, Pin.cpu.B10, Pin.cpu.B6)
              
              # Choose max speed for both motor (100% cycle duty)
              duty_L = 100
              duty_R = 100
              
              # Run 2 motors
              mot_L.set_duty(duty_L)
              mot_R.set_duty(duty_R)
                  
          @endcode
        """
        
        ## MCU sleep pin
        self.SLP = Pin(SLP, mode=Pin.OUT_PP)
        
        ## MCU direction pin
        self.DIR = Pin(DIR, mode=Pin.OUT_PP)
        
        ## MCU PWM timer channel pin
        self.PWM = tim.channel(channel, pin=PWM, mode=Timer.PWM, pulse_width_percent=0)
        
        self.SLP.low()

    def set_duty(self, duty):
        """!
        Sets the duty cycle for the motor. Positive values for one direction,
        negative for the other, and zero to stop.

        @param duty (int): The duty cycle percentage for motor speed and direction.
                        Positive for forward, negative for backward.
        """
        if duty > 0:
            self.DIR.low()
            self.PWM.pulse_width_percent(duty)
        elif duty < 0:
            self.DIR.high()
            self.PWM.pulse_width_percent((-1) * duty)
        else:
            self.PWM.pulse_width_percent(0)

    def enable(self):
        """!
        Enables the motor driver by setting the sleep pin high.
        """
        self.SLP.high()
