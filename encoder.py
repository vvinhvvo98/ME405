"""!
@file encoder.py
This file contains classes to run and read rotary encoder using MCU timer.

@author Vinh Vo
@author Quinn Stephens
@date   2023-Dec-11 
"""

from pyb import Timer
import time

class Encoder:
    """!
    A class to interface with an rotary encoder using a microcontroller's timer.
    """
    """
    Attributes:
        tim (Timer): The timer object used for the encoder.
        chA (TimerChannel): Timer channel A for the encoder.
        chB (TimerChannel): Timer channel B for the encoder.
        old (int): Previous encoder position.
        new (int): Current encoder position.
        delta (int): Change in encoder position.
        position (int): Total encoder position.
        Max (int): Maximum encoder periodic value that will be reset .
        halfMax (int): Half of the maximum encoder periodic value that will be reset .
        neg_halfMax (int): Negative half of the maximum encoder periodic value that will be reset.
        t_old (int): Previous time in milliseconds.

    Methods:
        __init__(self, tim, ch1, ch2, chA, chB): Initializes the Encoder with specified timer and channel settings.
        update(self): Updates the encoder position.
        get_position(self): Returns the current encoder position.
        get_delta(self): Returns the change in encoder position.
        get_rad_s(self): Calculates and returns the angular velocity in radians per second.
        zero(self): Resets the encoder position.
    """


    def __init__(self, tim, ch1, ch2, chA, chB):
        """!
        Initializes the Encoder instance with the specified timer and channel settings.

        @param tim (Timer): The timer object used for the encoder.
        @param ch1 (int): The first timer channel number.
        @param ch2 (int): The second timer channel number.
        @param chA (Pin): The pin associated with timer channel A.
        @param chB (Pin): The pin associated with timer channel B.
        
        """
        
        """!
        Example:
          @code
              '''! This code sample is used to read and print the speed of 2 encoders from ROMI (L) and (R)'''
              
              # Initiate 2 encoders with appropriate timer
              tim_Re = Timer(1, period=5000, prescaler=0)
              tim_Le = Timer(2, period=5000, prescaler=0)
              enc_L = Encoder (tim_Le, 1, 2, Pin.cpu.A0, Pin.cpu.A1)
              enc_R = Encoder (tim_Re, 1, 2, Pin.cpu.A8, Pin.cpu.A9)
              
              # Enable before run
              mot_R.enable()
              mot_L.enable()
              
              # Update both encoder before reading
              enc_R.update()
              enc_L.update()
            
              # Read and print the speed 
              wR_meas = enc_R.get_rad_s()
              wL_meas = enc_R.get_rad_s()
              print(f"Right Wheel Speed: {wR_meas} [rad/s]")
              print(f"Left Wheel Speed: {wL_meas} [rad/s]")
          @endcode
          

        """
        ## MCU timer
        self.tim = tim
        
        ## Encoder channel A
        self.chA = tim.channel(ch1, pin=chA, mode=Timer.ENC_AB)
        
        ## Encoder channel B
        self.chB = tim.channel(ch2, pin=chB, mode=Timer.ENC_AB)
        
        ## Encoder old value
        self.old = 0
        
        ## Encoder new value
        self.new = 0
        
        ## Encoder counter from the last time use update() function
        self.delta = 0
        
        ## Encoder position
        self.position = 0
        
        ## Encoder maximum counter if not overwrite
        self.Max = 5000
        
        ## Half maximun counter
        self.halfMax = self.Max / 2
        
        ## Negative haft maxnimum counter
        self.neg_halfMax = -self.halfMax
        
        ## Previous time stamp
        self.t_old = time.ticks_ms()

    def update(self):
        """!
        Updates the encoder's position. This should be called periodically.
        
        """
        self.new = self.tim.counter()
        self.delta = self.new - self.old
        if self.delta > self.halfMax:
            self.delta -= (self.Max + 1)
        elif self.delta < self.neg_halfMax:
            self.delta += (self.Max + 1)
        self.old = self.new
        self.position += self.delta

    def get_position(self):
        """!
        Returns the current position of the encoder.
        
        @return (int): The current encoder position.
        """
        return self.position

    def get_delta(self):
        """!
        Returns the change in position since the last update.
        
        @return (int): The change in encoder position.
        """
        return self.delta

    def get_rad_s(self):
        """!
        Calculates and returns the angular velocity in radians per second.
        
        @return (float): Angular velocity in radians per second.
        """
        t_current = time.ticks_ms()
        dt = float(time.ticks_diff(t_current, self.t_old))
        if dt == 0: return 0
        v = (self.get_delta()/1440*(2*3.1416))/(dt/1000)
        self.t_old = t_current

        return v

    def zero(self):
        """!
        Resets the encoder's position to zero.
        """
        self.position = 0
        self.old = 0
