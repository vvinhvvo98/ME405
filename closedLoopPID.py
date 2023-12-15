"""!
@file closedLoopPID.py
This file contains classes to run a multipurpose closed loop PID controller.

@author Vinh Vo
@author Quinn Stephens
@date   2023-Dec-11 
"""

class PIDController:
    """!
    Implements a template for multipurpose PID controller. 
    """
    """
    Attributes:
        KP (float): Proportional gain
        KI (float): Integral gain
        KD (float): Derivative gain
        integral (float): Sum of integral error
        prev_error (float): Previous error


    Methods:
        __init__(self, tim, ch1, ch2, chA, chB): Initializes the Encoder with specified timer and channel settings.
        update(self): Updates the encoder position.
        get_position(self): Returns the current encoder position.
        get_delta(self): Returns the change in encoder position.
        get_rad_s(self): Calculates and returns the angular velocity in radians per second.
        zero(self): Resets the encoder position.
    """
    

    def __init__(self, kp, ki, kd):
        """!
        Initialize a PID controller object for multipurpose.
        
        This class implement a PID controller with user-define KP KI KD value.
        
        @param kp (float): Proportional gain KP for the controller.
        @param ki (float): Integral gain KI for the controller.
        @param kd (float): Derivative gain KD for the controller.
        """
        
        """!
        Example:
          @code
              '''! This code sample is for using PID controller for a DC motor object'''
              # Assign KP KI KD
              KP = 1
              KI = 0.1
              KD = 0.1
              
              # Create and PID object
              PID_controller = PIDController(KP, KI, KD)
              
              # Set reference speed
              V_set  = 10                           # rad/s
            
              # Read actual speed from encoder
              V_meas = enc.get_rad_s()              # rad/s      
            
              # Run PID Controller
              Output = PID_controller.update(V_set,V_meas)
            
              # Assume [Motor] is a dc motor object
              Motor.set_duty(Output)       
          @endcode
          
          """
        
        ## Proportional gain
        self.kp = kp  
        
        ## Integral gain                                                          
        self.ki = ki  
        
        ## Derivative gain   
        self.kd = kd 
        
        ## Sum of integral error
        self.integral = 0

        ## Previous error                                                      
        self.prev_error = 0                                                     

    def update(self, setpoint, measured_value):
        """!
        This method update the output of the controller everytime base on setpoint and measured value.
        
        @param setpoint (float): Target setpoint of the controller.
        @param measured_value(float): Actual value measured from the sensor.
        
        @return (float): Output value base on sepoint value and measured value
        """
        
        error = setpoint - measured_value                                       
        self.integral += error                                                  
        derivative = error - self.prev_error                                    
        out = self.kp * error + self.ki * self.integral + self.kd * derivative  
        self.prev_error = error                                                
        return out

