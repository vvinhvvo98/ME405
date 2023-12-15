'''!
@file task_SER.py
This file contain a task that run 2 servos according to the FSM

@author Vinh Vo 
@author Quinn Stephens
@date 2023-Dec-11 
'''

from pyb import Pin, Timer

class ServoTask:
    """!
    A class for controlling servo motors based on external signals and states.
    """
    
    """!
    @image html servo_task.png
    """
    
    """
    Attributes:
        CLOSE (share): A share indicating whether to close the servo that act as a blindfold.
        SER_DIR (share): A share indicating the direction of the servo that rotate the ultrasonic distance sensor.
        state (int): The current state of the servo task.
        S0_INIT (int): State identifier for initialization.
        S1_HUB (int): State identifier for hub position.
        S2_MIDDLE (int): State identifier for middle position.
        S3_RIGHT (int): State identifier for right position.
        S4_LEFT (int): State identifier for left position.

    Methods:
        __init__(self, SER_DIR, CLOSE): Initializes the ServoTask instance with direction and close signals.
        run(self): Runs the servo task, controlling the servo based on the current state.
    """

    def __init__(self, SER_DIR, CLOSE):
        """
        Initializes the ServoTask instance.
          
        Parameters:
            @param SER_DIR (share): A share indicating the direction of the servo.
            @param CLOSE (share): A share indicating whether to close the servo.
        """
        """!
        Example:
          @code
              '''! This code is use to set mg90s servo to any desired angle from -90° to 90° '''
                  
              # Import libraries
              from pyb import Pin, Timer
              
              # Initiate 2 pins (trigger and echo) with appropriate timer
              servoTimer = Timer(3, freq=50)
              servoPin = servoTimer.channel(3, pin=Pin.cpu.B0, mode=Timer.PWM, pulse_width_percent=0)
              
              # Set angle to 45°
              angle = 45
              
              # Switch servo to 45°
              set_servo(45, servoPin)

            @endcode
        """
        
        ## Share that use to close or open the blind fold
        self.CLOSE = CLOSE
        
        ## Share that implement servo angle that connecto the ultrasonic sensor
        self.SER_DIR = SER_DIR
        
        ## This task state
        self.state = 0
        
        ## State 0: Init both servos
        self.S0_INIT = 0
        
        ## State 1: Servo hub that analysis input shares to control 2 servos
        self.S1_HUB = 1
        
        ## State 2: ROMI look forward
        self.S2_MIDDLE = 2
        
        ## State 3: ROMI look right
        self.S3_RIGHT = 3
        
        ## State 4: ROMI look left
        self.S4_LEFT = 4

    def run(self):
        """
        
        Runs the servo task in an infinite loop, transitioning between different states 
        to control servo positions based on external signals.

            STATE    NAME      DESCRIPTIOM
              0      INIT      This state init 2 servos, one for controlling ultrasonic sensor and one for the blindfold
              1      HUB       This state continuously read SER_DIR and CLOSE share to and send to appropriate state below
              2      MIDDLE    Servo 1 = 0° || Servo 2 = -45° if CLOSE = 0 and Servo 2 = 90° of CLOSE = 1
              3      RIGHT     Servo 1 = -90°
              4      LEFT      Servo 1 = 90°
              
        """
        while True:
            if self.state == self.S0_INIT:
                servoTimer = Timer(3, freq=50)
                servoPin = servoTimer.channel(3, pin=Pin.cpu.B0, mode=Timer.PWM, pulse_width_percent=0)
                servoTimer2 = Timer(2, freq=50)
                servoPin2 = servoTimer2.channel(4, pin=Pin.cpu.A3, mode=Timer.PWM, pulse_width_percent=0)
                self.state = self.S1_HUB
                
            elif self.state == self.S1_HUB:
                if self.CLOSE.get() == 0:    
                    set_servo(-45, servoPin2)
                else:
                    set_servo(90, servoPin2)
                    
                if self.SER_DIR.get() == 0:
                    self.state = self.S2_MIDDLE
                elif self.SER_DIR.get() == 1:
                    self.state = self.S3_RIGHT
                elif self.SER_DIR.get() == -1:
                    self.state = self.S4_LEFT
                else: 
                    self.state = self.S1_HUB
                    
            elif self.state == self.S2_MIDDLE:

                set_servo(0, servoPin)
                self.state = self.S1_HUB

            elif self.state == self.S3_RIGHT:
                set_servo(-90, servoPin)
                self.state = self.S1_HUB
                
            elif self.state == self.S4_LEFT:
                set_servo(90, servoPin)
                self.state = self.S1_HUB

            else:
                print("IMU_TASK Invalid State!!!")
                       
            yield self.state

def set_servo(angle, servoPin):
    """!
    Sets the servo motor to a specific angle due to PWM signal.

    @param angle (float): The desired angle to set the servo.
    @param servoPin (timer): The PWM channel associated with the servo motor.
    
    """
    pulse = 1.5 + angle / 90
    percent = (pulse / 20) * 100
    servoPin.pulse_width_percent(percent)
