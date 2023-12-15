'''!
@file                   task_ULS.py
@brief                  This file implement a task that interface with a Ultrasonic Distance sensor
@author                 Vinh Vo 
@author                 Quinn Stephens
@date                   12/09/2023
'''

from pyb import Pin, udelay, micros

class ULSTask:
    """!
    A class for managing Ultrasonic Sensor readings in a state machine manner. 
    It measures distances and outputs them to a share.
    """
    
    """!
    @image html uls_task.png
    """
    
    
    """
    Attributes:
        ULS_DIS (share): A share to store distance measurements.
        state (int): The current state of the ultrasonic sensor task.
        S0_INIT (int): State value representing initialization state.
        S1_READ (int): State value representing read state.

    Methods:
        __init__(self, ULS_DIS): Initializes the ULSTask instance.
        run(self): Runs the ultrasonic sensor task in a loop, handling initialization and reading of distance data.
    """

    def __init__(self, ULS_DIS):
        """!
        Initializes the ULSTask instance.
        @param ULS_DIS (share): A share for storing distance measurements.
        
        Example:
          @code
              '''! This code sample is used to read and print the distance of the ultrasonic sensor in cm'''
              
              # import libraries
              from pyb import Pin, udelay, micros
              
              # Initiate 2 pins (trigger and echo) with appropriate timer
              trig_pin = Pin(Pin.cpu.B5, mode=Pin.OUT_PP)
              echo_pin = Pin(Pin.cpu.B4, mode=Pin.IN)

              # Read distance in [cm]
              distance = dist(trig_pin, echo_pin)
              print(f"The distance is: {distance} [cm]")
                  
          @endcode
            
        """
        ## Share that store the distance that the ultrasonic sensor read in [cm]
        self.ULS_DIS = ULS_DIS
        
        ## This task state
        self.state = 0
        
        ## State 0: Init ultrasonic sensor through trigger and echo pins
        self.S0_INIT = 0
        
        ## State 1: continuously read distance in [cm]
        self.S1_READ = 1

    def run(self):
        """
        Runs the ultrasonic sensor task in a loop. Manages the state of the task, 
        initializing the ultrasonic sensor and reading distance values.

            STATE    NAME                             DESCRIPTIOM
              0      INIT      This state init the ultrasonics sensor's trigger and echo pins 
              1      READ      This state continuously read the distance [cm]rom the ultrsonic sensor using dist(trig_pin, echo_pin) function  
                               and update to ULS_DIS share  
              
        """
        while True:
            if self.state == self.S0_INIT:
                trig_pin = Pin(Pin.cpu.B5, mode=Pin.OUT_PP)
                echo_pin = Pin(Pin.cpu.B4, mode=Pin.IN)
                self.state = self.S1_READ

            elif self.state == self.S1_READ:
                self.ULS_DIS.put(dist(trig_pin, echo_pin))
                self.state = self.S1_READ

            else:
                print("ULS_TASK Invalid State!!!")

            yield self.state

def dist(trig_pin, echo_pin):
    """!
    Measures the distance using an ultrasonic sensor.

    @param trig_pin (Pin): The trigger pin of the ultrasonic sensor.
    @param echo_pin (Pin): The echo pin of the ultrasonic sensor.

    @return (float): The measured distance in centimeters.
    """
    trig_pin.high()
    udelay(10)
    trig_pin.low()
    while echo_pin.value() == 0:
        pass
    start = micros()
    while echo_pin.value() == 1:
        pass
    end = micros()
    duration = end - start
    distance = (duration * 343.0) / 2 / 10000  # Convert to cm
    return distance
