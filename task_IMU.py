'''!
@file                   task_IMU.py
This file contain a task that run BNO055 IMU Sensor and processing the data

@author                 Vinh Vo 
@author                 Quinn Stephens
@date                   2023-Dec-11 
'''
from pyb import I2C
from imu import BNO055Driver
from time import sleep_ms
from math import radians

class IMUTask:
    """!
    A task class for managing IMU readings and processing them in a state machine manner.
    """
    
    """!
    @image html imu_task.png
    """
    
    """  
    Attributes:
        IMU_YAW (share): A share to store the processed yaw values after corrected.
        state (int): The current state of the IMU task.
        S0_INIT (int): State value representing initialization state.
        S1_READ (int): State value representing reading yaw state.
        init_yaw (float): Initial yaw reading for calibration.

    Methods:
        __init__(self, IMU_YAW): Initializes the IMUTask instance.
        run(self): Runs the IMU task in a loop, handling initialization and reading of IMU data.
    """

    def __init__(self, IMU_YAW):
        """!
        Initializes the IMUTask instance.
        
        @param IMU_YAW (share): A share for storing correcte yaw value of ROMI.
        """
        
        """!
        Example:
          @code
              '''! This code sample is used to read and print the yaw angle after corrected everytime turn ROMI on'''
              
              # Initiate the IMU
              i2c = I2C(1, I2C.CONTROLLER)
              bno = BNO055Driver(i2c)
              
              # Reset to CONFIG mode before change to NDOF mode
              bno.set_mode('CONFIG')
              sleep_ms(10)
              bno.set_mode('NDOF')
              
              # Update calibration data from local drive
              coeff = bno.update_cal_coeffs()
              
              # Read initial yaw angle
              init_yaw = bno.read_eulers()
              
              # Read IMU yaw
              IMU_YAW.put(radians(bno.update_yaw(bno.read_eulers() - self.init_yaw)))
              print("The YAW angle is: {IMU_YAW} [rad]")
                  
          @endcode
        """
        
        ## IMU yaw angle [rad] that is share with task_MOT
        self.IMU_YAW = IMU_YAW
        
        ## This task state
        self.state = 0
        
        ## State 0: Init IMU & update calibration coefficient
        self.S0_INIT = 0
        
        ## State 1: continuously read yaw angle
        self.S1_READ = 1
        
        ## Initial yaw angle when first turn on ROMI
        self.init_yaw = 0

    def run(self):
        
        """
        Runs the IMU task in a loop. Manages the state of the task, initializing the IMU
        and reading yaw values, correct them based on the initial reading.
        
            STATE    NAME                             DESCRIPTIOM
              0      INIT      This state init the BNO055 driver and update the calibration coefficients
              1      READ      This state continuously read the corrected yaw angle from the IMU and update IMU_YAW share
              
        """
        
        while True:
            if self.state == self.S0_INIT:
                i2c = I2C(1, I2C.CONTROLLER)
                bno = BNO055Driver(i2c)
                bno.set_mode('CONFIG')
                sleep_ms(10)
                bno.set_mode('NDOF')
                coeff = bno.update_cal_coeffs()
                self.init_yaw = bno.read_eulers()
                self.state = self.S1_READ
            elif self.state == self.S1_READ:
                self.IMU_YAW.put(radians(bno.update_yaw(bno.read_eulers() - self.init_yaw)))
                self.state = self.S1_READ
            else:
                print("IMU_TASK Invalid State!!!")
            yield self.state
