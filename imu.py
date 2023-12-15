"""!
@file imu.py
This file implement a class that interface with an BNO055 IMU Sensor.

@author Vinh Vo
@author Quinn Stephens
@date   2023-Dec-11 
"""

import time
import struct

class BNO055Driver:
    
    """!
    A driver class for the BNO055 IMU sensor, providing functionality to interact with
    the sensor over I2C to retrieve and manipulate desired data
    """
    """
    Attributes:
        i2c (I2C): I2C object for communication with the sensor.
        yaw (float): Yaw value from the sensor.
        add (int): I2C address of the sensor.
        opr_add (int): Address of the operation mode register.
        calibrate_add (int): Address of the calibration status register.
        coeff_start (int): Start address of the calibration coefficients.
        euler_start (int): Start address of the Euler angles data.
        gyros_start (int): Start address of the gyroscope data.
        magns_start (int): Start address of the magnetometer data.
        coeffs (list): Calibration coefficients.
        fusion_cmd (dict): Mapping of fusion modes register address to their corresponding command values.

    Methods:
        __init__(self, i2c): Initializes the BNO055Driver instance.
        set_mode(self, mode): Sets the operation mode of the sensor.
        get_cal_stat(self): Retrieves the calibration status.
        get_cal_coeffs(self): Retrieves the calibration coefficients from the sensor.
        update_cal_coeffs(self): Updates the calibration coefficients from a text file.
        write_cal_coeffs(self): Writes the current calibration coefficients to a text file.
        read_eulers(self): Reads the Euler angles from the sensor.
        read_gyros(self): Reads the gyroscope data from the sensor.
        update_yaw(self, yaw): Updates and corrects the yaw value everytime reset MCU.
    """

    def __init__(self, i2c):
        """!
        Initializes a new instance of the BNO055Driver class.

        @param i2c (I2C): The I2C object used for communication with the sensor.
            
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
              
              IMU_YAW.put(radians(bno.update_yaw(bno.read_eulers() - self.init_yaw)))
              print("The YAW angle is: {IMU_YAW} [rad]")
          @endcode
        """
        
        ## I2C bus from MCU
        self.i2c = i2c
        
        ## Yaw Angle
        self.yaw = 0
        
        ## IMU address
        self.add = 0x28
        
        ## Operation address
        self.opr_add = 0x3D
        
        ## Calibration address
        self.calibrate_add = 0x35
   
        ## Starting address of calibration coefficient (22)
        self.coeff_start = 0x55
        
        ## Starting address of euler angles (6)
        self.euler_start = 0x1A
        
        ## Starting address of gyroscope angular speed (6)
        self.gyros_start = 0x14
        
        ## Starting address of magnetic values (6)
        self.magns_start = 0x0E
        
        ## Calibration coeeficient
        self.coeffs = []
        
        ## Operation fusion mode dictionary
        self.fusion_cmd = {
            'CONFIG': 0x00,
            'IMU': 0x08,
            'COMPASS': 0x09,
            'M4G': 0x0A,
            'NDOF_FMC_OFF': 0x0B,
            'NDOF': 0x0C
        }

    def set_mode(self, mode):
        """!
        Sets the operation mode of the BNO055 sensor.

        @param mode (str): The mode to set the sensor to.
        
        @return ValueError (exception): If an invalid mode is passed.
        """
        if mode not in self.fusion_cmd:
            raise ValueError("Invalid mode")
        cmd = self.fusion_cmd[mode]
        self.i2c.mem_write(cmd, self.add, self.opr_add)
        time.sleep_ms(50)

    def get_cal_stat(self):
        """!
        Retrieves the calibration status of the BNO055 sensor.

        @return (tuple): A tuple containing the calibration status of system, gyro, accelerometer, and magnetometer.
        """
        self.set_mode('NDOF')
        time.sleep_ms(50)
        cal_stat = self.i2c.mem_read(1, self.add, self.calibrate_add)
        data = (cal_stat[0])
        sys = (data >> 6) & 0x03
        gyro = (data >> 4) & 0x03
        accel = (data >> 2) & 0x03
        mag = data & 0x03
        return sys, gyro, accel, mag

    def get_cal_coeffs(self):
        """!
        Retrieves the calibration coefficients from the sensor.

        @return (list): A list of calibration coefficients.
        """
        time.sleep_ms(50)
        coeff = self.i2c.mem_read(22, self.add, self.coeff_start)
        self.coeffs = []
        for byte in coeff:
            self.coeffs.append(hex(byte))
        return self.coeffs

    def update_cal_coeffs(self):
        """!
        Updates the calibration coefficients from a file.

        @return (bytearray): The bytes written to the sensor.
        """
        with open('calibration_coeff.txt', 'r') as file:
            line = file.readline()
        line2 = line.split(",")
        coeff = []
        for b in line2:
            coeff.append(int(b, 16))
        byte_in = bytearray(coeff)
        self.i2c.mem_write(byte_in, self.add, self.coeff_start)
        return byte_in

    def write_cal_coeffs(self):
        """!
        Writes the current calibration coefficients to a file.
        """
        with open('calibration_coeff.txt', 'w') as file:
            file.write(str(','.join(self.coeffs)))

    def read_eulers(self):
        """!
        Reads the Euler angles from the sensor.

        @return (float): The yaw angle in degrees.
        """
        euler = self.i2c.mem_read(6, self.add, self.euler_start)
        yaw, pitch, roll = struct.unpack('<hhh', euler)
        return -yaw / 16

    def read_gyros(self):
        """!
        Reads the gyroscope data from the sensor.

        @return (tuple): A tuple of gyroscope readings in degrees per second (x, y, z).
        """
        gyro = self.i2c.mem_read(6, self.add, self.gyros_start)
        xd, yd, zd = struct.unpack('<hhh', gyro)
        return xd / 16, yd / 16, zd / 16

    def update_yaw(self, yaw):
        """!
        Updates and corrects the yaw value base on initial angle.

        @param yaw (float): The raw yaw value.

        @return (float_: The corrected yaw value.
        """
        if yaw < 0:
            yaw += 360
        elif yaw >= 360:
            yaw -= 360
        return yaw