'''!
@file task_MOT.py
This file implement a task that control ROMI motor base on input share from different task

@author Vinh Vo 
@author Quinn Stephens
@date 2023-Dec-11 
'''
from pyb import Timer, Pin, ADC, ExtInt
from encoder import Encoder
from l6206 import L6206
from closedLoopPID import PIDController as pid
from math import cos, sin, pi, sqrt

class MotorTask:
    """!
    This file implement a task that control ROMI motor base on input share from different task
    """
    
    """!
    @image html motor_task.png
    """
    
    """
    A class for managing ROMI DC motor to run it through the course include following objective
        1. Following 0.75 inches width line with different pattern
        2. Go around the a 10 inches segment long square wall obstacle and get back to the line
        3. Stop at the end of the line
        4. Return the the original starting point
        
    Attributes:
        
        Shares:
            SER_DIR (share): A share to set servo direction(ultrasonic sensor attached) from task_SER.py
            IMU_YAW (share): A share to get the yaw angle of the attached IMU from task_IMU.py
            ULS_DIS (share): A share to get the sensing distance from the ultrasonic sensor in task_ULS.py
            CLOSE   (share): A share to set blindfold condition from task_SER.py
        
        States:
            state (int): The current state of this motor task.
            S0_INIT (int): Init all required sensors and actuators (more detail below run() section) 
            S1_HUB (int): Motor hub that act as the brain to control the states base on sensor condition
            S2_PATH (int): ROMI follows the design path using 6 line sensors
            S3_WALL1 (int): ROMI meets the wall and turn left 90°, then goes straight until not see the wall then go to S3_WALL2
            S3_WALL2 (int): ROMI turn right 90°, then goes straight until not see the wall then go to S3_WALL3
            S3_WALL3 (int): ROMI turn right 90°, then goes straight untill see the black line
            S3_WALL4 (int): ROMI turn left 90°, then set flag WALL = 1 and go back to S1_HUB
            S4_RETURNX (int): ROMI facing to X axis and run forward until reach X < 0.02 [m]
            S4_RETUNRY (int): ROMI facing to Y axis and run forward until reach Y < 0.02 [m]
            S5_HOME (int): ROMI get back to the HOME position and pivot to celebrate
        
        Others:
            X (float): variable that continuosly store updated X coordinate of ROMI
            Y (float): variable that continuosly store updated Y coordinate of ROMI
            EXP_DIST (float): traveled distance that ROMI can go in [explore mode] in S2_PATH state (for detecting TARGET and HOME position)
            WALL (int): Wall flag that turn to 1 if ROMI pass the wall obstacle
            TARGET (int): Target flag that turn to 1 if ROMI reach the target
            HOME (int): Home flag that turn to 1 if ROMI successfully return the home position 
            OLD_YAW (float): Variable that store the old angle before turning to avoid the wall
            WALL1_YAW (float): Variable that store the target angle to turn to avoid the first side of the wall
            WALL2_YAW (float): Variable that store the target angle to turn to avoid the second side of the wall

    Methods:
        __init__(self, ULS_DIS): Initializes the ULSTask instance.
        run(self): Runs the motor task in a loop, handling all sensors and actuators appropriately.
        check_sensor(L2, L1, M, R1, R2, H): take the status of 6 line sensor then return the appropriate [case]
        update_speed(case): take the [case] variable then return the appropriate yaw rate speed [rad/s] and linear speed [m/s] 
        DC_speed_cal(y, v): take the yaw rate speed [rad/s] and linear speed [m/s] to process 
                            and return the appropriate angular speed [rad/s] for both right and left wheels 
        check_return(X, Y, state): take the global location X and Y of ROMI as well as the current state, return the new state and HOME flag when X and Y < 0.02 [m] 
    """
    
    def __init__(self,SER_DIR, IMU_YAW, ULS_DIS, CLOSE):
        ## Share that use to close or open the blindfold
        self.CLOSE     = CLOSE
        
        ## Share that implement servo angle that connecto the ultrasonic sensor
        self.SER_DIR   = SER_DIR
        
        ## Share that store the distance that the ultrasonic sensor read in [cm]
        self.ULS_DIS   = ULS_DIS
        
        ## IMU yaw angle [rad] that is share with task_MOT
        self.IMU_YAW   = IMU_YAW
        
        ## Simutaneous small linear distance to keep track of ROMI global coordinate locations
        self.DIST_PATH = 0
        
        ## X-Coordinate
        self.X         = 0
        
        ## Y-Coordinate
        self.Y         = 0
        
        ## Flag that is 1 if already pass the wall and 0 otherwise
        self.WALL      = 0
        
        ## Flag that is 1 if already stop at the target spot and 0 otherwise
        self.TARGET    = 0
        
        ## Flag that is 1 if already sucessfully return home and 0 otherwise
        self.HOME      = 0
        
        ## Traveled distance that ROMI can go in [explore mode] S2_PATH only
        self.EXP_DIST = 0
        
        ## The target angle to turn to avoid the first side of the wall
        self.WALL1_YAW = 0
        
        ## The target angle to turn to avoid the second side of the wall
        self.WALL2_YAW = 0
        
        ## The old angle before turning to avoid the wall
        self.OLD_YAW   = 0 
        
        ## The current state of this motor task.
        self.state     = 0
        
        ## Init all required sensors and actuators
        self.S0_INIT   = 0
        
        ## Motor hub that control the states base on sensor condition
        self.S1_HUB    = 1
        
        ## ROMI follows the design path using 6 line sensors
        self.S2_PATH   = 2
        
        ## ROMI meets the wall and turns left 90°, then go straight until not see the wall
        self.S3_WALL1  = 3
        
        ## ROMI turns right 90° then goes straight untill not see the wall 
        self.S3_WALL2  = 4
        
        ## ROMI turns right 90° then goes straight until pickup signal from the line sensor
        self.S3_WALL3  = 5
    
        ## ROMI turns left 90° then go back to follow the line
        self.S3_WALL4  = 6
        
        ## ROMI facing to X axis and run forward until reach X < 0.02 [m]
        self.S4_RETURNX= 7
        
        ## ROMI facing to Y axis and run forward until reach Y < 0.02 [m]
        self.S4_RETURNY= 8
        
        ## ROMI get back to the HOME position and pivot to celebrate
        self.S5_HOME   = 9 
        
    def run(self):
        """
        Run and manage ROMI's DC motor to run it through the course using the data from sensors to control the actuators

            STATE    NAME                             DESCRIPTIOM
              0      INIT      This state init the [2] DC motor, [2] encoders, [6] line sensor, [2] PID controller, [1] ADC pin for calibrate mode 
              
              1      HUB       This state act as a hub to decide which state to go base on condtion
                               The logic for this task is descibe as following:
                                  - if [in the calibration mode] stop and set all flags to 0
                                  - else if [HOME == 1 and TARGET == 1], state = S5_HOME
                                  - else if [no obstacle ditected and TARGET == 0], state = S2_PATH
                                  - else if [obstacle dictected and TARGET == 0 and WALL == 0], state = S3_WALL1
                                  - else if [TARGET == 1 and HOME == 0]: run check_return --> state = S4_RETURNX if X < 0.02 [m]
                                                                                              state = S4_RETURNY if Y < 0.02 [m]
              2      PATH      This state control ROMI to follow the line with condition from the line sensor
                               The logic for this task is descibe as following:
                                   - read the line sensors status
                                   - check the sensors status for [case] --> check_sensor()
                                       if [case == "explore"] and EXP_DIST > 0.15, case = "stop"
                                   - use [case] for finding the yaw rate and linear speed -->  update_speed()
                                   - use the yaw rate and linear speed to calculate both wheels angular speed --> DC_speed_cal()
                                   - update speed for ROMI's 2 DC motor and return S1_HUB 
                                   
              3a     WALL1      This state turn ROMI left for 90° and turn the first servo right 90° then go straight until not see the wall
              3b     WALL2      This state turn ROMI right for 90° and keep the first servo the same angle then go straight until not seeing the wall
              3c     WALL3      This state turn ROMI right for 90° then go straigh until pickup line sensor signal
              3d     WALL4      This state tune ROMI left for 90° then set flag WALL = 1 and go back to S1_HUB
              4x     RETURNX    This state turn ROMI to 180° from the original angle from stating point then:
                                   - if [HOME == 0 and abs(X) < 0.02] 
                                       if X > 0 then go forward and X < 0 backup
              4y     RETURNY    This state turn ROMI to 90° from the original angle from stating point then:
                                   - if [HOME == 0 and abs(Y) < 0.02] 
                                       if Y > 0 then go forward and Y < 0 backup
              5      HOME       ROMI at home position, pivoting and put the "money mask" [2nd servo] on
        """
        
        while True: 
            if self.state == self.S0_INIT:  
                button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, lambda p: self.CLOSE.put(0 if self.CLOSE.get() == 1 else 1))    
                tim_R = Timer(4, freq=20000)
                tim_L = Timer(4, freq=20000)
                mot_R = L6206(tim_R, 2, Pin.cpu.A2, Pin.cpu.A10, Pin.cpu.B7)
                mot_L = L6206(tim_L, 1, Pin.cpu.B3, Pin.cpu.B10, Pin.cpu.B6)
                mot_R.enable()
                mot_L.enable()
                tim_Re = Timer(1, period=5000, prescaler=0)
                tim_Le = Timer(2, period=5000, prescaler=0)
                enc_L = Encoder (tim_Le, 1, 2, Pin.cpu.A0, Pin.cpu.A1)
                enc_R = Encoder (tim_Re, 1, 2, Pin.cpu.A8, Pin.cpu.A9)
                PID_R = pid(3, 0.5, 0.5)
                PID_L = pid(3, 0.5, 0.5)
                
                PC0 = Pin(Pin.cpu.C0)
                PC1 = Pin(Pin.cpu.C1)
                PA5 = Pin(Pin.cpu.A5)
                PA6 = Pin(Pin.cpu.A6)
                PA7 = Pin(Pin.cpu.A7)
                PC3 = Pin(Pin.cpu.C3)
                PA4 = Pin(Pin.cpu.A4)

                line_L1 = ADC(PA7)
                line_R1 = ADC(PA5)
                line_L2 = ADC(PC0)
                line_R2 = ADC(PC1)
                line_M  = ADC(PA6)
                line_H  = ADC(PA4)
                
                cal_mode= ADC(PC3)

  
                
                self.state = self.S1_HUB
                
            elif self.state == self.S1_HUB:
                
                if cal_mode.read()  < 10:
                    case = "stop"
                    wL, wR = update_speed(case)
                    enc_R.update()
                    wR_meas = enc_R.get_rad_s()
                    pid_out_R = PID_R.update(wR,wR_meas)
                    mot_R.set_duty(pid_out_R)
                    enc_L.update()
                    wL_meas = enc_L.get_rad_s()
                    pid_out_L = PID_L.update(wL,wL_meas)
                    mot_L.set_duty(pid_out_L)
                    self.TARGET = 0
                    self.HOME = 0 
                    self.WALL = 0
                    self.SER_DIR.put(0)
                    self.state = self.S1_HUB
                    
                elif self.HOME == 1 and self.TARGET == 1:
                    self.state = self.S5_HOME
                    
            
                elif self.ULS_DIS.get() > 15 and self.TARGET == 0 :
                    self.state = self.S2_PATH
                    
                elif self.ULS_DIS.get() < 15 and self.TARGET == 0:
                    self.OLD_YAW = self.IMU_YAW.get()
                    self.WALL1_YAW = self.OLD_YAW + pi/2
                    if self.WALL1_YAW > 2*pi:
                        self.WALL1_YAW -= 2*pi
                    if self.WALL == 1:
                        self.state = self.S2_PATH 
                    else:
                        self.state = self.S3_WALL1
                    
                elif self.TARGET == 1 and self.HOME == 0:
                    self.state, self.HOME = check_return(self.X, self.Y, self.state)
                    
                else:
                    self.state = self.S1_HUB
                
            elif self.state == self.S2_PATH:
                
                dD = int((enc_R.get_delta() + enc_L.get_delta())/2)              
                dx = (dD * (cos(self.IMU_YAW.get()))) * ((pi*0.070)/1440) 
                dy = (dD * (sin(self.IMU_YAW.get()))) * ((pi*0.070)/1440) 
                self.X += dx
                self.Y += dy
                L1 = line_L1.read()
                R1 = line_R1.read()
                M  = line_M.read()
                L2 = line_L2.read()
                R2 = line_R2.read()
                H  = line_H.read()
                
                case = check_sensor(L2, L1, M, R1, R2, H)
                
                if case == "explore":
                    self.EXP_DIST += float(sqrt(dx**2 + dy**2))
                    if self.EXP_DIST > 0.15:
                        case = "stop"
                        self.TARGET = 1
                else:
                    self.EXP_DIST = 0    
                wL, wR = update_speed(case)
                enc_R.update()
                wR_meas = enc_R.get_rad_s()
                pid_out_R = PID_R.update(wR,wR_meas)
                mot_R.set_duty(pid_out_R)
                enc_L.update()
                wL_meas = enc_L.get_rad_s()
                pid_out_L = PID_L.update(wL,wL_meas)
                mot_L.set_duty(pid_out_L)
                self.SER_DIR.put(0)
                self.state = self.S1_HUB
                
            elif self.state == self.S3_WALL1:
                if cal_mode.read()  < 10:
                    self.state = self.S1_HUB
                dD = int((enc_R.get_delta() + enc_L.get_delta())/2)              
                dx = (dD * (cos(self.IMU_YAW.get()))) * ((pi*0.070)/1440) 
                dy = (dD * (sin(self.IMU_YAW.get()))) * ((pi*0.070)/1440) 
                self.X += dx
                self.Y += dy
                self.SER_DIR.put(1)
                if abs(self.WALL1_YAW - self.IMU_YAW.get()) > 0.1   :
                    case = "wall1"                    
                else:
                    if self.ULS_DIS.get() < 30:
                        case = "straight"
                        self.state = self.S3_WALL1
                    else:
                        self.state = self.S3_WALL2
 
                wL, wR = update_speed(case)
                enc_R.update()
                wR_meas = enc_R.get_rad_s()
                pid_out_R = PID_R.update(wR,wR_meas)
                mot_R.set_duty(pid_out_R)     
                enc_L.update()
                wL_meas = enc_L.get_rad_s()
                pid_out_L = PID_L.update(wL,wL_meas)
                mot_L.set_duty(pid_out_L)
            
            elif self.state == self.S3_WALL2:
                if cal_mode.read()  < 10:
                    self.state = self.S1_HUB
                dD = int((enc_R.get_delta() + enc_L.get_delta())/2)              
                dx = (dD * (cos(self.IMU_YAW.get()))) * ((pi*0.070)/1440) 
                dy = (dD * (sin(self.IMU_YAW.get()))) * ((pi*0.070)/1440) 
                self.X += dx
                self.Y += dy
                self.SER_DIR.put(1)
                
                if abs(self.OLD_YAW - self.IMU_YAW.get()) > 0.1   :
                    case = "wall2"                    
                else:
                    if self.ULS_DIS.get() < 30:
                        case = "straight"
                        self.state = self.S3_WALL2
                    else:
                        self.OLD_YAW = self.IMU_YAW.get()
                        self.WALL2_YAW = self.OLD_YAW - pi/2
                        if self.WALL2_YAW < 0:
                            self.WALL2_YAW += 2*pi
                        self.state = self.S3_WALL3
 
                wL, wR = update_speed(case)
                enc_R.update()
                wR_meas = enc_R.get_rad_s()
                pid_out_R = PID_R.update(wR,wR_meas)
                mot_R.set_duty(pid_out_R)     
                enc_L.update()
                wL_meas = enc_L.get_rad_s()
                pid_out_L = PID_L.update(wL,wL_meas)
                mot_L.set_duty(pid_out_L)
                
            elif self.state == self.S3_WALL3:
                if cal_mode.read()  < 10:
                    self.state = self.S1_HUB
                dD = int((enc_R.get_delta() + enc_L.get_delta())/2)              
                dx = (dD * (cos(self.IMU_YAW.get()))) * ((pi*0.070)/1440) 
                dy = (dD * (sin(self.IMU_YAW.get()))) * ((pi*0.070)/1440) 
                self.X += dx
                self.Y += dy
                self.SER_DIR.put(1)
                
                if abs(self.WALL2_YAW - self.IMU_YAW.get()) > 0.1:
                    case = "wall2" 
                else:
                    L1 = line_L1.read()
                    R1 = line_R1.read()
                    M  = line_M.read()
                    L2 = line_L2.read()
                    R2 = line_R2.read()
                    H  = line_H.read()
                    

                    case = check_sensor(L2, L1, M, R1, R2, H)
                    if case != "explore":
                        self.state = self.S3_WALL4
                    else:
                        self.state = self.S3_WALL3
                
                wL, wR = update_speed(case)
                enc_R.update()
                wR_meas = enc_R.get_rad_s()
                pid_out_R = PID_R.update(wR,wR_meas)
                mot_R.set_duty(pid_out_R)     
                enc_L.update()
                wL_meas = enc_L.get_rad_s()
                pid_out_L = PID_L.update(wL,wL_meas)
                mot_L.set_duty(pid_out_L)
                 
                        
            elif self.state == self.S3_WALL4:
                if cal_mode.read()  < 10:
                    self.state = self.S1_HUB
                dD = int((enc_R.get_delta() + enc_L.get_delta())/2)              
                dx = (dD * (cos(self.IMU_YAW.get()))) * ((pi*0.070)/1440) 
                dy = (dD * (sin(self.IMU_YAW.get()))) * ((pi*0.070)/1440) 
                self.X += dx
                self.Y += dy
                self.SER_DIR.put(-1)
                
                if abs(self.OLD_YAW - self.IMU_YAW.get()) > 0.1   :
                    case = "wall1" 
                    self.state = self.S3_WALL4
                else:
                    self.WALL = 1
                    self.state = self.S1_HUB
                    
                wL, wR = update_speed(case)
                enc_R.update()
                wR_meas = enc_R.get_rad_s()
                pid_out_R = PID_R.update(wR,wR_meas)
                mot_R.set_duty(pid_out_R)     
                enc_L.update()
                wL_meas = enc_L.get_rad_s()
                pid_out_L = PID_L.update(wL,wL_meas)
                mot_L.set_duty(pid_out_L)
                
            elif self.state == self.S4_RETURNX:
                if cal_mode.read()  < 10:
                    self.state = self.S1_HUB
                dD = int((enc_R.get_delta() + enc_L.get_delta())/2)              
                dx = (dD * (cos(self.IMU_YAW.get()))) * ((pi*0.070)/1440) 
                dy = (dD * (sin(self.IMU_YAW.get()))) * ((pi*0.070)/1440) 
                self.X += dx
                self.Y += dy
                self.SER_DIR.put(0)
                self.CLOSE.put(0)    
                
                if (self.IMU_YAW.get() - pi) > 0.1:
                    case = "pivot right"
                elif (self.IMU_YAW.get() - pi) < -0.1:
                    case = "pivot left"
                    
                elif abs(self.X) > 0.02:
                    if self.X > 0:
                        case = "straight"
                    else:
                        case = "backup"
                else:
                    case = "stop"
                
                    
                wL, wR = update_speed(case)
                enc_R.update()
                wR_meas = enc_R.get_rad_s()
                pid_out_R = PID_R.update(wR,wR_meas)
                mot_R.set_duty(pid_out_R)     
                enc_L.update()
                wL_meas = enc_L.get_rad_s()
                pid_out_L = PID_L.update(wL,wL_meas)
                mot_L.set_duty(pid_out_L)
                self.state = self.S1_HUB
                
            elif self.state == self.S4_RETURNY:
                if cal_mode.read()  < 10:
                    self.state = self.S1_HUB
                dD = int((enc_R.get_delta() + enc_L.get_delta())/2)              
                dx = (dD * (cos(self.IMU_YAW.get()))) * ((pi*0.070)/1440) 
                dy = (dD * (sin(self.IMU_YAW.get()))) * ((pi*0.070)/1440) 
                self.X += dx
                self.Y += dy
                self.SER_DIR.put(0)
                self.CLOSE.put(1)    
                if (self.IMU_YAW.get() - pi/2) > 0.1:
                    case = "pivot right"
                elif (self.IMU_YAW.get() - pi/2) < -0.1:
                    case = "pivot left"
                    
                elif abs(self.Y) > 0.02:
                    if self.Y > 0:
                        case = "backup"
                    else:
                        case = "straight"
                else:
                    case = "stop"
                    
                    
                wL, wR = update_speed(case)
                enc_R.update()
                wR_meas = enc_R.get_rad_s()
                pid_out_R = PID_R.update(wR,wR_meas)
                mot_R.set_duty(pid_out_R)     
                enc_L.update()
                wL_meas = enc_L.get_rad_s()
                pid_out_L = PID_L.update(wL,wL_meas)
                mot_L.set_duty(pid_out_L)
                self.state = self.S1_HUB
                
            elif self.state == self.S5_HOME:
                if cal_mode.read()  < 10:
                    self.HOME = 0
                
                case = "done"
                self.CLOSE.put(1)

                wL, wR = update_speed(case)
                enc_R.update()
                wR_meas = enc_R.get_rad_s()
                pid_out_R = PID_R.update(wR,wR_meas)
                mot_R.set_duty(pid_out_R)     
                enc_L.update()
                wL_meas = enc_L.get_rad_s()
                pid_out_L = PID_L.update(wL,wL_meas)
                mot_L.set_duty(pid_out_L)
                self.state = self.S1_HUB


            else:   
                print("MOTOR: INVALID STATE") 
                
            yield self.state
            

            
def check_sensor(L2, L1, M, R1, R2, H):
    
    """!
    Read the status from 6 line sensors determine what case for ROMI.

    @param L2 (float): far left line sensor reading 
    @param R2 (float): far right line sensor reading 
    @param L1 (float): close left line sensor reading 
    @param R1 (float): close right line sensor reading 
    @param M (float): close middle line sensor reading 
    @param H (float): far middle line sensor reading 

    @return (string): case that ROMI DC motor need to run .
    
    """
    
    if L2 < 2000 and L1 < 2000 and M < 2000 and R1 < 2000 and R2 < 2000:
        return "cross"
    elif L2 < 2000 and L1 < 2000 and M < 2000:
        return "90 left"
    elif M < 2000 and R1 < 2000 and R2 < 2000:
        return "90 right"
    elif L2 < 2000 and L1 < 2000:
        return "hard left"
    elif R2 < 2000 and R1 < 2000:
        return "hard right"
    elif L2 < 2000:
        return "really hard left"
    elif R2 < 2000:
        return "really hard right"
    elif L1 < 2000:
        return "solf left"
    elif R1 < 2000:
        return "solf right"

    elif M < 2000 and H > 500:
        return "straight fast"
    
    elif M < 2000 or H > 500:
        return "straight"

    elif L2 > 2000 and L1 > 2000 and M > 2000 and R1 > 2000 and R2 > 2000 and H < 500:
        return "explore"
    elif L2 > 2000 and L1 > 2000 and M > 2000 and R1 > 2000 and R2 > 2000 and H < 500:
        return "stop"
    else:
        return "stop"
    
def update_speed(case):
    
    """!
    Read input case, then decide ROMI yaw rate and linear speed then run DC_speed_cal() function below to find angular speeds of both wheels
    For the scope of this project, ROMI will have only 19 case including:
        
         CASE                   DESCRIPTION
         
        cross               reading the cross pattern    
        
        90° turn            90° turn left or right
        
        really hard turn    really hard turn left or right
        
        hard turn           hard turn left or right
        
        soft turn           soft turn left or right
        
        straight            going foward
        
        backup              going backward
        
        explore             exploring mode
        
        pivot               pivot left or right
        
        wall                turn to stay away from the wall
        
        stop                stop
        
        done                done and celebrate by pivoting

    @param case (string): input case that ROMI need to run.
    
    @return wL (float): left wheel angular velocity (rad/s).
    @return wR (float): right wheel angular velocity (rad/s).
    
    """
    
    v          = 0       
    v_stop     = 0
    v_pivot    = 0
    v_wall1    = 0.125
    v_wall2    = 0.225
    v_90       = 0.125
    v_rhard    = 0.1
    v_hard     = 0.125
    v_straight = 0.25
    v_straightf= 0.3
    v_solf     = 0.2
    v_done     = 0
    
    y          = 0  
    y_stop     = 0
    y_pivot    = 1
    y_wall     = 1.2
    y_straight = 0
    y_straightf= 0
    y_90       = 2.5
    y_rhard    = 2.25
    y_hard     = 2
    y_solf     = 1
    y_done     = 2
    
    if case == "cross":
        v = v_straight   
        y = y_straight
    elif case == "90 left":
        v = v_90
        y = y_90
    elif case == "90 right":
        v = v_90
        y = -y_90
    elif case == "really hard left":
        v = v_rhard
        y = y_rhard 
    elif case == "really hard right":
        v = v_rhard 
        y = -y_rhard
    elif case == "hard left":
        v = v_hard
        y = y_hard
    elif case == "hard right":
        v = v_hard
        y = -y_hard
    elif case == "solf left":
        v = v_solf
        y = y_solf
    elif case == "solf right":
        v = v_solf
        y = -y_solf
    elif case == "straight fast":
        v = v_straightf
        y = y_straightf
    elif case == "straight":
        v = v_straight
        y = y_straight
    elif case == "backup":
        v = -v_straight
        y = y_straight        
    elif case == "explore":
        v = v_straight
        y = y_straight
    elif case == "pivot left":
        v = v_pivot
        y = y_pivot
    elif case == "pivot right":
        v = v_pivot
        y = -y_pivot
    elif case == "wall1":
        v = v_wall1
        y = y_wall
    elif case == "wall2":
        v = v_wall2
        y = -y_wall
    elif case == "stop":
        v = v_stop
        y = y_stop
    elif case == "done":
        v = v_done
        y = y_done
    else:
        v = v_stop
        y = y_stop
        
    wR, wL = DC_speed_cal(y, v)
    return wL, wR

def DC_speed_cal(y, v):
    
    """!
    Take the require yaw rate and linear speed then calculate angular speeds for both left and right wheels

    @param y (float): require yaw rate (rad/s)
    @param v (float): require linear speed (m/s)
    
    @return wL (float): left wheel angular velocity (rad/s).
    @return wR (float): right wheel angular velocity (rad/s).
    
    """
    
    rW = 0.035
    rR = 0.075
    wL = (v - rR*y)/rW
    wR = (v + rR*y)/rW
    return wR, wL

def check_return(X, Y, state):
    
    """!
    Take the X and Y coordinate of ROMI location determine the output state for returning purpose
    
    @param X (float): X global coordinate (m)
    @param Y (float): Y global coordinate (m)
    
    @return state (float): FSM state for returning purpose 
    @return HOME (float): Home flag for return home status
    
    """
    
    RETURNX = 7 
    RETURNY = 8
    RETURN_HUB = 1
    HOME = 1
    NOT_YET = 0
        
    if abs(Y) > 0.02:
        return RETURNY, NOT_YET
    elif abs(X) > 0.02:
        return RETURNX, NOT_YET 
    else:
        return RETURN_HUB, HOME         
        