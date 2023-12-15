"""!
@file main.py
This file contain a the main code that run a Finite State Machine with multitasking for ROMI.

@author Vinh Vo
@author Quinn Stephens
@date   2023-Dec-11 
"""

import task_MOT
import task_SER
import task_IMU
import task_ULS
import task_share 
import cotask

def main():
    """!
    Main function to initialize and schedule tasks for controlling motors, servos, 
    IMU, and ultrasonic sensors. It uses a cooperative multitasking scheduler to 
    manage task execution.

    The function creates shared variables for communication between tasks, initializes
    each task with appropriate parameters, and adds them to the task scheduler. 
    It then enters an infinite loop to execute these tasks.
    
    The script run a multitask Finite State Machine as following:
        
        TASK    PRIORITY    PERIOD(ms)                                 DESCRIPTION
        MOT        4          1         This task run as a main task that control ROMI base on feedback from different sensor in other task 
        SER        1          2         This task run to control 2 servo, one is for ultrasonic sensor angle, and one for the blindfold   
        IMU        3          1         This task run to continuosly reading the corrected yaw angle from the IMU    
        ULS        2          2         This task run to continuosly reading the distance in the front of the ultrasonic sensor
        
    The MOT task and SER task share: SER_DIR    for the first servo that control the ultrasonic sensor
    The MOT task and SER task share: CLOSE      for the second motor that control a 3D printed blindfold
    The MOT task and ULS task share: ULS_DIS    for the sensor distance from the ultrasonic sensor to detect wall
    The MOT task and IMU task share: IMU_YAW    for ROMI updated yaw angle 
    """
    
    # Initialize shared variables for inter-task communication
    SER_DIR = task_share.Share('b', name = "Servo Direction")
    IMU_YAW = task_share.Share('f', name = "Romi's Yaw")
    ULS_DIS = task_share.Share('f', name = "Ultrasonic Distance")
    CLOSE   = task_share.Share('i', name = "Close Eye Servo")
    
    # Initialize tasks with their respective shared variables
    MOT_run   = task_MOT.MotorTask(SER_DIR, IMU_YAW, ULS_DIS, CLOSE)
    SER_run   = task_SER.ServoTask(SER_DIR, CLOSE)
    IMU_run   = task_IMU.IMUTask(IMU_YAW)
    ULS_run   = task_ULS.ULSTask(ULS_DIS)
    
    # Create cotask.Task objects for each task
    ULS       = cotask.Task(ULS_run.run, name='ULS_TASK' , priority=1, period=5)
    SER       = cotask.Task(SER_run.run, name='SER_TASK' , priority=2, period=5)
    MOT       = cotask.Task(MOT_run.run, name='MOT_TASK' , priority=4, period=1)
    IMU       = cotask.Task(IMU_run.run, name='IMU_TASK' , priority=3, period=1)
    
    # Create a task list and add tasks to it
    task_list = cotask.TaskList()
    task_list.append(MOT)
    task_list.append(SER)
    task_list.append(ULS)
    task_list.append(IMU)
    
    # Main loop to run tasks
    while True:     
        try:
            task_list.pri_sched()  # Schedule and run tasks
        except KeyboardInterrupt:
            print('PROGRAM TERMINATED')
            break
            
if __name__ == '__main__':
    main()
