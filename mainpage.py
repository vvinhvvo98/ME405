'''!@file                mainpage.py
    @brief               ROMI SEARCH & RESCUE TERM PROJECT
    @details             This porfolio page will be documenting everything related to Mechatronics ME_405 Term Project
                         include coding work, electronics, and logic behind the robot
                         Please feel free to explore the file and email me at vinhvo.career@gmail.com if you have any question

    @mainpage
    

    @section sec_1       OBJECTIVE
                         The objective of this project is to design and build a robot that operates autonomously. 
                         This robot will start from a designated 'Home' position 
                         with its primary task is to follow multiple pattern paths marked with black lines that are 0.75 inches wide. 
                         In the course of its journey, the robot will encounter and need to navigate over or around a wall-like obstacle. 
                         Once it reaches a predetermined spot known as the 'Target', it should be able to stop
                         After reaching the Target, the robot's next challenge is to find its way back to the original path it left. 
                         To help visualize and plan for this, a detailed diagram showing the layout of the paths and the location of the Home and Target positions is provided for reference. 
                         This diagram is crucial for understanding the robot's intended route and the obstacles it will face.
                         
                         @image html path.png
                         
    @section sec_2       HARDWARE
                         The first step in our approach involves determining the hardware configuration for the robot. 
                         To improve the process, we opted for a 2-wheel drive design, which offers simplicity and efficiency. 
                         Considering the time constraints of the project, we decided to utilize a ROMI kit from Pololu. 
                         This decision significantly cuts down on the time required for mechanical CAD design and the manufacturing of the chassis. 
                         The ROMI kit includes a circular chassis equipped with built-in 9V DC motors, drivers, and encoders. 
                         This choice allows us to kickstart the project effectively by minimizing the time typically spent on hardware development.

                         @image html ROMI_kit.png

                         Despite this, there remains a need for some CAD modeling to properly integrate the microcontroller and the necessary sensors to achieve the project's objectives. 
                         This CAD modeling will involve designing parts that are essential for sensor integration and overall functionality of the robot. 
                         The specific parts and detailed descriptions of these components can be found in the reference section linked below, under @ref hardware, where we delve into the finer details of the hardware aspect.
                         
    @section sec_3       FIRMWARE
                         For the educational objectives of the Mechatronics ME 405 course, the selected microcontroller is the Nucleo STM32, which is complemented by an extension board known as the "Shoe of Brian." 
                         This extension board, provided by Lecturer Charlie Refvem, enables the use of the MicroPython programming language with the Nucleo STM32, offering an alternative to the conventional C language. 
                         The Nucleo STM32 is readily available for purchase on platforms like Amazon at an affordable price, making it accessible for students. 
                         However, the "Shoe of Brian" extension board is specially designed and developed exclusively for the ME 405 course by of Dr. Ridgely.
                         Figure below is the Nucleo STM32 microcontroller, integrated with its specialized extension, the "Shoe of Brian" board. 
                         
                         @image html Nucleo.png
                             
                         More details about how the electronic parts are connected and designed will be given in the @ref firmware section. 
                         This part of the document will clearly explain how all the components work together.
                         
    @section sec_4      SIMULATION
                        The simulation aspect of this project involves creating a model and manually deriving the equations of motion for ROMI. 
                        Two solvers, the Euler solver and the RK4 solver, are used to predict and control ROMI's path based on the velocities of both wheels. 
                        The simulation considers two approaches: the first involves selecting the turning radius and speed of ROMI, while the second focuses on determining the wheel speeds as a function of time.
                        
                        These simulations are then integrated into either Jupyter Notebook or MATLAB with detailed results presented in the @ref simulation section. 
                        However, understanding the underlying physics and geometry of these equations can be challenging. 
                        A figure is provided below, which illustrates the hand-derived model of ROMI in a state-space vector format, complete with explanations of all parameters and their dimensions.
                         
                        @image html handcal2.png
                        @image html handcal1.png
    
    @section sec_5      FINAL DESIGN
                        The final CAD design of the ROMI robot includes an array of components: a digital line sensor module with five channels, an analog line sensor module, an ultrasonic distance sensor, two DC motors each equipped with built-in encoders, and two micro servos. 
                        The complete assembly of ROMI, representing the culmination of the term project, is depicted in the figure below. 
                        This design is current as of December 13, 2023. Please note that any further updates beyond this date will not be included in this portfolio due to time constraints.
                        
                        @image html assy.png 
      
    @section sec_6      TASK DIAGRAM
                        The robot will concurrently execute four tasks, as outlined in the Task Diagram below. 
                        This diagram specifies the duration (in ms) and relative priority of each task. 
                        These tasks will be scheduled according to their assigned priorities and periods using a scheduler script, the code for which has been provided by Lecturer Charlie Refvem.
                        
                        Note: All diagrams are listed and explain in detail at @ref diagram for futher more description
                        
                        @image html tasks_diagram.png
                        
    @section sec_7      MILESTONES DEMO
                        Below are four consecutive milestones that are overcome to complete the project as describe in the objective section. ROMI should first be able to follow any type of black line that is drawn, avoid a random wall obstacle at any point, stop at the predetermined target, the finally return at the Home position
                        Furthermore about the logic behind the performance, please visit the Task Diagram as well as the Finite State Machine Diagram at @ref diagram, also state diagram can approach at the task section below:
                            
                        task_MOT.py - Control ROMI motor base on input share from different task  
                        
                        task_IMU.py - Rrun BNO055 IMU Sensor and processing the data
                        
                        task_ULS.py - Managing Ultrasonic Sensor readings in a state machine manner
                        
                        task_SER.py - Control servo motors based on external signals and states
                            
                        
                        
                        Line Following Demo
                        
                        This demonstration focus the consistency of the line-following function and the ROMI's capability to recognize the absence of a line signal. 
                        When no line is detected, the ROMI will slow down in explore mode. 
                        This function marks the first milestone of the whole project
                        
                        @image html LINEDEMO.gif
                        
                        Wall Obstacle Avoiding Demo
                        
                        This demonstration focus on the capability of avoiding wall obstacles.
                        As illustrated in the small GIF figure below, ROMI is able to adapt to obstacles at any size and will return back the track as soon as the ultrasonic distance sensor can not detect the wall anymore.
                        This design will make sure ROMI can getaway from any obstacle size and loction not just the praticing obstacle in the lab.
                        This function marks the second milestone of the whole project
                        
                        @image html WALLDEMO.gif
                        
                        Stop at Target Demo
                        
                        This demonstration focus on the ability of stop at the Target after avoiding the obstacle. When no line is detected, the ROMI will slow down in explore mode and keep track of the traveled distance since its translate in to explore mode.
                        If the distance is greater than 0.15 [m] or 15 [cm], ROMI will stop and notice that as the Target.
                        This function marks the third milestone of the whole project
                        
                        @image html GOALDEMO.gif
                        
                        Return Home Demo
                        
                        This demonstration focus on the ability of returning Home after reaching the Target. By collecting the IMU yaw corrected yaw angle and the distance travel of very cycle of task, ROMI is able to keep track of X and Y global cordinate which is exactly the same
                        cordinate from the hand derivation of the previous sectio that can be found at @ref simulation. Then the returning goal can be breakdown to 2 distingush state, return y until y ~ 0 [m] and then return x until x ~ 0 [m]. 
                        This function marks the final milestone of the whole project
                        
                        @image html RETURNDEMO.gif
                        
    @section sec_8      PRACTICE FULLCOURSE DEMONSTRATION
                        As detailed in @ref sec_1 Section, the integration of all milestones archived in  @ref sec_7 is more than sufficient to enable the autonomous two-wheeled robot ROMi to successfully complete the practice course outlined below:
                        
                        The final run result in ROMI is able to complete the whole course and return to the startnig location in [33 seconds] total.
                        
                        Note: Real-time video URL link is accessible in @ref sec_yout
                        
                        @image html fullcourse.gif
    
    @section sec_yout   YOUTUBE REFERENCE
                        All videos including all the fail and successful run can be reference and accessible in the following URL links:
                            
                        All Videos:                     https://www.youtube.com/channel/UCh_4F4CJVqvAhHmCMTvIb-w
                        
                        Practice Course v1:             https://www.youtube.com/shorts/hcw71OoOBFM
                        
                        Practice Course v2:             https://www.youtube.com/shorts/IiCvUNNVnG8
                        
                        Final Demonstration Course:     https://www.youtube.com/watch?v=OuhY6JEWlXs
                        
                        Disassembly:                    https://www.youtube.com/watch?v=cobgG8Q9dXY

                        
    @section sec_repo   REPOSITORY REFERENCE
                        All code that will be referenced in this portfolio relate to ROMI project
                        is accessible through https://github.com/vvinhvvo98/ME_405
                        
                        However, you may find it more useful to read through
                        the labs before looking around there. 
                       
   @author              Vinh Vo
   @author              Quinlan Stephens
   @date                Dec 12, 2023
'''