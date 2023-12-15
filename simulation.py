'''! @page simulation Simulation
    @section sec_sim    Overview
                        This page encompasses all the simulations and data analyses conducted on ROMI. 
                        The simulation process includes using Python within a Jupyter Notebook to plot the desired path of ROMI. 
                        However, simulating animations in Python presents certain challenges. 
                        To address this, the animation aspect is tackled using MATLAB, which offers more convenience for this purpose.
                        For reference and further exploration, both the Jupyter Notebook file and the MATLAB file used in these simulations are available.
                        These files can be accessed through the provided GitHub link at https://github.com/vvinhvvo98/ME405  
                        The specific files are named `Animation.mlx` for the MATLAB animation and `HW0x03.ipynb` for the Jupyter Notebook simulation.
                        
    @section sec_sim1   Hand Calculation
                        Below, the hand calculations are provided again for convenience.
                        The objective of these hand calculations is to derive a state-space equation of motion, 
                        which is a system dynamics in generating ROMI's path based on the angular velocity inputs of both wheels. 
                        
                        @image html handcal2.png
                        @image html handcal1.png
 
    @section sec_sim2   Simulation 1
                        The simulation presented below, as shown in the figure, displays results from a Python Jupyter Notebook. 
                        It simulates ROMI following a predetermined path, specifically a circle with a radius of 0.6 m. 
                        In this simulation, ROMI is set to travel at a linear velocity of 0.25 meters per second. 
                        
                        @image html circular_path.png
                        
                        For this scenario, all the kinematic data plotted using Jupyter Notebook is also displayed below. 
                        This provides a comprehensive overview before proceeding to the animation segment. 
                        The plotted data offers a visual representation of ROMI's kinematic behaviors, laying a foundational understanding that complements the subsequent animation.
                        
                        Displacement Over Time:
                            
                        @image html plot_1c.png
                        
                        Velocity Over Time: 
            
                        @image html plot_2c.png
                        
                        Below is a small GIF showcasing a simulation where ROMI follows a circular path with a diameter of 0.6 inches, traveling at a speed of 0.25 meters per second.
                        This animation is generated using the MATLAB `Animation.mlx` file, visually representing ROMI's movement along the specified path as describe in the Jupyter Notebook figure above.
                        
                        @image html circular_path.gif
                        
    @section sec_sim3   Simulation 2
                        The simulation presented below, as shown in the figure, displays results from a Python Jupyter Notebook. 
                        In this simulation, ROMI's path will be determined by the following wheel angular velocities input profile:
                            
                            t(s)           Right Wheel Speed (rad/s)       Left Wheel Speed (rad/s)    
                              
                            0  -  1                  10                              15
                             
                            2  -  3                  20                              10
                            
                            4  -  5                  20                              15
                            
                            6  -  7                  20                              5
                            
                            7  -  8                  15                              5 
                            
                            8  -  9                  10                              20 
                            
                            9  - 10                  20                              5
                            
                            10 - 11                  5                               15
                                
                            11 - 12                  10                              10
                            
                            12 - 13                  5                               25
                            
                            13 - 14                  15                              15
                            
                            14 - 15                  15                              20
                            
                            else                     5                               5 
                            
                        @image html random_path.png
                        
                        For this scenario, all the kinematic data plotted using Jupyter Notebook is also displayed below. 
                        This provides a comprehensive overview before proceeding to the animation segment. 
                        The plotted data offers a visual representation of ROMI's kinematic behaviors, laying a foundational understanding that complements the subsequent animation.
                        
                        Displacement Over Time
                        
                        @image html plot_1r.png
                        
                        Velocity Over Time
                        
                        @image html plot_2r.png
                        
                        Below is a small GIF showcasing a simulation where ROMI follows the path in simulation 2 above with the same angular velocities profile
                        
                        @image html random_path.gif
                        
   @author              Vinh Vo
   @author              Quinlan Stephens
   @date                Dec 12, 2023
                        

'''