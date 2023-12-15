'''! @page firmware Electronic & Firmware
    @section sec_o      Overview
                        To effectively follow the line and avoid wall obstacles, the chosen sensors represent a balance of affordability and quality. 
                        These options are selected to ensure reliable performance without excessive cost.
                             
                        1. 1 x OSOYOO Infrared Line Tracking Sensor Module
                             
                        2. 1 x TCRT5000 Infrared Reflective Sensor
                             
                        3. 1 x HC-SR04 Ultrasonic Distance Sensor Module
                         
                        4. 1 x BNO055 Inertial Measurement Unit (IMU)
                             
                        5. 2 x Pololu Encoder
                        
                        Additionally, the robot requires a total of four actuators, which include the following components:
                        
                        1. 2 x Pololu DC motor
                        
                        2. 2 x MG90S DC Servo
                        
                        
                         
    @section sec_lin    OSOYOO Infrared Line Tracking Sensor
                        Description: This sensor features a total of seven digital output pins, 
                        which include two pins dedicated to a 5V power supply and five channels for line tracking, 
                        with each channel equipped to sense the track the black line beneath it. 
                        
                        Connection: 
                            
                        VCC_______________5V
                            
                        GND_______________GND
                            
                        IR1_______________PC0
                            
                        IR2_______________PA7
                            
                        IR3_______________PA6
                            
                        IR4_______________PA5
                            
                        IR5_______________PC1

                        @image html linesensor.png
                                                 
    @section sec_lin1   TCRT5000 Infrared Reflective Sensor Sensor
                        Description: This sensor features a total of 1 digital and 1 analog output pins, 
                        which include two pins dedicated to a 5V power supply. In this project, the analog output signal will be used
                        
                        Connection: 
                            
                        VCC_______________5V
                            
                        GND_______________GND
                            
                        A0________________PA4
                            
                        D0________________NC
                            
                        @image html linesensor2.png 
                        
    @section sec_uls    HC-SR04 Ultrasonic Distance Sensor
                        Description: This sensor features a total of 1 trigger pin as a digital output pin, and 1 echo pin as a digital input pin.
                        By measuring the timespan of the signal its transmitter and reciever, ROMI can sense the distance of the front object. 
                        Also the connectioninclude two pins dedicated to a 5V power supply.
                        
                        Connection: 
                            
                        VCC_______________5V
                            
                        GND_______________GND
                            
                        TRIG______________PB5
                            
                        ECHO______________PB4
                        
                        @image html uls.png
                                               
    @section sec_imu    BNO055 Inerial Measurement Unit(IMU)
                        Description: The IMU sensor in this project is capable of measuring the robot ROMI's tilt (Euler angles) and its linear and angular speeds. 
                        However, for guiding ROMI back, only the yaw angle from the Euler angle measurements is utilized. 
                        This sensor is equipped with six pins: two for 5V power, two for I2C bus communication (SCL and SDA), 
                        one providing a 3.3V output, and one for reset pin.
                        
                        Connection: 
                            
                        Vin_______________5V
                            
                        GND_______________GND
                            
                        SCL_______________PB8  (BUS 1 _ SCL)
                            
                        SDA_______________PC9  (BUS 1 _ SDA)
                        
                        3V________________NC
                        
                        RST_______________NC
                            
                        @image html imu_sensor.png

    @section sec_enc    POLOLU Encoders 
                        The encoder, serving as a sensor for this project, is typically bundled with the DC gear motor from Pololu, although it can also be purchased separately. 
                        It operates within a voltage range of 3.5V to 18V and features a Count Per Revolution (CPR) of 12. 
                        This encoder is designed with two pins, one for each of its channels, A and B. 
                        These pins are connected to the microcontroller unit (MCU) timer pins, which periodically read the counter numbers to track the motor's rotation.
                        
                        Connection: 
                            
                        ChA_______________PA0 (L) & PA8 (R)
                            
                        ChB_______________PA1 (L) & PA9 (R)
                            
                        @image html encoder.png
                        
    @section sec_dc     POLOLU Gear DC Motor 
                        Description: The DC motor used in this project, typically sold with an encoder from Pololu, can also be bought separately. 
                        It's designed to work at 9V for this specific kit. 
                        The encoder has three pins: two for controlling the motor's direction and on/off status, which connect to the microcontroller unit's (MCU) output pins, and a third pin for adjusting the motor's speed. 
                        This last pin connects to a PWM (Pulse Width Modulation) timer channel on the MCU, which controls the speed by changing the PWM signal's duty cycle.
                        
                        Connection: 
                        
                        SLP_______________PA2 (L) & PB3 (R)
                        
                        DIR_______________PA10 (L) & PB10 (R)
                        
                        PWM_______________PB7 (L) & B6 (R)
                            
                            
                        @image html motor.png
                        
    @section sec_ser    MG90S Micro Servo 
                        Description: This servo works very similar the SG90 microservo, but it has more torque using internal metal gear set.
                        The servo has total three pins that including two pins that dedicated to a 5V power supply, and one PWM signal pin to control the angle.
                        The servo is capable of moving at any angle between -90° to 90°
                        
                        Connection: 
                            
                        VCC________________5V
                            
                        GND________________GND
                            
                        SIGN_______________PB0  (Servo 1)
                            
                        SIGN_______________PC7  (Servo 2)
                            
                        @image html servo.png
                         
   @author              Vinh Vo
   @author              Quinlan Stephens
   @date                Dec 12, 2023
'''