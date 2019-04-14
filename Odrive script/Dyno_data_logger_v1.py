"""
Odrive based motor dyno and data logger V1
Requires custom odrive firmware. See fork: https://github.com/Capo01/ODrive
Designed to work with an external micro (arduino) making some measurements. 
See: https://github.com/Capo01/MotorDyno/blob/master/Arduino/Load_cell_moving_median/Load_cell_moving_median.ino
R. Parsons
14.04.19

How to use this script:
Set the name and type of text file you wish to output.
Text file will be placed in the same directory as this script.
Set the paratmers you wish to test to True.
Set the speed range, speed step size and 

TO DO:  * Setup second odrive and test motor readings.
        * Add actual readings of the motor thermistors rather than just placeholdes
        * Take an average of a couple of values before logging
        * Add checks for motor and board temps to make sure they don't get too hot.
        * Add readings for Iq and Id for each motor
        * Add motor parameters at start of file (motor resistance, inductance etc.)
        
"""

from __future__ import print_function
import odrive
from odrive.enums import *
import time
import math
import matplotlib.pyplot as plt

#Odrive setup
calibrate_on_startup        = False
absorber_motor_current_lim  = 30    # motor current limit [A]

#logging parameters
sleep_time                  = 1     # Time elapsed after setting parameters before measuring values [s]
speed_step                  = 100   # Step size between readings [RPM]
num_readings                = 30     # Number of readings to be averaged

#output text file setup
text_file_name              = 'no_load_test.csv' # Name of output text file
use_file_header             = True  # Include variable names and units in first line of text file.
file_header_flag            = False # Is set true after file header written to text file   

# Test motor and motor controller values to be measured.
measure_input_voltage       = True  # test motor motor-controller Vbus voltage odrive [V]
measure_input_current       = True  # test motor motor-controller input current as read by external sensor [mA]
measure_input_power         = True  # test motor motor-controller input power [W]
measure_test_motor_temp     = True  # test motor thermistor reading
measure_test_odrive_m0_temp = True  # test motor odrive M0 thermistor reading

# Absorber motor and motor controller values to be measured.
measure_output_torque       = True  # test motor torque output estimated from loadcell readings [N.mm)]
measure_output_speed        = True  # test motor speed estimated from encoder [rad.s]
measure_output_power        = True  # test motor power as estimated from torque x ang. vel [W]
measure_output_efficiency   = True  # test motor and motor controller efficiency [%]
measure_absorber_motor_temp = True  # absorber motor thermistor reading
measure_test_odrive_m0_temp = True  # absorber motor odrive M0 thermistor reading 
   
def odriveStartup():
    
    """
    Finds connected odrives and calibrates motors, sets closed loop control.
    """
    # Find a connected ODrive (this will block until you connect one)
    print("finding an odrive...")
    global my_odrive
    my_odrive = odrive.find_any()

    # Find an ODrive that is connected on the serial port /dev/ttyUSB0
    #my_drive = odrive.find_any("serial:/dev/ttyUSB0")

    if calibrate_on_startup == True:
            # Calibrate motor and wait for it to finish
        print("starting calibration...")
        my_odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while my_odrive.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.5)

    my_odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    my_odrive.axis0.motor.config.current_lim = absorber_motor_current_lim
    print("now in closed loop control")
    f = open(text_file_name, 'r+')
    f.truncate(0) # Erase the contents of the text file from previou runs (need '0' when using r+ )

def odriveShutdown():
    
    """
    Stops motor and end of test, returns it to positional control mode.
    """
    my_odrive.axis0.controller.vel_setpoint = 0
    my_odrive.axis0.controller.config.control_mode = 1

def readValues():
    
    """
    Reads those variables set to true.
    Returns header_list and unit_list for start of text file and measurement_list for output data.
    """
    header_list = []
    data_list = []
    unit_list = []
    
    if measure_input_voltage == True:
        header_list.append('Input Voltage')
        unit_list.append('V')
        input_voltage = (my_odrive.vbus_voltage)
        data_list.append(input_voltage)
    
    if measure_input_current == True:
        header_list.append('Input Current')
        unit_list.append('mA')
        input_current = (my_odrive.axis0.muv2) #NOTE: requires custom odrive firmware fork.
        data_list.append(input_current)
     
    if measure_input_power == True:
        header_list.append('Input Power')
        unit_list.append('W')
        input_power = (my_odrive.vbus_voltage * my_odrive.axis0.muv2 * 0.001)
        data_list.append(input_power)
        
    if measure_test_motor_temp == True:
        header_list.append('Test Motor Temperature')
        unit_list.append('Deg C')
        test_motor_temp = (my_odrive.axis0.muv3)
        data_list.append(test_motor_temp)
        
    if measure_test_odrive_m0_temp == True:
        header_list.append('Test Odrive M0 Temperature')
        unit_list.append('Deg C')
        test_odrive_m0_temp = (my_odrive.axis0.motor.get_inverter_temp())
        data_list.append(test_odrive_m0_temp)
           
    if measure_output_torque == True:
        header_list.append('Output Torque')
        unit_list.append('N.mm')
        output_torque = (my_odrive.axis0.muv1)
        data_list.append(output_torque)  
        
    if measure_output_speed == True:
        header_list.append('Output Speed')
        unit_list.append('rad.s')
        output_speed = ((my_odrive.axis0.encoder.vel_estimate / 8192) * 2 * math.pi)
        data_list.append(output_speed)          
     
    if measure_output_power == True:
        header_list.append('Output Power')
        unit_list.append('N.mm')
        output_power = ((my_odrive.axis0.encoder.vel_estimate / 8192) * 2 * math.pi) * (my_odrive.axis0.muv1) * 0.001
        data_list.append(output_power)   
        
    if measure_output_efficiency == True:
        header_list.append('Output Efficiency')
        unit_list.append('%')
        output_efficiency = ((((my_odrive.axis0.encoder.vel_estimate / 8192) * 2 * math.pi) * (my_odrive.axis0.muv1) * 0.001) / (my_odrive.vbus_voltage * my_odrive.axis0.muv2) * 100)
        data_list.append(output_efficiency)          

    if measure_absorber_motor_temp == True:
        header_list.append('Absorber Motor Temperature')
        unit_list.append('Deg C')
        absorber_motor_temp = (my_odrive.axis0.muv4)
        data_list.append(absorber_motor_temp)          

    if measure_test_odrive_m0_temp == True:
        header_list.append('Test Odrive M0 Temperature')
        unit_list.append('Deg C')
        test_odrive_m0_temp = (my_odrive.axis0.motor.get_inverter_temp())
        data_list.append(test_odrive_m0_temp)          
                        
        
    return [header_list, unit_list, data_list];

def writeValues(data):
    
    """
    Reads data and writes it to a file.
    """
    global file_header_flag
    header_list = data[0]
    unit_list = data[1]
    data_list = data[2]
    
    with open(text_file_name,"a") as text_file:
        header_list_formatted = ', '.join(str(e) for e in header_list) + '\n'
        unit_list_formatted = ', '.join(str(e) for e in unit_list) + '\n'
        data_list_formatted = ', '.join(str(e) for e in data_list) + '\n'
                        
        if file_header_flag == False:
            text_file.write(str(header_list_formatted))
            text_file.write(str(unit_list_formatted))
        
        text_file.write(str(data_list_formatted))
        file_header_flag = True

def no_load_speed_test():
    
    """
    Sets motor to velocity control mode.
    Incriments speed by speed_step up to num_reading * speed_step.
    Logs data to text file at each speed.
    """

    my_odrive.axis0.controller.config.control_mode = 2
    print("now in velocity control mode")

    for i in range(num_readings):
        my_odrive.axis0.controller.config.vel_limit = ((num_readings * speed_step) / 60) *8192 # [counts/s]
        set_speed = i * speed_step
        my_odrive.axis0.controller.vel_setpoint =  (set_speed / 60) * 8192 # [counts/s]
        print(set_speed)
        time.sleep(sleep_time) # wait for speed to stabalise [s]
        data = readValues()
        writeValues(data)

#Test procedure to be run.
odriveStartup()
no_load_speed_test()
odriveShutdown()