import statistics
import random
# =============================================================================
# from __future__ import print_function
# import odrive
# from odrive.enums import *
# import time
# import math
# import matplotlib.pyplot as plt
# 
# =============================================================================

"""
Odrive based motor dyno and data logger V1.1
Requires custom odrive firmware. See fork: https://github.com/Capo01/ODrive
Designed to work with an external micro (arduino) making some measurements. 
See: https://github.com/Capo01/MotorDyno/blob/master/Arduino/Load_cell_moving_median/Load_cell_moving_median.ino
R. Parsons
17.04.19

How to use this script:
Set the name and type of text file you wish to output.
Text file will be placed in the same directory as this script.
Set the paratmers you wish to test to True.
Set the speed range, speed step size and 

TO DO:  * Setup second odrive and test motor readings.
        * Add actual readings of the motor thermistors rather than just placeholdes
        * Add checks for motor and board temps to make sure they don't get too hot.
        * Add motor parameters at start of file (motor resistance, inductance etc.)
        
"""

#output text file setup
text_file_name              = 'pythontest.csv' # Name of output text file
file_header_flag            = False # Set true to exclude the file

#logging parameters
stabilise_time              = 1     # Time elapsed after setting parameters before measuring values [s]
speed_step                  = 100   # Step size between readings [RPM]
num_readings                = 4     # Number of readings to be averaged
delay_time                  = 12    # Wait time between readings [ms]

#Odrive setup
calibrate_on_startup        = False
absorber_motor_current_lim  = 30    # motor current limit [A]
encoder_cpr                 = 8192 # Encoder counts per rotation [CPR]

test_brake_resistance
absorber_brake_resistance



"""
Test motor and motor controller measurement list.
Each individual measurements has a name, unit and location
which is placed into a dictionary.
Comment out those measurements you don't need.

Example:
    
    'name'      :   'Input voltage', <---- Name printed in header of text file
    'unit'      :   'V',             <---- Unit printed in header of text file
    'location'  :   'my_odrive.vbus_voltage', <--- Value to measure from odrive

"""

measurement_list = [
{
    'name'      :   'Input voltage',
    'unit'      :   'V',
    'location'  :   'random.randrange(100)'#'(my_odrive.vbus_voltage)',
},
{
    'name'      :   'Input current',
    'unit'      :   'mA',
    'location'  :   'random.randrange(100)'# '(my_odrive.axis0.muv2)',
},
{
    'name'      :   'Input Power',
    'unit'      :   'W',
    'location'  :   'random.randrange(100)'# '(my_odrive.vbus_voltage * my_odrive.axis0.muv2 * 0.001)',
},
{
    'name'      :   'Test Motor Temperature',
    'unit'      :   'Deg C',
    'location'  :   'random.randrange(100)'# '(my_odrive.axis0.muv3)',
},
{
    'name'      :   'Test motor FET temp',
    'unit'      :   'Deg C',
    'location'  :   'random.randrange(100)'# '(my_odrive.axis0.motor.get_inverter_temp())',
},
{
    'name'      :   'Test motor q-axis current',
    'unit'      :   'A',
    'location'  :   'random.randrange(100)'# '(my_odrive.axis0.motor.current_control.Iq_measured)',
},
{
    'name'      :   'Test motor d-axis current',
    'unit'      :   'A',
    'location'  :   'random.randrange(100)'# '(my_odrive.axis0.motor.current_control.Id_measured)',
},
{
    'name'      :   'Output Torque',
    'unit'      :   'N.mm',
    'location'  :   'random.randrange(100)'# 'my_odrive.axis0.muv2',
},
{
    'name'      :   'Output Speed',
    'unit'      :   'rad.s',
    'location'  :   'random.randrange(100)'# '((my_odrive.axis0.encoder.vel_estimate / 8192) * 2 * math.pi)',
},
{
    'name'      :   'Output Power',
    'unit'      :   'W',
    'location'  :   'random.randrange(100)'# '((my_odrive.axis0.encoder.vel_estimate / 8192) * 2 * math.pi) * (my_odrive.axis0.muv1) * 0.001',
},
{
    'name'      :   'Output Efficiency',
    'unit'      :   '%',
    'location'  :   'random.randrange(100)'# '((((my_odrive.axis0.encoder.vel_estimate / 8192) * 2 * math.pi) * (my_odrive.axis0.muv1) * 0.001) / (my_odrive.vbus_voltage * my_odrive.axis0.muv2) * 100)',
},
{
    'name'      :   'Absorber Motor Temperature',
    'unit'      :   'Deg C',
    'location'  :   'random.randrange(100)'# '(my_odrive.axis0.muv4)',
},
{
    'name'      :   'Absorber motor FET temp',
    'unit'      :   'Deg C',
    'location'  :   'random.randrange(100)'# '(my_odrive.axis0.motor.get_inverter_temp())',
},
{
    'name'      :   'Absorber q-axis current',
    'unit'      :   'A',
    'location'  :   'random.randrange(100)'# '(my_odrive.axis0.motor.current_control.Iq_measured)',
},
{
    'name'      :   'Absorber d-axis current',
    'unit'      :   'A',
    'location'  :   'random.randrange(100)'# '(my_odrive.axis0.motor.current_control.Id_measured)',
}
]

def measure_values():
    """
    Measures the values outlined in measurement_list.
    Takes multipule measurements as determiened by num_readings.
    Averages the measurements and returns the data as a list.
    
    """
    data_list = []   
       
    # Measurement data to be stored in 'values'
    for i in range(len(measurement_list)):
        measurement_list[i]['value'] = []
    
    # Multipule measurements taken for averaging as per num_readings
    for x in range(num_readings):
        for i in range(len(measurement_list)):
                location = measurement_list[i]['location']
                measurement_list[i]['value'].append(eval(location))
                    
    # Mean taken of measurements to filter out noise.
    for i in range(len(measurement_list)):
                data_list.append(statistics.mean(measurement_list[i]['value']))

    return data_list;

def write_values(data):
    """
    Takes measured data and writes it, and the header, to a csv file. 
    
    """
    global file_header_flag
    name_list = []
    unit_list = []
    data_list = data
    
    # Add the measurement names and units to a list for the csv header.
    for i in range(len(measurement_list)):
        name_list.append(measurement_list[i]['name'])
        unit_list.append(measurement_list[i]['unit'])
       
    # Format the header data and write to file only once.    
    with open(text_file_name,"w") as text_file:
        name_list_formatted = ', '.join(str(e) for e in name_list) + '\n'
        unit_list_formatted = ', '.join(str(e) for e in unit_list) + '\n'
        data_list_formatted = ', '.join(str(e) for e in data_list) + '\n'
                            
        if file_header_flag == False:
            text_file.write(str(name_list_formatted))
            text_file.write(str(unit_list_formatted))
            file_header_flag = True
        
        # Write the data
        text_file.write(str(data_list_formatted))
        
# =============================================================================
# def odriveStartup():
#     
#     """
#     Finds connected odrives and calibrates motors, sets closed loop control.
#     """
#     # Find a connected ODrive (this will block until you connect one)
#     print("finding an odrive...")
#     global my_odrive
#     my_odrive = odrive.find_any()
# 
#     # Find an ODrive that is connected on the serial port /dev/ttyUSB0
#     #my_drive = odrive.find_any("serial:/dev/ttyUSB0")
# 
#     if calibrate_on_startup == True:
#             # Calibrate motor and wait for it to finish
#         print("starting calibration...")
#         my_odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
#         while my_odrive.axis0.current_state != AXIS_STATE_IDLE:
#             time.sleep(0.5)
# 
#     my_odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
#     my_odrive.axis0.motor.config.current_lim = absorber_motor_current_lim
#     print("now in closed loop control")
# =============================================================================


# =============================================================================
# def odriveShutdown():
#     
#     """
#     Stops motor and end of test, returns it to positional control mode.
#     """
#     my_odrive.axis0.controller.vel_setpoint = 0
#     my_odrive.axis0.controller.config.control_mode = 1
# =============================================================================

def rpm_to_cpr(x):
    
    """
    Converts motor RPM to counts/s for use by motor controller.
    """
    cpr = (x / 60)* encoder_cpr
    return cpr;

def no_load_speed_test():
    
    """
    Sets motor to velocity control mode.
    Incriments speed by speed_step up to num_reading * speed_step.
    Logs data to text file at each speed.
    """

    my_odrive.axis0.controller.config.control_mode = 2
    print("now in velocity control mode")

    for i in range(num_readings):
        my_odrive.axis0.controller.config.vel_limit = rpm_to_cpr(num_readings * speed_step) # [counts/s]
        set_speed = i * speed_step
        my_odrive.axis0.controller.vel_setpoint =  rpm_to_cpr(set_speed) # [counts/s]
        print(set_speed)
        time.sleep(sleep_time) # wait for speed to stabalise [s]
        writevalues(measure_values())

odriveStartup()
no_load_speed_test()
odriveShutdown()
