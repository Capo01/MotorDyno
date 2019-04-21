from __future__ import print_function
import odrive
from odrive.enums import *
import time
import math
import matplotlib.pyplot as plt
import statistics
import datetime

"""
Odrive based motor dyno and data logger V1.1
Requires custom odrive firmware. See fork: https://github.com/Capo01/ODrive
Designed to work with an external micro (Arduino Mega) making some measurements. 
See: https://github.com/Capo01/MotorDyno/blob/master/Arduino/Load_cell_moving_median/Load_cell_moving_median.ino
R. Parsons
17.04.19

How to use this script:
Set the name and type of text file you wish to output.
Text file will be placed in the same directory as this script.
Select the test type you wish to use and its parameters.
Run python script with motor controllers connected.

TO DO:  * Set-up second odrive and test motor readings.
        * Add actual readings of the motor thermistors rather than just place holders
        * Add checks for motor and board temps to make sure they don't get too hot.
        * Add motor parameters at start of file (motor resistance, inductance etc.)
        
"""

#output data text file set-up
text_file_name              = 'odrive_loss.csv' # Name of output text file
file_header_flag            = True 	# Set False to exclude the file header (name and unit at start of file)
erase_file_on_startup_flag  = True 	# Set True to erase text file at the start of each run

#logging parameters
stabilise_time              = 0.5   # Time elapsed after setting parameters before measuring values [s]
num_readings                = 10    # Number of readings to be averaged
sleep_time                  = 0.01  # Wait time between readings. Arduino Mega only outputs a reading every 10 ms [s]

#Odrive set-up
calibrate_on_startup        = False # motor current limit [A] 
idel_on_shutdown            = False # Encoder counts per rotation [CPR]
absorber_motor_current_lim  = 30    # motor current limit [A]
encoder_cpr                 = 8192  # Encoder counts per rotation [CPR]
test_brake_resistance       = 0.47  # resistance of the brake resistor connected to test motor odrive[Ohm]
absorber_brake_resistance   = 2.0   # resistance of the brake resistor connected to the absorber odrive[Ohm]
shutdown_ramp_speed         = 10E4 	# Speed to slow down at end of test [counts/s/s]

# Motor parameter reporting
text_file_name_motor_param	= 'Motor_parameter.csv' # Name of output text file

# No-load speed test
no_load_num_steps           = 70    # Number of steps to increment RPM and take a reading
no_load_speed_step          = 100   # Step size between readings [RPM]

# Motor controller power loss estimation
loss_est_num_steps			= 14	# Number of steps to increment motor current and take a reading
loss_est_current_step		= 5		# Current size between readings [A]
loss_est_max_motor_temp		= 30	# Maximum motor temperature that a reading will be made at.



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
    'location'  :   '(my_odrive.vbus_voltage)',
},
{
    'name'      :   'Input current',
    'unit'      :   'mA',
    'location'  :   '(my_odrive.axis0.muv2)',
},
{
    'name'      :   'Input Power',
    'unit'      :   'W',
    'location'  :   '(my_odrive.vbus_voltage * my_odrive.axis0.muv2 * 0.001)',
},
{
    'name'      :   'Test Motor Temperature',
    'unit'      :   'Deg C',
    'location'  :   '(my_odrive.axis0.muv3)',
},
{
    'name'      :   'Test motor FET temp',
    'unit'      :   'Deg C',
    'location'  :   '(my_odrive.axis0.motor.get_inverter_temp())',
},
{
    'name'      :   'Test motor q-axis current',
    'unit'      :   'A',
    'location'  :   '(my_odrive.axis0.motor.current_control.Iq_measured)',
},
{
    'name'      :   'Test motor d-axis current',
    'unit'      :   'A',
    'location'  :   '(my_odrive.axis0.motor.current_control.Id_measured)',
},
{
    'name'      :   'Output Torque',
    'unit'      :   'N.mm',
    'location'  :   'my_odrive.axis0.muv1',
},
{
    'name'      :   'Output Speed',
    'unit'      :   'rad.s',
    'location'  :   '((my_odrive.axis0.encoder.vel_estimate / 8192) * 2 * math.pi)',
},
{
    'name'      :   'Output Power',
    'unit'      :   'W',
    'location'  :   '((my_odrive.axis0.encoder.vel_estimate / 8192) * 2 * math.pi) * (my_odrive.axis0.muv1) * 0.001',
},
{
    'name'      :   'Output Efficiency',
    'unit'      :   '%',
    'location'  :   '((((my_odrive.axis0.encoder.vel_estimate / 8192) * 2 * math.pi) * (my_odrive.axis0.muv1) * 0.001) / (my_odrive.vbus_voltage * my_odrive.axis0.muv2) * 100)',
},
{
    'name'      :   'Absorber Motor Temperature',
    'unit'      :   'Deg C',
    'location'  :   '(my_odrive.axis0.muv4)',
},
{
    'name'      :   'Absorber motor FET temp',
    'unit'      :   'Deg C',
    'location'  :   '(my_odrive.axis0.motor.get_inverter_temp())',
},
{
    'name'      :   'Absorber q-axis current',
    'unit'      :   'A',
    'location'  :   '(my_odrive.axis0.motor.current_control.Iq_measured)',
},
{
    'name'      :   'Absorber d-axis current',
    'unit'      :   'A',
    'location'  :   '(my_odrive.axis0.motor.current_control.Id_measured)',
}
]

def measure_values():
    """
    Measures the values outlined in measurement_list.
    Takes multiple measurements as determined by num_readings.
    Averages the measurements and returns the data as a list.
    
    """
    data_list = []   
       
    # Measurement data to be stored in a new empty dictionary key called 'values'
    for i in range(len(measurement_list)):
        measurement_list[i]['value'] = []
    
    # Multiple measurements taken for averaging as per num_readings
    for x in range(num_readings):
        for i in range(len(measurement_list)):
                location = measurement_list[i]['location']
                measurement_list[i]['value'].append(eval(location))
                time.sleep(sleep_time) # odrive only outputs a reading every 10 ms
                
                    
    # Mean taken of measurements to filter out noise.
    for i in range(len(measurement_list)):
                data_list.append(statistics.mean(measurement_list[i]['value']))

    return data_list;

def write_values(data):
    """
    Takes measured data and writes it, and the header, to a csv file. 
    
    """
    global file_header_flag
    global erase_file_on_startup_flag
    name_list = []
    unit_list = []
    data_list = data
    
    # Add the measurement names and units to a list for the csv header.
    for i in range(len(measurement_list)):
        name_list.append(measurement_list[i]['name'])
        unit_list.append(measurement_list[i]['unit'])
       
    # Format the header data  
    with open(text_file_name,"a+") as text_file:
        name_list_formatted = ', '.join(str(e) for e in name_list) + '\n'
        unit_list_formatted = ', '.join(str(e) for e in unit_list) + '\n'
        data_list_formatted = ', '.join(str(e) for e in data_list) + '\n'
                            
        # Erase the contents of the text file from previous runs (need '0' when using r+ )
        if erase_file_on_startup_flag == True:
            text_file.truncate(0)
            erase_file_on_startup_flag = False 

        # Write header data to text file only once.
        if file_header_flag == True:
        	text_file.write(str(str(now))) # Add date and time to text file.
            text_file.write(str(name_list_formatted))
            text_file.write(str(unit_list_formatted))
            file_header_flag = False
        
        # Write the data
        text_file.write(str(data_list_formatted))
        
def odriveStartup():
    
    """
    Finds connected odrives and calibrates motors, sets closed loop control.
    """

    # Find a connected ODrive (this will block until you connect one)
    print("finding an odrive...")
    global my_odrive
    my_odrive = odrive.find_any()
    print("odrive found...")
    my_odrive.config.brake_resistance = absorber_brake_resistance

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

def odriveShutdown():
    
    """
    Stops motor and end of test, returns it to positional control mode.
    """
    print("Shutting down odrives")
    my_odrive.axis0.controller.vel_ramp_enable = True
    my_odrive.axis0.controller.config.vel_ramp_rate = shutdown_ramp_speed
    my_odrive.axis0.controller.vel_ramp_target = 0

    # wait for motors to slow down.
    while my_odrive.axis0.encoder.vel_estimate != 0:
        time.sleep(0.1)
    
    my_odrive.axis0.controller.vel_ramp_enable = False
    
    if idel_on_shutdown == True:
        # Calibrate motor and wait for it to finish
        my_odrive.axis0.controller.config.control_mode = 1

def rpm_to_cpr(x):
    
    """
    Converts motor RPM to counts/s for use by motor controller.
    """
    return (x / 60)* encoder_cpr;

def report_motor_parameters():

	"""
	Tests motor resistance and inductance and prints it to a separate text file.
	"""
	print("Testing motors...")
	my_odrive.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    while my_odrive.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    print("Testing complete...")   

	absorber_resistance = my_odrive.axis0.motor.config.phase_resistance
	absorber_inductance = my_odrive.axis0.motor.config.phase_inductance
	absorber_temperature = my_odrive.axis0.muv4
	print('Absorber resistance (Ohm): ', absorber_resistance)
	print('Absorber inductance (Henry): ', absorber_inductance)
	print('Absorber Temperature (Dec): ', absorber_temperature)
	print("Saving motor parameters to a text file...")

	with open(text_file_name_motor_param,"a+") as text_file:
		text_file.write(str(str(now))) # Add date and time to text file.
		text_file.write('Absorber resistance (Ohm) ,' + str(absorber_resistance))
		text_file.write('Absorber inductance (Henry) ,' + str(absorber_inductance))
		text_file.write('Absorber Temperature (Dec) , ' + str(absorber_temperature))
	
	print("Saving complete.")
     
def no_load_speed_test():
    
    """
    Sets motor to velocity control mode.
    Increments speed by speed_step up to no_load_num_steps * no_load_speed_step.
    Logs data to text file at each speed.
    """
    my_odrive.axis0.controller.config.control_mode = 2  #set to velocity control mode
    print("now in velocity control mode")
    print("Starting no-load speed test")

    for i in range(no_load_num_steps):
        max_speed = (no_load_num_steps * no_load_speed_step) # First step is zero.
        my_odrive.axis0.controller.config.vel_limit = rpm_to_cpr(max_speed) # [counts/s]
        set_speed = i * no_load_speed_step
        print(i, ' ', set_speed, ' ' , my_odrive.axis0.muv2)
        my_odrive.axis0.controller.vel_setpoint = rpm_to_cpr(set_speed) # [counts/s]
        time.sleep(stabilise_time) # wait for speed to stabilise [s]
        write_values(measure_values())

    print("No-load speed test complete")

def motor_controller_loss_estimation():
    """
    Used to estimate the losses produced by the motor controller (odrive).
    Sets motor to index search so that all phases experiences even heating and records values.
    Checks that motor temperature is within specified range, increments current limit and repeats.
    """
    for i in range(loss_est_num_steps)
    	my_odrive.axis0.motor.config.current_lim = i * loss_est_current_step
    	my_odrive.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    	time.sleep(stabilise_time)
    	write_values(measure_values())

    	# Wait for motor to cool down if it is too hot
    	# If the motor becomes too hot then its resistance will be very different to that measured at start-up.
    	while my_odrive.axis0.muv4 >= loss_est_max_motor_temp 
    		time.sleep(1)
    		print('Motor too hot. Motor Temp (DegC): ', my_odrive.axis0.muv4)


odriveStartup()
#no_load_speed_test()
motor_controller_loss_estimation()
odriveShutdown()
