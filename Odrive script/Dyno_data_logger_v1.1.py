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
        * Add checks to see if thermistor readings make sense (too large or too small).
        * Motor efficiency test function        
"""

#output data text file set-up
text_file_name              = 'odrive_loss.csv' # Name of output text file
file_header_flag            = True 	            # Set False to exclude the file header (name and unit at start of file)
erase_file_on_startup_flag  = True              # Set True to erase text file at the start of each run or between tests

#logging parameters
stabilise_time              = 1                 # Time elapsed after setting parameters before measuring values [s]
num_readings                = 10                # Number of readings to be averaged
sleep_time                  = 0.01              # Wait time between readings. Arduino Mega only outputs a reading every 10 ms [s]

#Odrive set-up
calibrate_on_startup        = False             # motor current limit [A] 
idel_on_shutdown            = False             # Encoder counts per rotation [CPR]
calibration_current         = 40                # Current used during calibration
absorber_motor_current_lim  = 30                # motor current limit [A]
encoder_cpr                 = 8192              # Encoder counts per rotation [CPR]
test_brake_resistance       = 0.47              # resistance of the brake resistor connected to test motor odrive[Ohm]
absorber_brake_resistance   = 2.0               # resistance of the brake resistor connected to the absorber odrive[Ohm]
shutdown_ramp_speed         = 1E6 	            # Speed to slow down at end of test [counts/s/s]
max_safe_temp               = 80                # Maximum allowable temperature for motor and MOSFETs [Deg C]
emergency_stop_flag         = False             # Set true if motor or controller temp goes outside set limits.

# Motor parameter reporting
text_file_name_motor_param	= 'Motor_parameter.csv' # Name of output text file

# No-load speed test
no_load_max_motor_speed     = 4000              # Maximum safe speed for motor. Note: Check encoder maximum RPM settings [RPM]
no_load_speed_step          = 1000              # Step size between readings [RPM]
no_load_max_vel_error       = 2                 # Maximum allowable error between true motor speed and commanded speed [%]

# Motor controller power loss estimation
loss_est_num_steps			= 14	            # Number of steps to increment motor current and take a reading
loss_est_current_step		= 5		            # Current size between readings [A]
loss_est_max_motor_temp		= 30	            # Maximum motor temperature that a reading will be made at.
loss_est_motor_under_test   = 0                 # Motor to be tested. 0 = absorber, 1 = test motor.

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
    'location'  :   '(motor_temp(1))',
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
    'name'      :   'Test motor Ical',
    'unit'      :   'A',
    'location'  :   '(my_odrive.axis0.motor.config.calibration_current)',
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
    'name'      :   'Output Speed set-point',
    'unit'      :   '%',
    'location'  :   '(my_odrive.axis0.controller.vel_setpoint)',
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
    'location'  :   '(motor_temp(0))',
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
},
{
    'name'      :   'Absorber motor Ical',
    'unit'      :   'A',
    'location'  :   '(my_odrive.axis0.motor.config.calibration_current)',
}
]

def rpm_to_cpr(x):
    
    """
    Converts motor RPM to counts/s for use by motor controller.
    """
    return (x / 60)* encoder_cpr;

def temp_check():
    """
    Compares motor and MOSFET temperatures to max_safe_temp.
    If exceeded, measurement stopped, motor speed reduced to zero, controller made idle and an error printed to console.
    """
    global emergency_stop_flag
    
    if my_odrive.axis0.muv3 >= max_safe_temp or my_odrive.axis0.motor.get_inverter_temp() >= max_safe_temp:
        emergency_stop_flag = True
        print('Warning: Maximum safe temperature (', max_safe_temp, ' Deg C) exceeded.')
        print('Motor Temperature (Deg C): ', my_odrive.axis0.muv3 )
        print('MOSFET Temperature (Deg C): ', my_odrive.axis0.motor.get_inverter_temp())
        odriveShutdown()

    return

def motor_temp(x):
    """
    Reads motor temperature thermistor which is connected to Odrive's analog input pins.
    Motors sold by Odrive have inbuilt thermistor thermally connected to motor windings.
    These can be used to monitor motor temperature.
    Note: get_adc_voltage() returns a voltage.
    """
    # Odrive thermistor parameters: NTC (10k 1% 3435)
    
    voltage_reference       = 3.3   # voltage used for thermistor readings.
    thermistor_nominal_val  = 1E4   # Thermistor resistance [Ohm]
    thermistor_coefficient  = 3435  # Thermistor coefficient
    thermistor_r0           = 25    # Temperature at which thermistor has its nominal resistance [deg C]
    series_resistor_val     = 1E4   # Series resistor used in thermistor voltage divider [Ohm]

    absorber_motor_reading  = my_odrive.get_adc_voltage(3) # Read absorber motor ADC
    test_motor_reading      = my_odrive.get_adc_voltage(4) # Read test motor ADC

    absorber_motor_temp_reading = ((1/ ((math.log10(((series_resistor_val / (voltage_reference  / absorber_motor_reading - 1))/thermistor_nominal_val))/thermistor_coefficient) + 1.0 / (25 + 273.15))) - 273.15)
    test_motor_temp_reading = ((1/ ((math.log10(((series_resistor_val / (voltage_reference  / test_motor_reading - 1))/thermistor_nominal_val))/thermistor_coefficient) + 1.0 / (25 + 273.15))) - 273.15)
    
    if x == 0:
        return absorber_motor_temp_reading;

    if x == 1:
        return test_motor_temp_reading;

    print('Motor temp error: incorrect motor name entered.')
    return;


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
            text_file.write(str(datetime.datetime.now()) + '\n') # Add date and time to text file.
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
    my_odrive.axis0.motor.config.calibration_current = calibration_current
    temp_check() # Check all temperatures are safe.

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
    Stops motor and end of test, sets motor to positional control mode.
    """
    print("Shutting down odrives")
    my_odrive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    my_odrive.axis0.controller.vel_ramp_enable = True
    my_odrive.axis0.controller.config.vel_ramp_rate = shutdown_ramp_speed
    my_odrive.axis0.controller.vel_ramp_target = 0

    # wait for motors to slow down.
    while my_odrive.axis0.encoder.vel_estimate != 0:
        time.sleep(0.5)
    
    my_odrive.axis0.controller.vel_ramp_enable = False
    # Prevent motor being set to position zero at full speed when set to position control mode.
    print("Setting to position control mode")
    my_odrive.axis0.controller.pos_setpoint = my_odrive.axis0.encoder.pos_estimate 
    my_odrive.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    
    if idel_on_shutdown == True or emergency_stop_flag == True:
        # Calibrate motor and wait for it to finish
        my_odrive.axis0.requested_state = AXIS_STATE_IDLE

    print("Shut down complete.")

def report_motor_parameters():
    """
	Tests motor resistance and inductance and prints it to a separate text file.
    Note: Choice of calibration current will impact motor resistance value.
    A larger calibration current will tend to be more accurate.
	"""
    temp_check() # Check all temperatures are safe.
    print("Testing motors...")
    my_odrive.axis0.motor.config.calibration_current = calibration_current
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
        text_file.write(str(datetime.datetime.now()) + '\n') # Add date and time to text file.
        text_file.write('Absorber resistance (Ohm) ,' + str(absorber_resistance) + '\n')
        text_file.write('Absorber inductance (Henry) ,' + str(absorber_inductance) + '\n')
        text_file.write('Absorber Temperature (Dec) ,' + str(absorber_temperature) + '\n')

    print("Saving complete.")
     
def no_load_speed_test():
    
    """
    Estimates maximum motor speed and motor + motor controller no-load power loss.
    Increments motor speed until allowable_speed_error is exceeded.
    Logs data for each no_load_speed_step
    """
    my_odrive.axis0.controller.config.vel_limit = rpm_to_cpr(no_load_max_motor_speed) # Limit motor top speed [counts/s]
    my_odrive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL  # set to velocity control mode
    my_odrive.axis0.controller.vel_setpoint = 0 # Stop motor if it isn't already stopped [counts/s]
    while my_odrive.axis0.encoder.vel_estimate != 0:# wait for motors to slow down.
        time.sleep(0.1)

    print("Starting no-load speed test")
    print("Maximum allowable speed error (%): ", no_load_max_vel_error)
    print('Increment, Set Speed (RPM), vel_error (%), Input Power (W)') 
    
    vel_error = 0.0
    no_load_max_error = no_load_max_vel_error / 100
    i = 0

    # Increment speed by no_load_speed_step until the maximum motor speed is reached
    while vel_error < no_load_max_error:
        temp_check() # Check all temperatures are safe
        set_speed = i * no_load_speed_step # [counts/s]
        
        # Prevent motor speed becoming dangerously fast
        if set_speed > no_load_max_motor_speed:
            print("Maximum safe speed of ", no_load_max_motor_speed, ' reached.')
            break

        # Set motor speed, wait for things to stabilise
        my_odrive.axis0.controller.vel_setpoint = rpm_to_cpr(set_speed) # [counts/s]
        time.sleep(stabilise_time) # wait for speed to stabilise [s]

        # Estimate how close motor speed is to that requested and prevent divide by zero
        if set_speed > 0:
            vel_error = 1 - (my_odrive.axis0.encoder.vel_estimate / my_odrive.axis0.controller.vel_setpoint) # speed error [%]
        input_power = my_odrive.vbus_voltage * my_odrive.axis0.muv2 * 0.001
        print(i, ' ', set_speed, ' ' , vel_error * 100, ' ', input_power)
        write_values(measure_values())
        i += 1

    print("No-load speed test complete")
    print("Maximum motor speed = " , set_speed - no_load_speed_step) # Revert set speed to previous value 

def motor_controller_loss_test():
    """
    Estimates Odrive I^2R losses by incrementally increasing motor current.
    Motor phase current and reported motor resistance report_motor_parameters() gives motor power draw.
    Subtracting motor power draw from Odrive power draw (current shunt reading x Vbus) gives Odrive I^2R loss.
    """
    print('Starting motor controller loss test..')
    print('Reading No., Calibration current (A), Motor temperature (DegC)')
    my_odrive.axis0.requested_state = AXIS_STATE_IDLE
    for i in range(loss_est_num_steps):
        temp_check() # Check all temperatures are safe.
        print(i)

        # If the motor becomes too hot then its resistance will be different to that measured at start-up.
        # A higher than expected motor resistance will lead to an over-estimate Odrive I^2R losses.
        while motor_temp(loss_est_motor_under_test) >= loss_est_max_motor_temp:
            time.sleep(2)
            print('Motor too hot. Motor Temp (DegC): ', motor_temp(loss_est_motor_under_test))

        my_odrive.axis0.motor.config.calibration_current = (i + 1) * loss_est_current_step # Can not calibrate with current = 0
        # Offset calibration used as a means for the motor to draw a large current without doing mechanical work.
        # Offset calibration also loads each phases evenly, resulting in even MOSFET and motor phase heating.
        
        x = 0
        my_odrive.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        while my_odrive.axis0.current_state != AXIS_STATE_IDLE: # wait for offset calibration to finish before incrementing current 
            if x == 0:
                print(i, ' ', my_odrive.axis0.motor.config.calibration_current, ' ', motor_temp(loss_est_motor_under_test))
                time.sleep(stabilise_time)
                write_values(measure_values())
                x = 1 # Write to file only once per calibration current setting
            time.sleep(0.5)

    my_odrive.axis0.motor.config.calibration_current = calibration_current # set calibration current back to default value.
    print('Motor controller loss test complete.')

######################## Measurement procedure ###########################
# Note: To prevent different test from overwriting earlier results set erase_file_on_startup_flag to false.

odriveStartup()
report_motor_parameters()
motor_controller_loss_test()
report_motor_parameters() # repeat to check motor resistance is still the same
#no_load_speed_test()
odriveShutdown()
