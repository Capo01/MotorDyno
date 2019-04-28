import odrive
import time
import math
import matplotlib.pyplot as plt

<<<<<<< Updated upstream
time_sleep = 2 # [s]
speed_step = 100 # [RPM]
max_speed = 3000 # [RPM]
num_readings = 10 # Number of readings to be averaged
smoothing_factor = 0.5 #smoothing factor

req_steps = max_speed / speed_step
=======

num_odrvies_to_find = 2
odrives = odrive.find_any(find_multiple=num_odrvies_to_find)

print('found {} ODrives'.format(len(odrives)))
for odrv in odrives:
    print(odrv.serial_number)
>>>>>>> Stashed changes

my_odrive.axis0.controller.config.control_mode = 2
my_odrive.axis0.controller.config.vel_limit = (max_speed / 60) *8192 # [counts/s]

time.sleep(1)

f = open("Output_data.txt","w+")

for i in range(int(req_steps)):
if i == 0:
	f.write('Time' + '	' + 'Input voltage' + '	' + 'Input current' + '	' + 'Input power' + '	' + 'Output torque' + '	' + 'Output speed' + '	' + 'Output power'+ '	' + 'Efficiency' + '\n')
	f.write('S' + '	' + 'V' + '	' + 'mA' + '	' + 'W' + '	' + 'N.m' + '	' + 'rad.s' + '	' + 'W' + '	' + '%' + '\n')

set_speed = i * speed_step
my_odrive.axis0.controller.vel_setpoint =  (set_speed / 60) * 8192# [counts/s]
time.sleep(time_sleep) # wait for speed to stabalise [s]

input_voltage 		= (my_odrive.vbus_voltage) # measured by odrive [V]
input_current 		= (my_odrive.axis1.controller.vel_ramp_target) # measured by external hall effect sensor [mA]
input_power 		= (input_voltage * input_current * 0.001) # [W]
output_torque 		= (- my_odrive.axis0.controller.vel_ramp_target) # estimated by load cell [N.m]
output_speed 		= ((my_odrive.axis0.encoder.vel_estimate / 8192) * 2 * math.pi)# motor speed [rad.s]
output_power		= (output_torque * output_speed) # power produced by the motor [W]
output_efficiency 	= (output_power / input_power * 100) # efficiency including motor controller (%)

for i in range(num_readings):

	# Read input values
	# reading = (reading * x) + (1.0-x) * adc_reading -> where x is a smoothing factor between 0.0 and 1.0
	input_voltage 		= (input_voltage * smoothing_factor ) + (1.0 - smoothing_factor) * (my_odrive.vbus_voltage) # measured by odrive [V]
	input_current 		= (input_current * smoothing_factor ) + (1.0 - smoothing_factor) * (my_odrive.axis1.controller.vel_ramp_target) # measured by external hall effect sensor [mA]
	input_power 		= (input_power  * smoothing_factor ) + (1.0 - smoothing_factor) * (input_voltage * input_current * 0.001) # [W]
	output_torque 		= (output_torque  * smoothing_factor ) + (1.0 - smoothing_factor) * (- my_odrive.axis0.controller.vel_ramp_target) # estimated by load cell [N.m]
	output_speed 		= (output_speed * smoothing_factor ) + (1.0 - smoothing_factor) * ((my_odrive.axis0.encoder.vel_estimate / 8192) * 2 * math.pi)# motor speed [rad.s]
	output_power		= (output_power * smoothing_factor ) + (1.0 - smoothing_factor) * (output_torque * output_speed) # power produced by the motor [W]
	output_efficiency 	= (output_efficiency * smoothing_factor ) + (1.0 - smoothing_factor) * (output_power / input_power * 100) # efficiency including motor controller (%)

# Avearge input valuse

line = str(i * time_sleep) + '	' + str(input_voltage) + '	' + str(input_current) + '	' + str(input_power) + '	' + str(output_torque) + '	' + str(output_speed) + '	' + str(output_power) + '	' + str(output_efficiency) + '\n'
print(line)
f.write(line)

my_odrive.axis0.controller.vel_setpoint = 0

	absorber_temp = ((1/ ((math.log10(((10000 / (3.3 / my_odrive.get_adc_voltage(2) - 1))/10000))/3435) + 1.0 / (25 + 273.15))) - 273)


##### Example of how to plot data ######
mysample = []
myothersample = []
for x in range(100):
  time.sleep(0.1) # wait for steady state
  mysample.append(my_odrive.axis0.controller.vel_ramp_target)
  myothersample.append(((8.27*my_odrive.axis0.motor.current_control.Iq_setpoint/150)))

<<<<<<< Updated upstream
plt.plot(mysample)
plt.plot(myothersample)
plt.show()
=======
# 	absorber_temp = ((1/ ((math.log10(((10000 / (3.3 / my_odrive.get_adc_voltage(2) - 1))/10000))/3435) + 1.0 / (25 + 273.15))) - 273)


# ##### Example of how to plot data ######
# mysample = []
# myothersample = []
# for x in range(100):
#   time.sleep(0.1) # wait for steady state
#   mysample.append(my_odrive.axis0.controller.vel_ramp_target)
#   myothersample.append(((8.27*my_odrive.axis0.motor.current_control.Iq_setpoint/150)))

# plt.plot(mysample)
# plt.plot(myothersample)
# plt.show()
>>>>>>> Stashed changes
