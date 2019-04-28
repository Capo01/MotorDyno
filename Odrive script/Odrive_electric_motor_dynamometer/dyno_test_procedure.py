#!/usr/bin/env python3

"""
Odrive electric motor dynamometer test procedure.This file states what 
tests will be conducted their parameters.
https://github.com/Capo01/MotorDyno
R. Parsons

Example:
	odrive_startup()
    no_load_max_speed(500, 100)
    odrive_shutdown()
"""

# local source
import dyno_functions

# test procedure
dyno_functions.odrive_startup()
dyno_functions.write_values(dyno_functions.measure_values())
dyno_functions.odrive_shutdown()