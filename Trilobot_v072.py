#!/usr/bin/env python3
"""
  Name:       Trilobot
  Author:     Dale Weber <hybotics.wy@gmail.com>
  License:    MIT

  Version:    0.7.2
  Date:       20-Sep-2022
  Purpose:    Removed the translate() and ultrasonic ranger functions because they are not needed
              Added a DRIVE_ON bool to allow turning the drive system on or off

  Version:    0.7.1
  Date:       17-Sep-2022
  Purpose:    Folded the array extraction code from average_distances() into a new
                np_extract() function and added error detection for that. This code
                was used three times in average_distances().
              Added documentation comments to the top of every function.

  Version:    0.7.0
  Date:       16-Sep-2022
  Purpose:    Added VL53L5X ToF sensor

  Version:    0.6.0
  Date:       24-Jul-2022
  Purpose:    Fix turning and backing up, add backup lights

  Version:    0.5.0
  Purpose:    Obstacle avoidance, initial version
  Date:       23-Jul-2022
"""

from time import perf_counter, sleep
from random import random
import numpy as np
import vl53l5cx_ctypes as vl53l5cx
from trilobot import Trilobot, BUTTON_A, LIGHT_FRONT_LEFT, LIGHT_MIDDLE_LEFT, LIGHT_REAR_LEFT, LIGHT_FRONT_RIGHT, LIGHT_MIDDLE_RIGHT, LIGHT_REAR_RIGHT

DEBUG = True
DEBUG_1 = False

# Default values for the distance_reading() function
COLLISION_THRESHOLD_MM  = 200
MAX_NUM_READINGS        = 10
MAX_TIMEOUT_SEC         = 25
MAX_NUM_SAMPLES         = 3

# Speed parametners
NORMAL_LEFT_SPEED       = 0.50
NORMAL_LEFT_OFFSET      = 0.0
NORMAL_RIGHT_SPEED      = 0.50
NORMAL_RIGHT_OFFSET     = -0.50

# Turning parameters
TURN_SPEED              = 0.45
TURN_TIME_SEC           = 0.45
TURN_RIGHT_PERCENT      = 45
TURN_TOLERANCE_MM       = 1.0
LEFT                    = 1
RIGHT                   = 2

# Backup parameters
DEFAULT_BACKUP_LOOPS    = 5
DEFAULT_BACKUP_TIME_SEC = 0.15

# Colors for the underlighting
RED                     = (255, 0, 0)
BLUE                    = (0, 0, 255)
GREEN                   = (0, 255, 0)
PURPLE                  = (127, 0, 127)
YELLOW                  = (255, 255 ,0)
CYAN                    = (0, 255, 255)

'''
  Setup the underlighting
'''
DEFAULT_BLINK_COLOR     = BLUE
DEFAULT_BLINK_RATE_SEC  = 0.25
DEFAULT_NUM_CYCLES      = 1

# Light groups
LEFT_LIGHTS             = [ LIGHT_FRONT_LEFT, LIGHT_MIDDLE_LEFT ]
RIGHT_LIGHTS            = [ LIGHT_FRONT_RIGHT, LIGHT_MIDDLE_RIGHT ]
REAR_LIGHTS             = [ LIGHT_REAR_LEFT, LIGHT_REAR_RIGHT ]
FRONT_LIGHTS            = [ LIGHT_FRONT_LEFT, LIGHT_FRONT_RIGHT ]

# VL53L5CX Sensor Constants
RANGE_PERCENT = 0.50
DATA_THRESHOLD = 2000.0

INTEGRATION_TIME_MS = 20
SHARPENER_PERCENT = 40

def blink_underlights(trilobot, group, color, nr_cycles=DEFAULT_NUM_CYCLES, blink_rate_sec=DEFAULT_BLINK_RATE_SEC):
  '''
      Blink underlighting by group

      Required inputs:
        instance        trilobot                The current Trilobot instance
        list or tuple   group                   A List or Tuple of the light(s) to blink
        tuple           color                   Color for the lights

      Optional inputs:
        int             nr_cycles               Number of cycles of blinking
        float           blink_rate_sec          The blink rate in seconds

      Returns:
        None
  '''
  for cy in range(nr_cycles):
    trilobot.set_underlights(group, color)
    sleep(blink_rate_sec)
    trilobot.clear_underlights(group)

  return None

def print_array(arr):
  '''
    Print a floating point array formatted row/column format
    Currently, a two dimension array is assumed.
  '''
  _rows = len(arr)

  try:
    _cols = len(arr[0])
  except TypeError:
    _rows = 1
    _cols = len(arr)

  if DEBUG_1:
    print(f"(print_array) Rows: {_rows}, Cols: {_cols}")

  print("[")

  for r in range(_rows):
    print("  [ ", end="")

    for c in range(_cols):
      if DEBUG_1:
        print(F"(print_array) r = {r}, c = {c}") 

      if isinstance(c, int):
        print("{0:6d}".format(arr[r][c]), end="")
      else:
        print("{0:6.3f}".format(arr[r][c]), end="")

      if c == _cols - 1:
        print(" ]")
      else:
        print(", ", end="")

  print("]")

  return _rows, _cols

def np_distances_vl53l5cx_mm(vl53l5cx):
  '''
      Get an array of distance measurements in mm

      Required inputs:
        instance        vl53l5cx            Instance of the VL53L5CX sensor

      Returns:
        float           distances           numpy.ndarray of distances in mm
  '''
  # Wait for valid data
  while not vl53l5cx.data_ready():
    pass

  #   Pick out the data to look at
  data = vl53.get_data()
  distances_mm = np.array(data.distance_mm, dtype='uint64').reshape((8, 8))

  #   Data must be flipped horizontilly and vertically to be useful
  distances_mm = np.flipud(distances_mm).astype('uint64')
  distances_mm = np.fliplr(distances_mm).astype('uint64')

  sleep(0.25)

  return distances_mm

def np_extract(arr, start_row, start_col, end_row, end_col):
  '''
      Extract data from a numpy.ndarray
  '''
  if not isinstance(arr, np.ndarray):
    raise ValueError("First parameter must be a 1 or 2 dimension numpy.ndarray")

  _nr_rows = len(arr)

  try:
    _nr_cols = len(arr[0])
  except TypeError:
    _nr_rows = 1
    _nr_cols = len(arr)

  _mm = []

  if DEBUG_1:
    print("(np_extract) Input data")
    print_array(arr)
    print("(np_extract)")
    print(F"(np_extract) _nr_rows = {_nr_rows}, _nr_cols = {_nr_cols}")
    print(F"(np_extract) start_row = {start_row}, start_col = {start_col}, end_row = {end_row}, end_col = {end_col}")

  valid = start_row >= 0 and start_col >= 0 and start_row <= _nr_rows and start_col <= _nr_cols and end_row >= 0 and end_col >= 0 and end_row <= _nr_rows and end_col <= _nr_cols

  if DEBUG_1:
    print(F"(np_extract) Valid = {valid}")

  if valid:
    if DEBUG_1:
      print("(np_extract) Validated data")

    for r in range(start_row, end_row + 1, 1):
      if DEBUG_1:
        print(F"(np_extract) r = {r}")

      _row = []

      for c in range(start_col, end_col + 1, 1):
        if DEBUG_1:
          print(F"(np_extract) c = {c}")
        _row.append(arr[r][c])

      _mm.append(_row)

    if DEBUG_1:
      print("(np_extract) Extracted data")
      print_array(_mm)
  else:
    raise ValueError("(np_extract) Array boundaries are out of range!")

  return _mm

def np_average_distances(vl53):
  '''
      Get distances for left, right, and center
  '''

  # Get distance data from the VL53L5CX sensor
  dist_mm = np_distances_vl53l5cx_mm(vl53)

  if DEBUG:
    print("(np_average_distances) Distances")
    print_array(dist_mm)
    print()

  if DEBUG:
    print("(np_average_distances) Center distance data")

  _mm = np_extract(dist_mm, 2, 2, 5, 5)

  center_distance_mm = int(np.average(_mm))

  if DEBUG:
    print_array(_mm)
    print()
    print("(np_average_distances) Left distance data")

  _mm = np_extract(dist_mm, 0, 0, 7, 1)

  left_distance_mm = int(np.average(_mm))

  if DEBUG:
    print_array(_mm)
    print()
    print("(np_average_distances) Right distance data")

  _mm = np_extract(dist_mm, 0, 6, 7, 7)

  right_distance_mm = int(np.average(_mm))

  if DEBUG:
    print_array(_mm)

  if DEBUG:
    print(F"(average_distances) Left distance = {left_distance_mm} mm, Center distance = {center_distance_mm}, Right distance = {right_distance_mm}")

  return left_distance_mm, right_distance_mm, center_distance_mm

def check_for_collision(fwd_mm, threshold=COLLISION_THRESHOLD_MM):
  '''
      Check to see if the robot is about to collide with an obstacle
  '''
  result = False

  if fwd_mm <= threshold:
    result = True

  return result

def turn_left():
  if DEBUG:
    print("(main) Turning left")

  last_turn = LEFT
  blink_underlights(trilobot, LEFT_LIGHTS, BLUE)

  if DRIVE_ON:
    trilobot.set_motor_speeds(-TURN_SPEED, TURN_SPEED)

def turn_right():
  if DEBUG:
    print("(main) Turning right")

  last_turn = RIGHT
  blink_underlights(trilobot, RIGHT_LIGHTS, BLUE)

  if DRIVE_ON:
    trilobot.set_motor_speeds(TURN_SPEED, -TURN_SPEED)

def backup(loops=DEFAULT_BACKUP_LOOPS, wait_sec=DEFAULT_BACKUP_TIME_SEC):
  if DEBUG:
    print("(main) Backing up")

  if DRIVE_ON:
    for l in range(loops):
      blink_underlights(trilobot, REAR_LIGHTS, YELLOW)
      trilobot.set_motor_speeds(-TURN_SPEED, -TURN_SPEED)
      sleep(wait_sec)

'''
  *****************************************************************************
  Start of Initialization
  *****************************************************************************
'''

print("Trilobot with a VL53L5CX ToF Distance Sensor")
print()
print("    Beginning Initialization")
print()

trilobot = Trilobot()

collision = False

print("    Initializing the VL53L5CX ToF distance sensor, please stand by... ", end="")

vl53 = vl53l5cx.VL53L5CX()

#   Set Resolution
vl53.set_resolution(8 * 8)

#   Set Target Order
vl53.set_target_order(0)

#   This is a visual demo, so prefer speed over accuracy
vl53.set_ranging_frequency_hz(15)
vl53.set_integration_time_ms(INTEGRATION_TIME_MS)
vl53.set_sharpener_percent(SHARPENER_PERCENT)

print("Done!")

vl53.start_ranging()

# Turn drive on or off
DRIVE_ON = False

'''
  *****************************************************************************
  Start of Main Line
  *****************************************************************************
'''

if DEBUG:
  print()

try:
  left_distance_mm, right_distance_mm, center_distance_mm = np_average_distances(vl53)
  collision = check_for_collision(center_distance_mm)

  percent = 0.0
  last_turn = 0

  while True:
    if DEBUG:
      print(F"(main) Moving forward: Left distance = {left_distance_mm} mm, Center distance = {center_distance_mm}, Right distance = {right_distance_mm}")

    if DRIVE_ON:
      trilobot.set_motor_speeds(NORMAL_LEFT_SPEED + NORMAL_LEFT_OFFSET, NORMAL_RIGHT_SPEED + NORMAL_RIGHT_OFFSET)

    # Move forward until we have a collision nevent
    while not collision:
      if DEBUG:
        print("(main) Center distance is {0:5.2f} mm, Collision is {1}".format(center_distance_mm, collision))

      blink_underlights(trilobot, FRONT_LIGHTS, GREEN)
      left_distance_mm, right_distance_mm, center_distance_mm = np_average_distances(vl53)
      collision = check_for_collision(center_distance_mm)
      sleep(0.25)

    # React to a possible collision
    while collision:
      percent = random() * 100.0

      if DEBUG:
        print("(main) Reacting to an imminent collision")
        print("(main)")

      backup()

      if DEBUG:
        print("(main) Beginning turn distance = {0:5.2f}, Collision = {1}".format(center_distance_mm, collision))

      percent = round(random() * 100.0, 2)

      if DEBUG:
        print(F"(main) Choosing Turn Direction: Percent = {percent}, Left = {left_distance_mm} mm, Right = {right_distance_mm} mm")

      #if (percent < 50.0 and last_turn == RIGHT) or right_distance_mm >= left_distance_mm + TURN_TOLERANCE_MM:
      if right_distance_mm >= left_distance_mm + TURN_TOLERANCE_MM:
        # Turn right
        turn_right()
      #elif (percent > 50.0 and last_turn == LEFT) or left_distance_mm >= right_distance_mm + TURN_TOLERANCE_MM:
      elif left_distance_mm >= right_distance_mm + TURN_TOLERANCE_MM:
        # Turn left
        turn_left()
      elif percent < 50.0:
        turn_right()
      elif percent >= 50.0:
        turn_left()
      else;
        backup()

      sleep(TURN_TIME_SEC)

      left_distance_mm, right_distance_mm, center_distance_mm = np_average_distances(vl53)
      collision = check_for_collision(center_distance_mm)

      if DEBUG:
        print("(main) Turning distance = {0:5.2f} mm, Collision = {1}".format(center_distance_mm, collision))

    if DEBUG:
      print("Exit turn distance is {0:5.2f} mm, Collision is {1}".format(center_distance_mm, collision))
except KeyboardInterrupt:
  trilobot.set_motor_speeds(0.0, 0.0)

  if DEBUG:
    print()
    print("Exiting by Ctrl/C")
