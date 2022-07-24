#!/usr/bin/env python3

from time import perf_counter, sleep
from random import randint
from trilobot import Trilobot, BUTTON_A, LIGHT_FRONT_LEFT, LIGHT_MIDDLE_LEFT, LIGHT_FRONT_RIGHT, LIGHT_MIDDLE_RIGHT

"""
  Name:       Trilobot
  Author:     Dale Weber <hybotics.wy@gmail.com>

  Purpose:    Obstale avoidance
  Date:       23-Jul-2022

  Version:    1.0.0   Initial version
  License:    MIT
"""

DEBUG = True

# Default values for the distance_reading() function
COLLISION_THRESHOLD_CM  = 18
MAX_NUM_READINGS        = 10
MAX_TIMEOUT_SEC         = 25
MAX_NUM_SAMPLES         = 3

START_LEFT_SPEED        = 0.75
START_RIGHT_SPEED       = 0.75
TURN_SPEED              = 0.50
TURN_TIME_SEC           = 0.50
TURN_RIGHT_PERCENT      = 75

# Colors for the underlighting
RED                     = (127, 0, 0)
BLUE                    = (0, 0, 127)
GREEN                   = (0, 127, 0)

'''
  Setup the underlighting
'''
DEFAULT_BLINK_COLOR     = BLUE
DEFAULT_BLINK_RATE_SEC  = 0.25
DEFAULT_NUM_CYCLES      = 1

LEFT_LIGHTS             = [ LIGHT_FRONT_LEFT, LIGHT_MIDDLE_LEFT ]
RIGHT_LIGHTS            = [ LIGHT_FRONT_RIGHT, LIGHT_MIDDLE_RIGHT ]


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
def blink_underlights(trilobot, group, color, nr_cycles=DEFAULT_NUM_CYCLES, blink_rate_sec=DEFAULT_BLINK_RATE_SEC):
  for cy in range(nr_cycles):
    trilobot.set_underlights(group, color)
    sleep(blink_rate_sec)
    trilobot.clear_underlights(group)

  return None

'''
  Get a distance reading from an ultrasonic sensor

  Required inputs:
    instance        trilobot                The current Trilobot instance

  Optional inputs:
    float           collision_threshold_cm  The distance in cm where a collision is imminent
    int             nr_readings             The number of readings to take
    int             timeout_sec             The timeout in seconds
    int             nr_samples              The number of samples to get for each reading

  This takes the average of several readings and returns the averaged result along with a collion indicator

  Returns:
    float           distance_average_cm     Average distance in cm
    bool            collision               Collision indicator which will be True for an imminent collsion
'''
def distance_reading_cm(trilobot, collision_threshold_cm=COLLISION_THRESHOLD_CM, nr_readings=MAX_NUM_READINGS, nr_samples=MAX_NUM_SAMPLES, timeout_sec=MAX_TIMEOUT_SEC):
  # Take 10 measurements rapidly
  readings_count = 0
  distance_total_cm = 0.0
  distance_total_cm
  collision = False

  for num in range(nr_readings):
    clock_check = perf_counter()
    counter = 0
    distance = -1.0

    while distance < 0.0 and counter < 10:
      # Make sure the reading is valid
      distance = trilobot.read_distance(timeout=timeout_sec, samples=nr_samples)
      counter += 1

      if counter >= 10:
        emsg = "Unable to get a valid distance reading!"
        blink_underlighting(trilobot, [LEFT_LIGHTS, RIGHT_LIGHTS], RED, 10)
        raise ValueError(emsg)

    readings_count += 1
    distance_total_cm += distance

    sleep(0.01)

  distance_average_cm = distance_total_cm / readings_count

  if distance_average_cm < collision_threshold_cm:
    collision = True

  return distance_average_cm, collision


'''
  Start of mainline
'''
trilobot = Trilobot()

collision = False
percent = 0

'''
  Testing obstacle avoidance
'''
if DEBUG:
  print("Trilobot: Testing obstacle avoidance")

try:
  while True:
    trilobot.set_motor_speeds(START_LEFT_SPEED, START_RIGHT_SPEED)

    while not collision:
      if DEBUG:
        print("Moving forward")

      distance_cm, collision = distance_reading_cm(trilobot)

      if DEBUG:
        print("Forward distance is {0:5.2f} cm, Collision is {1}".format(distance_cm, collision))

    # React to a possible collision
    if DEBUG:
      print("Reacting to an imminent collision")
      print()
      print("Backing up")

    trilobot.set_motor_speeds(-TURN_SPEED, -TURN_SPEED)
    sleep(2.0)

    percent = randint(1, 100)

    if DEBUG:
      print(f"Percentage = {percent}")

    distance_cm, collision = distance_reading_cm(trilobot)

    if DEBUG:
      print("Beginning turn distance = {0:5.2f}, Collision = {1}".format(distance_cm, collision))

    while collision:
      if percent < TURN_RIGHT_PERCENT:
        # Turn right
        if DEBUG:
          print("Turning right")

        blink_underlights(trilobot, RIGHT_LIGHTS, DEFAULT_BLINK_COLOR, 2)
        trilobot.set_motor_speeds(TURN_SPEED, -TURN_SPEED)
      else:
        # Turn left
        if DEBUG:
          print("Turning left")

        blink_underlights(trilobot, LEFT_LIGHTS, DEFAULT_BLINK_COLOR, 2)
        trilobot.set_motor_speeds(-TURN_SPEED, TURN_SPEED)

      sleep(TURN_TIME_SEC)
      distance_cm, collision = distance_reading_cm(trilobot)

      if DEBUG:
        print("Turning distance = {0:5.2f}, Collision = {1}".format(distance_cm, collision))

    distance_cm, collision = distance_reading_cm(trilobot)

    if DEBUG:
      print("Exit turn distance is {0:5.2f} cm, Collision is {1}".format(distance_cm, collision))
except KeyboardInterrupt:
  trilobot.set_motor_speeds(0.0, 0.0)

  if DEBUG:
    print()
    print("Exiting by Ctrl/C")
