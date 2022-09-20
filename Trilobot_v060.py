#!/usr/bin/env python3

from time import perf_counter, sleep
from random import randint
from trilobot import Trilobot, BUTTON_A, LIGHT_FRONT_LEFT, LIGHT_MIDDLE_LEFT, LIGHT_REAR_LEFT, LIGHT_FRONT_RIGHT, LIGHT_MIDDLE_RIGHT, LIGHT_REAR_RIGHT

"""
  Name:       Trilobot
  Author:     Dale Weber <hybotics.wy@gmail.com>
  License:    MIT

  Version:    0.5.0
  Purpose:    Obstacle avoidance, initial version
  Date:       23-Jul-2022

  Version:    0.6.0
  Date:       24-Jul-2022
  Purpose:    Fix turning and backing up, add backup lights
"""

DEBUG = True

# Default values for the distance_reading() function
COLLISION_THRESHOLD_CM  = 22
MAX_NUM_READINGS        = 10
MAX_TIMEOUT_SEC         = 25
MAX_NUM_SAMPLES         = 3

# Speed parametners
NORMAL_LEFT_SPEED       = 0.65
NORMAL_RIGHT_SPEED      = 0.65

# Turning parameters
TURN_SPEED              = 0.65
TURN_TIME_SEC           = 0.55
TURN_RIGHT_PERCENT      = 65

# Backup parameters
BACKUP_LOOPS            = 5
BACKUP_TIME_SEC         = 0.15

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
    trilobot.set_motor_speeds(NORMAL_LEFT_SPEED, NORMAL_RIGHT_SPEED)
    distance_cm, collision = distance_reading_cm(trilobot)

    # Move forward until we have a collision nevent
    while not collision:
      if DEBUG:
        print("Forward distance is {0:5.2f} cm, Collision is {1}".format(distance_cm, collision))

      blink_underlights(trilobot, FRONT_LIGHTS, GREEN)
      distance_cm, collision = distance_reading_cm(trilobot)
      sleep(0.25)

    # React to a possible collision
    if collision:
      if DEBUG:
        print("Reacting to an imminent collision")
        print()
        print("Backing up")

      for ba in range(BACKUP_LOOPS):
        blink_underlights(trilobot, REAR_LIGHTS, YELLOW)
        trilobot.set_motor_speeds(-TURN_SPEED, -TURN_SPEED)
        sleep(BACKUP_TIME_SEC)

      percent = randint(1, 100)

      if DEBUG:
        print(f"Percentage = {percent}")

      if DEBUG:
        print("Beginning turn distance = {0:5.2f}, Collision = {1}".format(distance_cm, collision))

      while collision:
        if percent < TURN_RIGHT_PERCENT:
          # Turn right
          if DEBUG:
            print("Turning right")

          blink_underlights(trilobot, RIGHT_LIGHTS, BLUE)
          trilobot.set_motor_speeds(TURN_SPEED, -TURN_SPEED)
        else:
          # Turn left
          if DEBUG:
            print("Turning left")

          blink_underlights(trilobot, LEFT_LIGHTS, BLUE)
          trilobot.set_motor_speeds(-TURN_SPEED, TURN_SPEED)

        sleep(TURN_TIME_SEC)
        distance_cm, collision = distance_reading_cm(trilobot)

        if DEBUG:
          print("Turning distance = {0:5.2f}, Collision = {1}".format(distance_cm, collision))

    if DEBUG:
      print("Exit turn distance is {0:5.2f} cm, Collision is {1}".format(distance_cm, collision))
except KeyboardInterrupt:
  trilobot.set_motor_speeds(0.0, 0.0)

  if DEBUG:
    print()
    print("Exiting by Ctrl/C")
