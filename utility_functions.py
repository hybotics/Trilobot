
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

def translate(value, to_min, to_max, from_min, from_max):
  '''
      Translate a value from one range to another
  '''
  # Figure out how 'wide' each range is
  to_span = to_max - to_min
  from_span = from_max - from_min

  spans_decimal = to_span / from_span

  # Convert the left range into a 0-1 range (float)
  scaled_value = int(round(float(value - to_min) * spans_decimal, 1) + 1)

  return scaled_value

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

  #print(f"Rows: {rows}, Cols: {cols}")
  print("[")

  for r in range(_rows):
    print("  [ ", end="")

    for c in range(_cols):
      if isinstance(c, int):
        print("{0:6d}".format(arr[r][c]), end="")
      else:
        print("{0:6.3f}".format(arr[r][c]), end="")

      if c == cols - 1:
        print(" ]")
      else:
        print(", ", end="")

  print("]")

  return rows, cols

def distance_ultrasonic_reading_cm(trilobot, collision_threshold_cm=COLLISION_THRESHOLD_MM, nr_readings=MAX_NUM_READINGS, nr_samples=MAX_NUM_SAMPLES, timeout_sec=MAX_TIMEOUT_SEC):
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

