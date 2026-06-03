# Challenge 3: Wall Follow — Full PID
# Add the Integral term to fix drift around the L corner

from aidriver import AIDriver, hold_state
import aidriver

aidriver.DEBUG_AIDRIVER = True
my_robot = AIDriver("left")  # ← Change to "right" if wall is on your right

# ══════════════════════════════════════���════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════

# ------------------ PRIMARY BASE SPEED ------------------
BASE_SPEED = 200

# ------------------ KI KP KD AND WALL DISTANCE ------------------
TARGET_WALL_DISTANCE = 150
MAX_STEERING = 60
side_Kp = 0.35            # Use the Kp you found in Challenge 1
side_Kd = 1.5             # Use the Kd you found in Challenge 2
side_Ki = 0.001            # Start very small — raise in 0.002 steps
side_INTEGRAL_MAX = 60   # Anti-windup clamp

# ------------------ FRONT WALL DISTANCE ------------------
FRONT_SLOW_DISTANCE = 500   # mm — start decelerating
FRONT_STOP_DISTANCE = 100   # mm — stop and turn
FRONT_Kp            = 1.2   # deceleration gain
TURN_SPEED          = 200
TURN_TIME_90        = 0.71

# ═════════��═════════��════════════════════════════════���════
# MAIN LOOP
# ════════════════════════════════���════════════════════════
side_previous_error = 0
side_integral = 0

while True:
# ------------------ Full Front Distance Wall PID  ------------------
  front = my_robot.read_distance()

  if front != -1 and front < FRONT_SLOW_DISTANCE:
      if front <= FRONT_STOP_DISTANCE:
          my_robot.brake()
          hold_state(0.3)

          # Rotate AWAY from the wall (wall_sign tells us which side).
          if my_robot.wall_sign == -1:
              my_robot.rotate_right(TURN_SPEED)
          else:
              my_robot.rotate_left(TURN_SPEED)
          hold_state(TURN_TIME_90)

          my_robot.brake()
          hold_state(0.3)

          side_integral = 0
          side_previous_error = 0
          continue
      else:
          # Approach the wall on a P-controlled deceleration ramp.
          approach_speed = int(FRONT_Kp * (front - FRONT_STOP_DISTANCE))
          if approach_speed < 120:
              approach_speed = 120
          if approach_speed > BASE_SPEED:
              approach_speed = BASE_SPEED
          my_robot.drive(approach_speed, approach_speed)
          hold_state(0.05)
          continue

# ------------------ Full Side Wall PID  ------------------
  wall_distance = my_robot.read_distance_2()

  if wall_distance == -1:
      my_robot.drive(BASE_SPEED, BASE_SPEED)
      side_integral = 0  # Reset when wall lost
      hold_state(0.05)
      continue

  error = wall_distance - TARGET_WALL_DISTANCE

  # Integral: accumulated error
  side_integral = side_integral + error
  if side_integral > side_INTEGRAL_MAX:
      side_integral = side_INTEGRAL_MAX
  elif side_integral < -side_INTEGRAL_MAX:
      side_integral = -side_INTEGRAL_MAX

  # Derivative
  side_derivative = error - side_previous_error

  # Full PID
  steering = (side_Kp * error) + (side_Ki * side_integral) + (side_Kd * side_derivative)

  if steering > MAX_STEERING:
      steering = MAX_STEERING
  elif steering < -MAX_STEERING:
      steering = -MAX_STEERING

  right_speed = BASE_SPEED - (my_robot.wall_sign * steering)
  left_speed  = BASE_SPEED + (my_robot.wall_sign * steering)

  my_robot.drive(int(right_speed), int(left_speed))

  side_previous_error = error
  hold_state(0.05)
