# Challenge 3: Wall Follow — Full PID
# Add the Integral term to fix drift around the L corner

from aidriver import AIDriver, hold_state
import aidriver

aidriver.DEBUG_AIDRIVER = True
my_robot = AIDriver("left")  # ← Change to "right" if wall is on your right

# ═══════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════
BASE_SPEED = 200
TARGET_WALL_DISTANCE = 200
side_Kp = 0             # Use the Kp you found in Challenge 1
side_Kd = 0             # Use the Kd you found in Challenge 2
side_Ki = 0            # Start very small — raise in 0.002 steps
MAX_STEERING = 60
side_INTEGRAL_MAX = 0   # Anti-windup clamp

# ══════════════════════════════════════��════════════════
# MAIN LOOP
# ════════════════════════════════════════════════════���══
side_previous_error = 0
side_integral = 0

while True:
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
