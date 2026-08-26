from aidriver import AIDriver, hold_state, ticks_ms, ticks_diff
import aidriver

# Begin Program
aidriver.DEBUG_AIDRIVER = False
my_robot = AIDriver("left", "tof")

# Main Parameters
BASE_SPEED = 200             # Foward Speed
TARGET_WALL_DISTANCE = 100   # Side Wall Distance Target
MAX_STEERING = 40            # Wheel Difference Speed

# PID Controller
side_Kp = 0.55               # carry forward
side_Kd = 1.5                # carry forward
side_Ki = 0.015              # integral gain — start very small
side_INTEGRAL_MAX = 40       # anti-windup clamp

FRONT_SLOW_DISTANCE = 200    # Wall Slow Distance (mm)
FRONT_Kp = 1.4               # Slow down Break

# Gyroscope PID
turn_Kp = 30                 # Speed Toward Angle
turn_Kd = 1.2                # Dampen Turn Angle
turn_tolerance = 1.7         # Gryo Acceptable Stop Distance

# Stop Distance
FRONT_STOP_DISTANCE = 50     # Front Stop Distance

# Presistent State
state = "FOLLOW_WALL"
side_integral = 0
side_previous_error = 0

# No Tune Section
TURN_ANGLE = 90  # every corner is a 90 degree turn
TURN_DT = 0.02  # seconds per turn step (matches hold_state)
TURN_MAX_SPEED = 120  # fastest spin speed
MIN_TURN_SPEED = 100  # slowest spin that still rotates the robot
TURN_TIMEOUT_MS = 2500  # safety cap so an untuned turn can't spin forever
TURN_KICK_SPEED = 255  # opening burst that breaks static friction
TURN_KICK_STEPS = 4  # how many steps that burst lasts
TURN_COAST_TIME = 0.03  # seconds the robot keeps rotating after braking
TURN_SETTLE_STEPS = 15  # gyro samples taken while it coasts to a stop
NUDGE_SPEED = 220  # correction pulse speed (must beat static friction)
NUDGE_MS_PER_DEG = 4  # pulse length per degree of remaining error
NUDGE_MIN_MS = 25  # shortest useful pulse
NUDGE_MAX_MS = 250  # longest allowed pulse
NUDGE_STEP = 0.005  # sample interval inside a pulse
TURN_MAX_NUDGES = 6  # give up correcting after this many pulses
TURN_CLEAR_TIME = 0.4  # drive on after a turn to leave the trigger zone


def _turn_is_right(relative_angle_deg):
    """Map wall-relative turn sign to a physical spin direction."""
    if my_robot.wall_sign < 0:
        return relative_angle_deg >= 0
    return relative_angle_deg < 0


def _spin(turn_right, speed, reverse=False):
    """Pivot on the spot at *speed*, optionally against the turn direction."""
    if turn_right != reverse:
        my_robot.drive(-speed, speed)
    else:
        my_robot.drive(speed, -speed)


def _settle(heading, last_ms, gyro_sign, samples):
    """Integrate the gyro while the robot coasts to a stop after braking."""
    for _ in range(samples):
        hold_state(0.02)
        gz = my_robot.read_gyro_z_dps() * gyro_sign
        now = ticks_ms()
        dt = ticks_diff(now, last_ms) / 1000.0
        last_ms = now
        heading = heading + (gz * dt)
    return heading, last_ms


def gyro_turn_pid(relative_angle_deg):
    """Spin by a wall-relative angle using the gyro turn PID, then stop."""
    turn_right = _turn_is_right(relative_angle_deg)
    target = float(abs(relative_angle_deg))

    heading = 0.0
    last_ms = ticks_ms()

    # Phase 1 - kick through static friction, and learn which sign this IMU
    # gives for this rotation so an inverted mounting cannot flip the loop.
    _spin(turn_right, TURN_KICK_SPEED)
    raw_sum = 0.0
    for _ in range(TURN_KICK_STEPS):
        hold_state(TURN_DT)
        raw = my_robot.read_gyro_z_dps()
        raw_sum = raw_sum + raw
        now = ticks_ms()
        dt = ticks_diff(now, last_ms) / 1000.0
        last_ms = now
        heading = heading + (abs(raw) * dt)
    gyro_sign = -1 if raw_sum < 0 else 1

    # Phase 2 - PID cruise. Braking is predictive: cut power once the angle we
    # would coast through lands on the target.
    prev_error = target - heading
    start_ms = last_ms
    while True:
        gz = my_robot.read_gyro_z_dps() * gyro_sign
        now = ticks_ms()
        dt = ticks_diff(now, last_ms) / 1000.0
        if dt <= 0:
            dt = 0.001
        last_ms = now
        heading = heading + (gz * dt)

        if heading + (gz * TURN_COAST_TIME) >= target:
            break
        if ticks_diff(now, start_ms) > TURN_TIMEOUT_MS:
            break

        error = target - heading
        output = (turn_Kp * error) + (turn_Kd * (error - prev_error))
        prev_error = error
        speed = int(output)
        if speed > TURN_MAX_SPEED:
            speed = TURN_MAX_SPEED
        if speed < MIN_TURN_SPEED:
            speed = MIN_TURN_SPEED
        _spin(turn_right, speed)
        hold_state(TURN_DT)

    my_robot.brake()

    # Phase 3 - keep integrating through the coast, then close the remaining
    # error with short pulses until it is inside tolerance.
    heading, last_ms = _settle(heading, last_ms, gyro_sign, TURN_SETTLE_STEPS)
    nudges = 0
    while nudges < TURN_MAX_NUDGES:
        error = target - heading
        if abs(error) <= turn_tolerance:
            break
        pulse_ms = int(NUDGE_MS_PER_DEG * abs(error))
        if pulse_ms < NUDGE_MIN_MS:
            pulse_ms = NUDGE_MIN_MS
        if pulse_ms > NUDGE_MAX_MS:
            pulse_ms = NUDGE_MAX_MS
        _spin(turn_right, NUDGE_SPEED, error < 0)
        pulse_start = ticks_ms()
        while ticks_diff(ticks_ms(), pulse_start) < pulse_ms:
            hold_state(NUDGE_STEP)
            gz = my_robot.read_gyro_z_dps() * gyro_sign
            now = ticks_ms()
            dt = ticks_diff(now, last_ms) / 1000.0
            last_ms = now
            heading = heading + (gz * dt)
        my_robot.brake()
        heading, last_ms = _settle(heading, last_ms, gyro_sign, 10)
        nudges = nudges + 1
    return heading


def follow_wall():
    """STATE: hold the side wall with the side PID. Returns the next state."""
    global side_integral, side_previous_error

    front = my_robot.read_distance()
    # Trigger -> TURN: a wall is reached straight ahead (the corner).
    if front != -1 and front <= FRONT_STOP_DISTANCE:
        side_integral = 0
        side_previous_error = 0
        return "TURN"

    side = my_robot.read_distance_2()
    if side == -1:
        # No side wall this tick: cruise straight until it returns.
        my_robot.drive(BASE_SPEED, BASE_SPEED)
        hold_state(0.05)
        return "FOLLOW_WALL"

    # Speed: slow down if a wall is coming up ahead.
    if front != -1 and front < FRONT_SLOW_DISTANCE:
        speed = int(FRONT_Kp * (front - FRONT_STOP_DISTANCE))
        if speed < my_robot.min_approach_speed:
            speed = my_robot.min_approach_speed
        if speed > BASE_SPEED:
            speed = BASE_SPEED
    else:
        speed = BASE_SPEED

    # Steering: the side PID holds the wall at the target distance.
    error = side - TARGET_WALL_DISTANCE
    side_integral = side_integral + error
    if side_integral > side_INTEGRAL_MAX:
        side_integral = side_INTEGRAL_MAX
    elif side_integral < -side_INTEGRAL_MAX:
        side_integral = -side_INTEGRAL_MAX
    derivative = error - side_previous_error
    steering = (side_Kp * error) + (side_Ki * side_integral) + (side_Kd * derivative)

    # Cap steering so the slower wheel can never fall into the motor dead zone.
    steer_limit = speed - my_robot.MIN_MOTOR_SPEED
    if steer_limit < 0:
        steer_limit = 0
    if steer_limit > MAX_STEERING:
        steer_limit = MAX_STEERING
    if steering > steer_limit:
        steering = steer_limit
    elif steering < -steer_limit:
        steering = -steer_limit

    right_speed = speed - (my_robot.wall_sign * steering)
    left_speed = speed + (my_robot.wall_sign * steering)
    my_robot.drive(int(right_speed), int(left_speed))
    side_previous_error = error
    hold_state(0.05)
    return "FOLLOW_WALL"


def turn():
    """STATE: wall ahead — spin 90 deg AWAY from the wall."""
    my_robot.brake()
    my_robot.clear_display()  # no I2C traffic while the gyro loop is running
    hold_state(0.3)
    gyro_turn_pid(TURN_ANGLE)
    hold_state(0.3)
    # Move out of the trigger zone, else FOLLOW_WALL re-fires TURN instantly.
    front = my_robot.read_distance()
    if front == -1 or front > FRONT_STOP_DISTANCE:
        my_robot.drive(BASE_SPEED, BASE_SPEED)
        hold_state(TURN_CLEAR_TIME)
        my_robot.brake()
    return "FOLLOW_WALL"


# --- Main loop ---
while True:
    if state == "FOLLOW_WALL":
        state = follow_wall()
    elif state == "TURN":
        state = turn()
