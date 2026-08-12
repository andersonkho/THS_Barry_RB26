# Challenge 4: Corner Detection — your first state machine.
# The robot is always in ONE state: FOLLOW_WALL or TURN.
# Write the gyro turn PID here — you reuse it in every later challenge.
# Fill in the values below. Guide: docs.html?doc=Challenge_4

from aidriver import AIDriver, hold_state
import aidriver

aidriver.DEBUG_AIDRIVER = False
my_robot = AIDriver("left")

# Each loop runs the current state, which returns the next state to run.
# States: FOLLOW_WALL (hold the wall) and TURN (spin 90° away from a wall ahead).

# --- FOLLOW_WALL parameters ---
BASE_SPEED = 200  # cruise speed
TARGET_WALL_DISTANCE = 110  # mm to hold from the side wall
MAX_STEERING = 60  # steering clamp

side_Kp = 0.4  # proportional gain
side_Ki = 0.015  # integral gain
side_Kd = 0.3  # derivative gain
side_INTEGRAL_MAX = 1100  # anti-windup clamp

FRONT_SLOW_DISTANCE = 1500  # start slowing when a wall is this close ahead
FRONT_Kp = 1  # how hard to slow down on approach

# --- TURN parameters (your gyro turn PID — reused in every later challenge) ---
turn_Kp = 30  # proportional gain on the heading error
turn_Kd = 0.4  # derivative gain — damps overshoot
turn_tolerance = 2.0  # stop within this many degrees of the target

# Fixed turn mechanics (no tuning needed):
TURN_ANGLE = 90  # every corner is a 90 degree turn
TURN_DT = 0.02  # seconds per turn step (matches hold_state)
TURN_MAX_SPEED = 240  # fastest spin speed
MIN_TURN_SPEED = 190  # slowest spin that still moves the motors
TURN_MAX_STEPS = 200  # safety cap so an untuned turn can't loop forever

# --- Trigger threshold (the logic that moves between states) ---
FRONT_STOP_DISTANCE = 40  # a front wall this close = reached -> TURN (you set)

# --- Persistent state ---
state = "FOLLOW_WALL"
side_integral = 0
side_previous_error = 0


def gyro_turn_pid(turn_right):
    """Spin 90 deg on the spot using the gyro turn PID, then stop."""
    heading = 0.0
    prev_error = TURN_ANGLE
    steps = 0
    while (TURN_ANGLE - heading) > turn_tolerance and steps < TURN_MAX_STEPS:
        gz = my_robot.read_gyro_z_dps()
        heading = heading + abs(gz) * TURN_DT
        error = TURN_ANGLE - heading
        derivative = error - prev_error
        speed = (turn_Kp * error) + (turn_Kd * derivative)
        if speed > TURN_MAX_SPEED:
            speed = TURN_MAX_SPEED
        if speed < MIN_TURN_SPEED:
            speed = MIN_TURN_SPEED
        if turn_right:
            my_robot.drive(-int(speed), int(speed))
        else:
            my_robot.drive(int(speed), -int(speed))
        prev_error = error
        hold_state(TURN_DT)
        steps = steps + 1
    my_robot.brake()


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
    if steering > MAX_STEERING:
        steering = MAX_STEERING
    elif steering < -MAX_STEERING:
        steering = -MAX_STEERING

    right_speed = speed - (my_robot.wall_sign * steering)
    left_speed = speed + (my_robot.wall_sign * steering)
    my_robot.drive(int(right_speed), int(left_speed))
    side_previous_error = error
    hold_state(0.05)
    return "FOLLOW_WALL"


def turn():
    """STATE: wall ahead — spin 90 deg AWAY from the wall."""
    my_robot.brake()
    hold_state(0.3)
    gyro_turn_pid(my_robot.wall_sign == -1)  # left wall -> spin right
    hold_state(0.3)
    return "FOLLOW_WALL"


# --- Main loop ---
while True:
    if state == "FOLLOW_WALL":
        state = follow_wall()
    elif state == "TURN":
        state = turn()
