# Anderson’s code: 

# Challenge 3: Wall Follow — Full PID
from aidriver import AIDriver, hold_state
import aidriver

aidriver.DEBUG_AIDRIVER = True
my_robot = AIDriver()

BASE_SPEED = 170
TARGET_WALL_DISTANCE = 250
Kp = 0.9
Ki = 0.05
Kd = 3.0
MAX_STEERING = 30
INTEGRAL_MAX = 250

previous_error = 0
integral = 0

while True:
    wall_distance = my_robot.read_distance_2()

    if wall_distance == -1:
        my_robot.drive(BASE_SPEED, BASE_SPEED)
        integral = 0
        hold_state(0.05)
        continue

    error = wall_distance - TARGET_WALL_DISTANCE

    integral = integral + error
    if integral > INTEGRAL_MAX:
        integral = INTEGRAL_MAX
    elif integral < -INTEGRAL_MAX:
        integral = -INTEGRAL_MAX

    derivative = error - previous_error

    steering = (Kp * error) + (Ki * integral) + (Kd * derivative)

    if steering > MAX_STEERING:
        steering = MAX_STEERING
    elif steering < -MAX_STEERING:
        steering = -MAX_STEERING

    right_speed = BASE_SPEED - steering
    left_speed = BASE_SPEED + steering

    my_robot.drive(int(right_speed), int(left_speed))

    previous_error = error
    hold_state(0.05)
