from aidriver import AIDriver, hold_state
import aidriver

"""Hardware sanity test for the AIDriver robot.

Runs a short sequence of movements and distance readings.
Most details are reported via the AIDriver debug logger.
"""

aidriver.DEBUG_AIDRIVER = True

print("Initialising AIDriver hardware test...")

try:
    robot = AIDriver()
except Exception as exc:
    print("Failed to initialise AIDriver:", exc)
    print("Check that 'aidriver.py' is in the 'lib' folder on the device.")
    raise SystemExit

print("Starting tests in 3 seconds. Ensure clear space around the robot.")
hold_state(3)

# Test 1: Drive Forward
print("Test 1: drive_forward")
robot.drive_forward(200, 200)
hold_state(2)
robot.brake()
hold_state(1)

# Test 2: Drive Backward
print("Test 2: drive_backward")
robot.drive_backward(200, 200)
hold_state(2)
robot.brake()
hold_state(1)

# Test 3: Rotate Right
print("Test 3: rotate_right")
robot.rotate_right(200)
hold_state(2)
robot.brake()
hold_state(1)

# Test 4: Rotate Left
print("Test 4: rotate_left")
robot.rotate_left(200)
hold_state(2)
robot.brake()
hold_state(1)

# Test 5: Ultrasonic Sensors
print("Test 5: ultrasonic distance readings (sensor 1 + sensor 2)")
for i in range(5):
    distance_1 = robot.read_distance()
    distance_2 = robot.read_distance_2()
    print(
        "Reading", i + 1, "- Sensor 1:", distance_1, "mm | Sensor 2:", distance_2, "mm"
    )
    hold_state(0.5)

print("All hardware tests completed.")

