from controller import Robot
import numpy as np
import cv2

# Time in [ms] of a simulation step
TIME_STEP = 64

# Create the Robot instance
robot = Robot()

# Initialize distance sensors
ds_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
ds = []
for name in ds_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    ds.append(sensor)

# Initialize motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))  # Velocity control mode
right_motor.setPosition(float('inf'))

# Set initial speed
MAX_SPEED = 6.28

# ✅ Enable the camera
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)

# Main loop
while robot.step(TIME_STEP) != -1:
    # Read sensor values
    sensor_values = [sensor.getValue() for sensor in ds]

    # Obstacle detection
    right_obstacle = sensor_values[0] > 80.0 or sensor_values[1] > 80.0 or sensor_values[2] > 80.0
    left_obstacle = sensor_values[5] > 80.0 or sensor_values[6] > 80.0 or sensor_values[7] > 80.0

    # Set default speeds
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED

    if left_obstacle:
        print("↪️ Obstacle on the left, turning right")
        left_speed = 0.5 * MAX_SPEED
        right_speed = -0.5 * MAX_SPEED
    elif right_obstacle:
        print("↩️ Obstacle on the right, turning left")
        left_speed = -0.5 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED
    else:
        print("🚶 Path clear, moving forward")

    # Apply speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # ✅ Camera processing (optional)
    image = camera.getImage()
    if image:
        width = camera.getWidth()
        height = camera.getHeight()
        image_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
        bgr_image = cv2.cvtColor(image_array, cv2.COLOR_BGRA2BGR)

        # Example: convert to HSV for future color detection
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # You could add color blob detection or line following logic here

        # (Optional) Display image for debugging
        # cv2.imshow("e-puck Camera", bgr_image)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
#
