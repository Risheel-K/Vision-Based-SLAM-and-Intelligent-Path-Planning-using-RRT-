import cv2
import torch
import numpy as np
import time
import math
import random
import RPi.GPIO as GPIO

# === Motor GPIO Setup (Single Motor Driver) ===
RPWM = 7
LPWM = 8
REN = 9
LEN = 10

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in [RPWM, LPWM, REN, LEN]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

GPIO.output(REN, GPIO.HIGH)
GPIO.output(LEN, GPIO.HIGH)

pwm_freq = 1000
forward_speed = 30
turn_speed = 20

pwm_RPWM = GPIO.PWM(RPWM, pwm_freq)
pwm_LPWM = GPIO.PWM(LPWM, pwm_freq)

pwm_RPWM.start(0)
pwm_LPWM.start(0)

def set_motor(left_dir, right_dir):
    # Simulate left and right motor through PWM split (duty cycle emulation)
    # Left motor = RPWM, Right motor = LPWM
    pwm_RPWM.ChangeDutyCycle(left_dir)
    pwm_LPWM.ChangeDutyCycle(right_dir)

def send_command(cmd):
    if cmd == 'f':       # Forward
        set_motor(forward_speed, forward_speed)
    elif cmd == 'b':     # Backward
        set_motor(0, 0)  # Your setup may not support reverse on shared PWM
    elif cmd == 'l':     # Turn Left
        set_motor(0, turn_speed)
    elif cmd == 'r':     # Turn Right
        set_motor(turn_speed, 0)
    elif cmd == 's':     # Stop
        set_motor(0, 0)
    print(f"[CMD] Sent: {cmd}")

# === YOLOv5 Setup ===
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
model = model.to('cpu')
model.conf = 0.5

# === SLAM Map Setup ===
MAP_SIZE = 600
OBSTACLE_COUNT = 30
OBSTACLE_SIZE = 20
ROBOT_RADIUS = 8
robot_pos = [MAP_SIZE // 2, MAP_SIZE // 2]
destination_locked = False
destination = None
obstacles = []

def load_map():
    img = np.ones((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8) * 255
    obstacles.clear()
    for _ in range(OBSTACLE_COUNT):
        x = random.randint(0, MAP_SIZE - OBSTACLE_SIZE)
        y = random.randint(0, MAP_SIZE - OBSTACLE_SIZE)
        obstacles.append((x, y))
        cv2.rectangle(img, (x, y), (x + OBSTACLE_SIZE, y + OBSTACLE_SIZE), (50, 50, 50), -1)
    return img

def is_collision(x, y):
    return any(ox <= x <= ox + OBSTACLE_SIZE and oy <= y <= oy + OBSTACLE_SIZE for ox, oy in obstacles)

def map_camera_to_world(x, y, w, h):
    return int((x / w) * MAP_SIZE), int((y / h) * MAP_SIZE)

def get_direction(p1, p2):
    return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))

def distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

map_img = load_map()
cap = cv2.VideoCapture(0)
prev_angle = 0

ROBOT_STEP_SIZE = 15
STOPPING_DISTANCE = 15
TURN_TOLERANCE = 30

obstacle_turn_counter = 0
last_forward_time = time.time()

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w = frame.shape[:2]
        map_disp = map_img.copy()
        results = model(frame)
        detections = results.xyxy[0]

        if not destination_locked:
            for det in detections:
                if model.names[int(det[5])] == 'person':
                    x1, y1, x2, y2 = map(int, det[:4])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    destination = map_camera_to_world(cx, cy, w, h)
                    destination_locked = True
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    print(f"[DESTINATION] Locked at: {destination}")
                    break

        if destination_locked:
            cv2.circle(map_disp, destination, 10, (0, 0, 255), -1)
            cv2.circle(map_disp, tuple(robot_pos), ROBOT_RADIUS, (255, 0, 0), -1)
            cv2.line(map_disp, tuple(robot_pos), destination, (0, 255, 0), 2)

            dist = distance(robot_pos, destination)
            angle_to_target = get_direction(robot_pos, destination)

            print(f"[DEBUG] Distance: {dist:.2f}, Robot pos: {robot_pos}, Destination: {destination}")

            if dist <= STOPPING_DISTANCE:
                send_command('s')
                print(f"[REACHED] Destination reached!")
            else:
                rad = math.radians(prev_angle)
                nx = int(robot_pos[0] + math.cos(rad) * ROBOT_STEP_SIZE)
                ny = int(robot_pos[1] + math.sin(rad) * ROBOT_STEP_SIZE)
                nx = max(ROBOT_RADIUS, min(MAP_SIZE - ROBOT_RADIUS, nx))
                ny = max(ROBOT_RADIUS, min(MAP_SIZE - ROBOT_RADIUS, ny))

                if not is_collision(nx, ny):
                    angle_diff = abs(((angle_to_target - prev_angle + 180) % 360) - 180)
                    if angle_diff > TURN_TOLERANCE:
                        turn_diff = ((angle_to_target - prev_angle + 180) % 360) - 180
                        if turn_diff > 0:
                            send_command('l')
                            prev_angle = (prev_angle + 20) % 360
                            print(f"[TURN] Left to align, angle: {prev_angle}")
                        else:
                            send_command('r')
                            prev_angle = (prev_angle - 20) % 360
                            print(f"[TURN] Right to align, angle: {prev_angle}")
                        obstacle_turn_counter = 0
                    else:
                        robot_pos[0], robot_pos[1] = nx, ny
                        send_command('f')
                        print(f"[MOVE] Forward to {robot_pos}")
                        obstacle_turn_counter = 0
                        last_forward_time = time.time()
                else:
                    send_command('l')
                    prev_angle = (prev_angle + 20) % 360
                    obstacle_turn_counter += 1
                    print(f"[OBSTACLE] Turning left, angle: {prev_angle}")
                    if obstacle_turn_counter > 8:
                        send_command('r')
                        prev_angle = (prev_angle - 40) % 360
                        obstacle_turn_counter = 0
                        print(f"[OBSTACLE] Switching direction, angle: {prev_angle}")

            cv2.putText(map_disp, f"Dist: {dist:.1f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            cv2.putText(map_disp, f"Angle: {prev_angle:.0f}", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            cv2.circle(map_disp, destination, STOPPING_DISTANCE, (0, 255, 255), 2)

        cv2.imshow("Camera", frame)
        cv2.imshow("SLAM Map", map_disp)
        time.sleep(0.1)
        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("[INTERRUPT] Keyboard interrupt received")
except Exception as e:
    print(f"[ERROR] {e}")
finally:
    send_command('s')
    cap.release()
    pwm_RPWM.stop()
    pwm_LPWM.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
    print("[SHUTDOWN] All cleaned up.")

