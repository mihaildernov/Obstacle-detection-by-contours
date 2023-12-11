import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
from dronekit import *
from pymavlink import mavutil
import cv2


master = mavutil.mavlink_connection('/dev/ttyACM0')
vehicle = connect('/dev/ttyACM0', wait_ready=True)

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("Ждем моторы...")
    time.sleep(1)

def circle(duration):
    vehicle.channels.overrides['1'] = 1250
    vehicle.channels.overrides['3'] = 1250

def move_forward(duration):
    vehicle.channels.overrides['3'] = 1500
    vehicle.channels.overrides['1'] = 1500

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    edges = cv2.Canny(blur, 70, 35)

    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        area = (w - x) * (h - y)

        if area > 275000:
            print("Обнаружено препятствие")
            print("Turn")
            circle(0.5)
        else:
            print("Move Forward")
            move_forward(0.5)

    cv2.imshow('Obstacle Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
