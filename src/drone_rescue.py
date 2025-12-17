import WebGUI
import HAL
import Frequency
import cv2
import math
import time
import numpy as np


def get_original_coords(x, y, w, h, angle, img_w, img_h):
    if angle == 0:
        return int(x), int(y), int(w), int(h)
    elif angle == 90:
        new_x = y
        new_y = img_h - x - w
        return int(new_x), int(new_y), int(h), int(w)
    elif angle == 180:
        new_x = img_w - x - w
        new_y = img_h - y - h
        return int(new_x), int(new_y), int(w), int(h)
    elif angle == 270:
        new_x = img_w - y - h
        new_y = x
        return int(new_x), int(new_y), int(h), int(w)
    return int(x), int(y), int(w), int(h)


def detect_faces_with_rotation(image, face_cascade):
    angles = [0, 90, 180, 270]
    h, w = image.shape[:2]
    faces_all = []

    for angle in angles:
        if angle == 0:
            rotated = image
        elif angle == 90:
            rotated = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        elif angle == 180:
            rotated = cv2.rotate(image, cv2.ROTATE_180)
        elif angle == 270:
            rotated = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

        gray = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)

        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.05,
            minNeighbors=3,
            minSize=(20, 20)
        )

        for (x, y, fw, fh) in faces:
            rx, ry, rw, rh = get_original_coords(x, y, fw, fh, angle, w, h)
            faces_all.append((rx, ry, rw, rh))

    return faces_all


TRACKER_CENTROID_DIST_PX = 60      
TRACKER_TIMEOUT = 1.5             
MIN_SEEN_FRAMES = 2                
MIN_DIST_BETWEEN_PEOPLE = 5.0      
GLOBAL_LOG_COOLDOWN = 1.0          


altitude = 4
HAL.takeoff(altitude)

img_down = HAL.get_ventral_image()
img_front = HAL.get_frontal_image()

WebGUI.showImage(img_down)
WebGUI.showLeftImage(img_front)



dronePos = HAL.get_position()
dronePosX = dronePos[0]
dronePosY = dronePos[1]
droneYAW = HAL.get_yaw()

# Coordenadas del centro de búsqueda
newLocX = 32
newLocY = -35
newLocZ = altitude

targetDistance = math.sqrt((newLocX-dronePosX)**2 + (newLocY-dronePosY)**2)
newYawAngle = math.atan2(newLocY-dronePosY, newLocX-dronePosX)

# Orientarse
HAL.set_cmd_pos(dronePosX, dronePosY, altitude, newYawAngle)
while abs(newYawAngle - droneYAW) > 0.1:
    droneYAW = HAL.get_yaw()
    Frequency.tick()

# Viajar
speed_travel = 8

while targetDistance > 3:
    dronePos = HAL.get_position()
    dronePosX = dronePos[0]
    dronePosY = dronePos[1]
    droneAltitude = dronePos[2]

    angle_to_target = math.atan2(newLocY-dronePosY, newLocX-dronePosX)
    yaw_error = angle_to_target - HAL.get_yaw()
    if yaw_error > math.pi: yaw_error -= 2*math.pi
    if yaw_error < -math.pi: yaw_error += 2*math.pi

    climbRate = 0.8 * (altitude-droneAltitude)
    HAL.set_cmd_vel(speed_travel, 0, climbRate, yaw_error)

    targetDistance = math.sqrt((newLocX-dronePosX)**2 + (newLocY-dronePosY)**2)
    WebGUI.showImage(HAL.get_ventral_image())
    Frequency.tick()


HAL.set_cmd_vel(0, 0, 0, 0)
print("Zona alcanzada.")

search_altitude = 4.0
HAL.set_cmd_pos(newLocX, newLocY, search_altitude, 0)

while True:
    if abs(HAL.get_position()[2] - search_altitude) < 0.3:
        break
    Frequency.tick()



face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

found_people_coords = []
TOTAL_PEOPLE = 6

linear_speed = 1.2
current_radius = 0.5
MAX_RADIUS = 25.0

spiral_center = (newLocX, newLocY)

tracked_faces = []
next_track_id = 0
last_log_time = 0.0

print(f"INICIANDO BARRIDO A {search_altitude} METROS...")

while len(found_people_coords) < TOTAL_PEOPLE:
    img_down = HAL.get_ventral_image()
    img_debug = img_down.copy()
    img_h, img_w = img_debug.shape[:2]

    faces = detect_faces_with_rotation(img_down, face_cascade)  
    for (x, y, w, h) in faces:
        cv2.rectangle(img_debug, (x, y), (x + w, y + h), (255, 0, 0), 1)


    now = time.time()
    detections_centroids = []
    for (x, y, w, h) in faces:
        cx = x + w/2
        cy = y + h/2
        detections_centroids.append({'bbox': (x, y, w, h), 'centroid': (cx, cy), 'matched': False})


    for det in detections_centroids:
        cx, cy = det['centroid']
        best_track = None
        best_dist = None
        for track in tracked_faces:
            tx, ty = track['centroid']
            dist = math.hypot(cx - tx, cy - ty)
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_track = track

        if best_track is not None and best_dist is not None and best_dist < TRACKER_CENTROID_DIST_PX:
            best_track['bbox'] = det['bbox']
            best_track['centroid'] = det['centroid']
            best_track['last_seen'] = now
            best_track['seen_count'] += 1
            det['matched'] = True
        else:
            tracked_faces.append({
                'id': next_track_id,
                'bbox': det['bbox'],
                'centroid': det['centroid'],
                'first_seen': now,
                'last_seen': now,
                'seen_count': 1,
                'logged': False
            })
            next_track_id += 1
            det['matched'] = True

    tracked_faces = [t for t in tracked_faces if now - t['last_seen'] <= TRACKER_TIMEOUT]

    for track in tracked_faces:
        if track['logged']:
            continue

    
        if track['seen_count'] >= MIN_SEEN_FRAMES:
           
            if now - last_log_time < GLOBAL_LOG_COOLDOWN:
                continue
            current_pos = HAL.get_position()
            px, py = current_pos[0], current_pos[1]

        
            is_new_person = True
            for p in found_people_coords:
                dist = math.hypot(px - p[0], py - p[1])
                if dist < MIN_DIST_BETWEEN_PEOPLE:
                    is_new_person = False
                    break

        
            if is_new_person:
                found_people_coords.append((px, py))
                last_log_time = now
                track['logged'] = True
                print(f"¡NUEVA PERSONA VALIDADA! ({len(found_people_coords)} de {TOTAL_PEOPLE})")
                print(f"Posición dron: X:{round(px,1)} Y:{round(py,1)}")
            else:
                track['logged'] = True

    
    for track in tracked_faces:
        x, y, w, h = track['bbox']
        cx, cy = track['centroid']
        color = (0, 255, 0) if track['logged'] else (0, 200, 255)
        cv2.rectangle(img_debug, (int(x), int(y)), (int(x + w), int(y + h)), color, 2)
        cv2.putText(img_debug, f"id:{track['id']} s:{track['seen_count']}", (int(x), int(y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)


    WebGUI.showImage(img_debug)

    
    current_pos = HAL.get_position()
    distance_from_center = math.sqrt(
        (current_pos[0] - spiral_center[0])**2 +
        (current_pos[1] - spiral_center[1])**2
    )

    if distance_from_center > MAX_RADIUS:
        print(f"Límite alcanzado. Reiniciando espiral...")
        HAL.set_cmd_pos(spiral_center[0], spiral_center[1], search_altitude, 0)
        time.sleep(3)
        current_radius = 0.2
    else:
        current_z = HAL.get_position()[2]
        fix_z_speed = 1.0 * (search_altitude - current_z)  # Corrector de altura

        # evitar división por cero (si current_radius muy pequeño)
        yaw_rate = 0.0
        if current_radius > 0.01:
            yaw_rate = -1 * (linear_speed / current_radius)
        HAL.set_cmd_vel(linear_speed, 0, fix_z_speed, yaw_rate)

        current_radius += 0.001

    Frequency.tick()

print(f"¡Búsqueda completada! {len(found_people_coords)} personas encontradas")
HAL.set_cmd_vel(0, 0, 0, 0)
HAL.land()
