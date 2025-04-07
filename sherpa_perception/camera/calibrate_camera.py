import cv2
import numpy as np
import glob
import os

# Erstelle Verzeichnis für Überprüfungsbilder
check_dir = 'calibration_check'
if not os.path.exists(check_dir):
    os.makedirs(check_dir)

# Konfiguration
chessboard_size = (9, 6)
square_size = 23.5  # Größe eines Quadrats in mm - HIER ANPASSEN!

print(f"Kalibriere mit 9x6 Schachbrett ({chessboard_size[0]}x{chessboard_size[1]} innere Ecken)")

# Vorbereitung der Objektpunkte mit realer Größe
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1,2) * square_size

# Speichern der 3D-Punkte und 2D-Bildpunkte
obj_points = []
img_points = []
images = glob.glob('calibration_images/*.jpg')

print(f"Gefundene Bilder: {len(images)}")
if len(images) == 0:
    print("Keine Bilder gefunden! Überprüfe den Pfad 'calibration_images/'")
    exit(1)

# Verwende die erfolgreiche Methode
detection_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

# Verarbeite alle Bilder
for i, image_path in enumerate(images):
    img = cv2.imread(image_path)
    if img is None:
        print(f"Konnte Bild nicht laden: {image_path}")
        continue
        
    # Prüfe tatsächliche Bildgröße
    if i == 0:
        frame_size = (img.shape[1], img.shape[0])
        print(f"Bildgröße: {frame_size}")
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Schachbrettmuster erkennen
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, detection_flags)
    
    if ret:
        obj_points.append(objp)
        # Verfeinere die Eckpunkte
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        img_points.append(corners_refined)
        
        # Zeichne die Ecken ein und speichere das Bild
        img_with_corners = img.copy()
        cv2.drawChessboardCorners(img_with_corners, chessboard_size, corners_refined, ret)
        check_path = f'{check_dir}/corners_{i:03d}.jpg'
        cv2.imwrite(check_path, img_with_corners)
        
        print(f"✓ Schachbrett erkannt in Bild {i+1}/{len(images)}: {image_path}")
    else:
        print(f"✗ Kein Schachbrett erkannt in Bild {i+1}/{len(images)}: {image_path}")
        
        # Versuche mit Weichzeichnung (hat in Tests funktioniert)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        ret, corners = cv2.findChessboardCorners(blur, chessboard_size, detection_flags)
        
        if ret:
            obj_points.append(objp)
            # Verfeinere die Eckpunkte (mit dem ursprünglichen Graustufenbild)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            img_points.append(corners_refined)
            
            # Zeichne die Ecken ein
            img_with_corners = img.copy()
            cv2.drawChessboardCorners(img_with_corners, chessboard_size, corners_refined, ret)
            check_path = f'{check_dir}/corners_blur_{i:03d}.jpg'
            cv2.imwrite(check_path, img_with_corners)
            
            print(f"✓ Schachbrett mit Blur-Methode erkannt in Bild {i+1}/{len(images)}: {image_path}")

print(f"\nSchachbretter erkannt in {len(obj_points)} von {len(images)} Bildern")

if len(obj_points) < 5:
    print("Zu wenige Schachbretter erkannt (mindestens 5 benötigt). Kalibrierung abgebrochen.")
    exit(1)

# Kamera-Kalibrierung
print("\nFühre Kalibrierung durch...")
ret, camera_matrix, dist_coeff, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, frame_size, None, None)

# Berechne Rückprojektionsfehler
total_error = 0
for i in range(len(obj_points)):
    imgpoints2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeff)
    error = cv2.norm(img_points[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    total_error += error

avg_error = total_error/len(obj_points)
print(f"Kalibrierung abgeschlossen mit durchschnittlichem Rückprojektionsfehler: {avg_error}")

# Speichern der Kalibrierungsdaten
calibration_file = 'calibration_data.npz'
np.savez(calibration_file, 
         camera_matrix=camera_matrix, 
         dist_coeff=dist_coeff,
         square_size_mm=square_size,
         avg_reprojection_error=avg_error)

# Zeige Ergebnisse
print("\nKamera-Matrix:")
print(camera_matrix)
print("\nVerzerrungskoeffizienten:")
print(dist_coeff.ravel())

print(f"\nKalibrationsdaten wurden in '{calibration_file}' gespeichert.")
print(f"Bilder mit erkanntem Schachbrett wurden in '{check_dir}/' gespeichert.")

# Erzeuge ein entzerrtes Beispielbild
if len(images) > 0:
    test_img = cv2.imread(images[0])
    h, w = test_img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeff, (w,h), 1, (w,h))
    
    # Entzerrung
    dst = cv2.undistort(test_img, camera_matrix, dist_coeff, None, newcameramtx)
    
    # Schneide Bereich zu und speichere
    x, y, w, h = roi
    if w > 0 and h > 0:
        dst = dst[y:y+h, x:x+w]
    cv2.imwrite(f"{check_dir}/undistorted.jpg", dst)
    
    # Speichere auch Original zum Vergleich
    cv2.imwrite(f"{check_dir}/original_for_comparison.jpg", test_img)
    
    print(f"Entzerrtes Beispielbild gespeichert als '{check_dir}/undistorted.jpg'")
    print(f"Original zum Vergleich gespeichert als '{check_dir}/original_for_comparison.jpg'")

print("\nKalibrierung abgeschlossen.")