#!/usr/bin/env python3
"""
Einfaches Programm zur Echtzeit-ArUco-Marker-Erkennung mit einer Raspberry Pi Kamera.
"""

import cv2
import numpy as np
import os
import subprocess
import tempfile

# Konfigurationsparameter
MARKER_SIZE = 0.10  # Markergröße in Metern (10 cm)


def create_camera_matrix():
    """Erstellt und gibt die Kamera-Kalibrierungsmatrix zurück.

    Returns:
        tuple: Kameramatrix und Verzerrungskoeffizienten
    """
    camera_matrix = np.array([
        [800.0, 0.0, 320.0],
        [0.0, 800.0, 240.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)

    dist_coeff = np.zeros((1, 5), dtype=np.float32)
    
    return camera_matrix, dist_coeff


def start_preview():
    """Startet die Kamera-Vorschau mit libcamera-hello.

    Returns:
        subprocess.Popen: Der gestartete Vorschau-Prozess
    """
    preview_process = subprocess.Popen([
        'libcamera-hello',
        '--qt-preview',
        '--width', '1920',
        '--height', '1080',
        '--timeout', '0'
    ])
    print("Kamera-Vorschau gestartet")
    return preview_process


def capture_image():
    """Nimmt ein Bild mit libcamera-still auf.

    Returns:
        numpy.ndarray: Das aufgenommene Bild oder None bei einem Fehler
    """
    temp_file = tempfile.NamedTemporaryFile(suffix='.jpg', delete=False)
    temp_file.close()
    
    try:
        subprocess.run([
            'libcamera-still',
            '--width', '1920',
            '--height', '1080',
            '-o', temp_file.name,
            '-n',
            '-t', '1'
        ], stderr=subprocess.DEVNULL)
        
        # Bild einlesen
        frame = cv2.imread(temp_file.name)
        os.unlink(temp_file.name)
        
        return frame
    except Exception as e:
        print(f"Fehler beim Aufnehmen des Bildes: {e}")
        if os.path.exists(temp_file.name):
            os.unlink(temp_file.name)
        return None


def detect_markers(frame, aruco_dict, parameters, camera_matrix, dist_coeff):
    """Erkennt ArUco-Marker im Bild und zeichnet sie ein.

    Args:
        frame: Das Eingabebild
        aruco_dict: Das ArUco-Dictionary
        parameters: Die Erkennungsparameter
        camera_matrix: Die Kamera-Kalibrierungsmatrix
        dist_coeff: Die Verzerrungskoeffizienten

    Returns:
        numpy.ndarray: Das Bild mit eingezeichneten Markern
    """
    if frame is None:
        return None
        
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    # Erkannte Marker anzeigen
    if ids is not None:
        # Marker zeichnen
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Position für jeden Marker
        for i in range(len(ids)):
            try:
                rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], MARKER_SIZE, camera_matrix, dist_coeff)
                cv2.aruco.drawAxis(frame, camera_matrix, dist_coeff, rvec, tvec, 0.1)
                
                # Position anzeigen
                marker_id = ids[i][0]
                opencv_x, opencv_y, opencv_z = tvec[0][0]
                
                # Rotationsmatrix aus Rodrigues-Vektor
                rot_matrix, _ = cv2.Rodrigues(rvec[0])
                
                # Roboterkoordinaten basierend auf OpenCV-Koordinaten
                robot_x = opencv_z  # Tiefe zur Kamera
                robot_y = opencv_x  # Seitlicher Abstand
                
                # Marker-Achsen im Kamera-Koordinatensystem
                marker_z = rot_matrix[:, 2]  # Z-Achse des Markers = Wandnormale
                
                # Projektion der Marker-Z-Achse auf die XZ-Ebene (horizontale Ebene)
                marker_z_xz = np.array([marker_z[0], 0, marker_z[2]])
                
                # Normalisieren des projizierten Vektors
                if np.linalg.norm(marker_z_xz) > 0:
                    marker_z_xz = marker_z_xz / np.linalg.norm(marker_z_xz)
                else:
                    marker_z_xz = np.array([0, 0, -1])  # Fallback (Marker auf gerade Wand)
                
                # Referenzvektor: invertierte Z-Achse der Kamera (= Richtung zur Wand mit Marker)
                camera_z_inv = np.array([0, 0, -1])
                
                # Winkelberechnung zwischen diesen Vektoren
                dot_product = np.dot(camera_z_inv, marker_z_xz)
                angle_rad = np.arccos(np.clip(dot_product, -1.0, 1.0))
                angle_deg = angle_rad * 180 / np.pi
                
                # Vorzeichen des Winkels: negativ wenn Wand nach links gedreht
                if marker_z_xz[0] > 0:
                    angle_deg = -angle_deg
                
                # Bei einer geraden Wand (Kamera rechtwinklig zur Wand) sollte theta = 0 sein
                # Bei einer nach links gedrehten Wand: theta < 0
                # Bei einer nach rechts gedrehten Wand: theta > 0
                
                print(f"ID {marker_id}: x={robot_x:.2f}m (Tiefe), y={robot_y:.2f}m (seitlich), theta={angle_deg:.2f}°")
            except Exception as e:
                print(f"Positionsberechnung fehlgeschlagen: {e}")
    
    return frame


def main():
    """Hauptfunktion für die ArUco-Marker-Erkennung."""
    # OpenCV-Version anzeigen
    print(f"OpenCV Version: {cv2.__version__}")
    print(f"Verwendete Markergröße: {MARKER_SIZE}m")

    # ArUco-Setup
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    parameters = cv2.aruco.DetectorParameters_create()

    # Kamerakalibrierung
    camera_matrix, dist_coeff = create_camera_matrix()
    
    # Versuche Kalibrierungsdaten zu laden
    calib_file = os.path.join('/home/ros/catkin_ws/src/sherpa/sherpa_perception/camera', 
                                'calibration_data.npz')
    if os.path.exists(calib_file):
        try:
            calib_data = np.load(calib_file)
            camera_matrix = calib_data['camera_matrix']
            dist_coeff = calib_data['dist_coeff']
            print(f"Kalibrierungsdaten aus {calib_file} geladen")
        except Exception as e:
            print(f"Fehler beim Laden der Kalibrierungsdaten: {e}")
    else:
        print(f"Kalibrierungsdatei nicht gefunden: {calib_file}")

    # Vorschau starten
    preview_process = start_preview()

    # Erkennungsfenster
    cv2.namedWindow("ArUco-Erkennung", cv2.WINDOW_NORMAL)
    # Bildschirmposition und Größe anpassen
    cv2.moveWindow("ArUco-Erkennung", 20, 20)
    cv2.resizeWindow("ArUco-Erkennung", 640, 480)

    print("ArUco-Erkennung läuft. Drücke 'q' zum Beenden.")

    try:
        while True:
            # Bild aufnehmen
            frame = capture_image()
            
            # Marker erkennen
            result_frame = detect_markers(frame, aruco_dict, parameters, 
                                         camera_matrix, dist_coeff)
            
            if result_frame is not None:
                # Bild anzeigen
                cv2.imshow("ArUco-Erkennung", result_frame)
            
            # Tastendruck prüfen
            key = cv2.waitKey(10) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        print("Programm unterbrochen")
    except Exception as e:
        print(f"Fehler: {e}")
    finally:
        # Aufräumen
        preview_process.terminate()
        cv2.destroyAllWindows()
        print("Programm beendet")


if __name__ == "__main__":
    main()