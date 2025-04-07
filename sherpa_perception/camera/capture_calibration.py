import cv2
import os
import time
import numpy as np
import subprocess
import threading
import signal
import sys

# Einstellungen
CALIB_IMG_DIR = "calibration_images"
SAVE_DELAY = 1  # Sekunden zwischen den Aufnahmen
RESOLUTION = (1920, 1080)  # 1080p Auflösung (statt 1280x960)

# Überprüfe Ordner
if not os.path.exists(CALIB_IMG_DIR):
    os.makedirs(CALIB_IMG_DIR)

# Globale Variablen
img_counter = 0
last_save_time = 0
running = True
preview_process = None

def start_preview():
    global preview_process
    preview_process = subprocess.Popen([
        'libcamera-hello',
        '--qt-preview',
        '--width', str(RESOLUTION[0]),
        '--height', str(RESOLUTION[1]),
        '--timeout', '0'
    ])
    print("Kamera-Vorschau gestartet (1080p)")

def capture_image():
    global preview_process
    if preview_process:
        preview_process.terminate()
        preview_process.wait()
    
    import tempfile
    temp_file = tempfile.NamedTemporaryFile(suffix='.jpg', delete=False)
    temp_file.close()
    
    try:
        subprocess.run([
            'libcamera-still',
            '--width', str(RESOLUTION[0]),
            '--height', str(RESOLUTION[1]),
            '-o', temp_file.name,
            '-n',
            '-t', '1'
        ], check=True)
        
        img = cv2.imread(temp_file.name)
        os.unlink(temp_file.name)
        start_preview()
        return img
    except subprocess.CalledProcessError as e:
        print(f"Fehler: {e}")
        start_preview()
        if os.path.exists(temp_file.name):
            os.unlink(temp_file.name)
        return None

def save_image(frame):
    global img_counter
    if frame is None:
        return
    
    img_name = f"{CALIB_IMG_DIR}/calib_{img_counter:03d}.jpg"
    cv2.imwrite(img_name, frame)
    print(f"Bild {img_counter} gespeichert")
    img_counter += 1

def keyboard_monitor():
    global running, last_save_time
    print("\nDrücke LEERTASTE für Aufnahme, ESC/q zum Beenden...")
    
    while running:
        key = cv2.waitKey(100) & 0xFF
        if key == 32:  # LEERTASTE
            current_time = time.time()
            if current_time - last_save_time >= SAVE_DELAY:
                frame = capture_image()
                save_image(frame)
                last_save_time = current_time
            else:
                remaining = int(SAVE_DELAY - (current_time - last_save_time))
                print(f"Warte {remaining}s...")
        elif key == 27 or key == ord('q'):
            running = False

def cleanup_and_exit():
    global preview_process, running
    running = False
    if preview_process:
        preview_process.terminate()
        preview_process.wait()
    cv2.destroyAllWindows()
    print("Programm beendet.")
    sys.exit(0)

def signal_handler(sig, frame):
    print("\nBeende Programm (STRG+C)...")
    cleanup_and_exit()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    print("\nKamerakalibrierung - 1080p-Modus")
    print("===============================")
    
    # Hilfsfenster für Tastatur
    cv2.namedWindow('Steuerung', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Steuerung', 400, 100)
    info_img = np.zeros((100, 400, 3), dtype=np.uint8)
    cv2.putText(info_img, "LEERTASTE: Aufnahme", (10, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    cv2.putText(info_img, "ESC/q: Beenden", (10, 60), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    cv2.imshow('Steuerung', info_img)
    
    try:
        start_preview()
        keyboard_thread = threading.Thread(target=keyboard_monitor)
        keyboard_thread.daemon = True
        keyboard_thread.start()
        keyboard_thread.join()
    except Exception as e:
        print(f"Fehler: {e}")
    finally:
        cleanup_and_exit()