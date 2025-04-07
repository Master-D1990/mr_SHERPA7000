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

# Überprüfe ob Ordner existiert
if not os.path.exists(CALIB_IMG_DIR):
    os.makedirs(CALIB_IMG_DIR)

# Globale Variablen
img_counter = 0
last_save_time = 0
running = True
preview_process = None

# Funktion zum Starten des Kamera-Previews
def start_preview():
    global preview_process
    preview_process = subprocess.Popen([
        'libcamera-hello',
        '--qt-preview',
        '--width', '1280',
        '--height', '960',
        '--timeout', '0'  # Kein Timeout
    ])
    print("Kamera-Vorschau gestartet")

# Funktion zum Aufnehmen eines Bildes
def capture_image():
    # Stoppe kurz die Vorschau
    global preview_process
    if preview_process:
        preview_process.terminate()
        preview_process.wait()
    
    # Temporäre Datei für das Bild erstellen
    import tempfile
    temp_file = tempfile.NamedTemporaryFile(suffix='.jpg', delete=False)
    temp_file.close()
    
    try:
        # libcamera-still verwenden, um ein Bild aufzunehmen (ohne Vorschau)
        subprocess.run([
            'libcamera-still',
            '--width', '1280',
            '--height', '960',
            '-o', temp_file.name,
            '-n',  # Keine Vorschau
            '-t', '1'  # Minimale Verzögerung
        ], check=True)
        
        # Bild einlesen
        img = cv2.imread(temp_file.name)
        # Temporäre Datei löschen
        os.unlink(temp_file.name)
        
        # Starte die Vorschau wieder
        start_preview()
        
        return img
    except subprocess.CalledProcessError as e:
        print(f"Fehler beim Erfassen des Bildes: {e}")
        # Starte die Vorschau wieder
        start_preview()
        if os.path.exists(temp_file.name):
            os.unlink(temp_file.name)
        return None

# Funktion zur Speicherung des aufgenommenen Bildes
def save_image(frame):
    global img_counter
    
    if frame is None:
        print("Konnte kein Bild aufnehmen.")
        return
    
    # Bild speichern
    img_name = f"{CALIB_IMG_DIR}/calib_{img_counter:03d}.jpg"
    cv2.imwrite(img_name, frame)
    
    print(f"Bild {img_counter} als {img_name} gespeichert.")
    img_counter += 1

# Funktion zur Überwachung der Tastatureingabe
def keyboard_monitor():
    global running, last_save_time
    
    print("\nDrücke LEERTASTE für Aufnahme, ESC oder q zum Beenden...")
    
    while running:
        try:
            # Verwende cv2.waitKey für Tastendruck
            key = cv2.waitKey(100) & 0xFF
            
            if key == 32:  # LEERTASTE
                current_time = time.time()
                if current_time - last_save_time >= SAVE_DELAY:
                    print("Nehme Bild auf...")
                    frame = capture_image()
                    save_image(frame)
                    last_save_time = current_time
                else:
                    remaining = int(SAVE_DELAY - (current_time - last_save_time))
                    print(f"Bitte warte noch {remaining} Sekunden...")
            elif key == 27 or key == ord('q'):  # ESC oder q
                print("Beende Programm...")
                running = False
                break
        except Exception as e:
            print(f"Fehler bei Tastatureingabe: {e}")
    
    # Programm beenden
    cleanup_and_exit()

# Funktion zum sauberen Beenden des Programms
def cleanup_and_exit():
    global preview_process, running
    running = False
    
    if preview_process:
        try:
            preview_process.terminate()
            preview_process.wait()
        except:
            pass
    
    cv2.destroyAllWindows()
    print("Programm beendet.")
    sys.exit(0)

# Signal-Handler für STRG+C
def signal_handler(sig, frame):
    print("\nProgramm wird beendet (STRG+C)...")
    cleanup_and_exit()

# Hauptprogramm
if __name__ == "__main__":
    # Signal-Handler registrieren
    signal.signal(signal.SIGINT, signal_handler)
    
    print("\nKamerakalibrierung - Bildaufnahme")
    print("===============================")
    print(f"Bilder werden im Ordner '{CALIB_IMG_DIR}' gespeichert")
    
    # Erzeuge ein kleines Hilfsfenster für Tastaturereignisse
    cv2.namedWindow('Tastatur-Steuerung', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Tastatur-Steuerung', 400, 100)
    info_img = np.zeros((100, 400, 3), dtype=np.uint8)
    cv2.putText(info_img, "Drücke LEERTASTE für Aufnahme", (10, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    cv2.putText(info_img, "Drücke ESC oder q zum Beenden", (10, 60), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    cv2.imshow('Tastatur-Steuerung', info_img)
    
    try:
        # Kamera-Vorschau starten
        start_preview()
        
        # Tastatureingabe in separatem Thread überwachen
        keyboard_thread = threading.Thread(target=keyboard_monitor)
        keyboard_thread.daemon = True
        keyboard_thread.start()
        
        # Hauptthread wartet, bis das Programm beendet wird
        keyboard_thread.join()
        
    except Exception as e:
        print(f"\nFehler aufgetreten: {e}")
    finally:
        cleanup_and_exit()