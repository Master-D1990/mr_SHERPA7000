import cv2
import numpy as np
import glob
import os

check_dir = 'calibration_check'
if not os.path.exists(check_dir):
    os.makedirs(check_dir)

# Konfiguration
chessboard_size = (9, 6)        # Innere Ecken (nicht Felder!)
square_size = 23.5              # In mm (physische Größe des Schachbretts!)

objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1,2) * square_size

obj_points = []
img_points = []
images = glob.glob('calibration_images/*.jpg')

print(f"Verarbeite {len(images)} Bilder...")
for i, image_path in enumerate(images):
    img = cv2.imread(image_path)
    if img is None:
        continue
        
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(
        gray, 
        chessboard_size, 
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    
    if ret:
        obj_points.append(objp)
        corners_refined = cv2.cornerSubPix(
            gray, 
            corners, 
            (11,11), 
            (-1,-1), 
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        img_points.append(corners_refined)
        
        # Speichere Bilder mit erkannten Ecken
        cv2.drawChessboardCorners(img, chessboard_size, corners_refined, ret)
        cv2.imwrite(f'{check_dir}/corners_{i:03d}.jpg', img)
    else:
        print(f"Fehler in {os.path.basename(image_path)}")

# Kalibrierung
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, 
    img_points, 
    gray.shape[::-1],  # Automatische Größenermittlung (1920x1080)
    None, 
    None
)

# Ergebnisse
print(f"\nKameramatrix (1080p):\n{mtx}")
print(f"\nVerzerrungskoeffizienten:\n{dist.ravel()}")

# Speichern
np.savez('calibration_data.npz', 
         camera_matrix=mtx, 
         dist_coeff=dist,
         resolution=gray.shape[::-1])

# Beispielbild entzerren
if len(images) > 0:
    test_img = cv2.imread(images[0])
    h, w = test_img.shape[:2]
    new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    undistorted = cv2.undistort(test_img, mtx, dist, None, new_mtx)
    cv2.imwrite(f"{check_dir}/undistorted.jpg", undistorted)

print("\nKalibrierung abgeschlossen!")