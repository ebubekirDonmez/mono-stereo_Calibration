import cv2
import numpy as np
import time

# Kalibrasyon dosyasının yolu (ana dizinde olduğunu varsayıyorum)
calibration_file = r"C:\Users\donme\Downloads\sunta_detection.v3-roboflow-instant-2--eval-.yolov5pytorch\stereo_calibration_data.xml"
fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)

# Matrisleri yükle
cameraMatrix_left = fs.getNode("cameraMatrix_left").mat()
distCoeffs_left = fs.getNode("distCoeffs_left").mat()
cameraMatrix_right = fs.getNode("cameraMatrix_right").mat()
distCoeffs_right = fs.getNode("distCoeffs_right").mat()
fs.release()

# Hata kontrolü
if cameraMatrix_left is None or distCoeffs_left is None or cameraMatrix_right is None or distCoeffs_right is None:
    print("Kalibrasyon verileri yüklenemedi!")
    exit()

# Kamera başlat
cap_left = cv2.VideoCapture(0)
cap_right = cv2.VideoCapture(2)

if not cap_left.isOpened() or not cap_right.isOpened():
    print("Kamera açılamadı.")
    exit()

# Pencere boyutları
window_width = 600
window_height = 600

cv2.namedWindow("Stereo Duzeltme", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Stereo Duzeltme", window_width * 2, window_height) # Yan yana iki görüntü
cv2.moveWindow("Stereo Duzeltme", 0, 0)

print("Görüntüler alınıyor. Çıkmak için 'q' tuşuna basın.")

while True:
    ret_left, frame_left = cap_left.read()
    ret_right, frame_right = cap_right.read()
    if not ret_left or not ret_right:
        print("Kare okunamadı.")
        break

    # Orijinal görüntüleri oran koruyarak yeniden boyutlandır
    frame_left_resized = cv2.resize(frame_left, (window_width, window_width))
    frame_right_resized = cv2.resize(frame_right, (window_width, window_width))

    # Düzeltme işlemi
    h, w = frame_left.shape[:2]
    newCamMatrix_left, roi_left = cv2.getOptimalNewCameraMatrix(cameraMatrix_left, distCoeffs_left, (w, h), 0.2, (w, h))
    undistorted_left = cv2.undistort(frame_left, cameraMatrix_left, distCoeffs_left, None, newCamMatrix_left)
    undistorted_left_resized = cv2.resize(undistorted_left, (window_width, window_width))

    h, w = frame_right.shape[:2]
    newCamMatrix_right, roi_right = cv2.getOptimalNewCameraMatrix(cameraMatrix_right, distCoeffs_right, (w, h), 0.2, (w, h))
    undistorted_right = cv2.undistort(frame_right, cameraMatrix_right, distCoeffs_right, None, newCamMatrix_right)
    undistorted_right_resized = cv2.resize(undistorted_right, (window_width, window_width))

    # Başlık ekle
    cv2.putText(frame_left_resized, "SOL ORIJINAL", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.putText(frame_right_resized, "SAG ORIJINAL", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.putText(undistorted_left_resized, "SOL DUZELTILMIS", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(undistorted_right_resized, "SAG DUZELTILMIS", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Yan yana birleştir
    originals = np.hstack((frame_left_resized, frame_right_resized))
    undistorteds = np.hstack((undistorted_left_resized, undistorted_right_resized))

    cv2.imshow("Orijinaller", originals)
    cv2.imshow("Duzeltilmisler", undistorteds)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()
