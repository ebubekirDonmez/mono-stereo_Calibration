

import numpy as np
import cv2 as cv
import os
import time

# Ayarlar
chessboard_size = (18, 12)  # Satranç tahtası iç köşe sayısı
square_size = 20            # Her karenin kenar uzunluğu (mm)
required_samples = 75     # Stereo için yeterli görüntü sayısı

# Satranç tahtası objesi oluştur
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []      # 3D gerçek dünya noktaları
imgpoints_left = [] # Sol kamera 2D noktaları
imgpoints_right = []# Sağ kamera 2D noktaları
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Sadece USB kameraları kullan (ID 1 ve 2)
# Sadece USB kameraları kullan (ID 1 ve 2)
cap_right = cv.VideoCapture(2)
cap_left = cv.VideoCapture(0)

# Kamera ayarlarını manuel yap: parlaklığı düşür, pozlamayı düzelt
# Sol kamera için ayarlar (cap_left)
cap_left.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manuel moda al
cap_left.set(cv.CAP_PROP_EXPOSURE, -7)         # Daha karanlık, -6 veya -8 de denenebilir
cap_left.set(cv.CAP_PROP_AUTO_WB, 0)           # Otomatik beyaz dengeyi kapat
cap_left.set(cv.CAP_PROP_WB_TEMPERATURE, 5000) # 4500–5500 ideal

# Sağ kamera için ayarlar (cap_right)
cap_right.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap_right.set(cv.CAP_PROP_EXPOSURE, -7)
cap_right.set(cv.CAP_PROP_AUTO_WB, 0)
cap_right.set(cv.CAP_PROP_WB_TEMPERATURE, 5000)


if not cap_left.isOpened() or not cap_right.isOpened():
    print("Kameralar açılamadı! Sadece USB kameralar kullanılacak şekilde ayarlandı (ID 1 ve 2).")
    exit()

# Kamera çözünürlüklerini al ve ekrana yazdır
frame_width_left = int(cap_left.get(cv.CAP_PROP_FRAME_WIDTH))
frame_height_left = int(cap_left.get(cv.CAP_PROP_FRAME_HEIGHT))
frame_width_right = int(cap_right.get(cv.CAP_PROP_FRAME_WIDTH))
frame_height_right = int(cap_right.get(cv.CAP_PROP_FRAME_HEIGHT))
print(f"Sol kamera çözünürlüğü: {frame_width_left}x{frame_height_left}")
print(f"Sağ kamera çözünürlüğü: {frame_width_right}x{frame_height_right}")

print("Stereo kalibrasyon başlatılıyor...")
print("Satranç tahtasını iki kameraya da gösterin ve 'q' ile çıkın.")

try:
    last_capture_time = 0  # döngüden önce tanımla
    capture_interval = 1.0  # saniye cinsinden, istersen artırabilirsin
    while True:
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()
        if not ret_left or not ret_right:
            print("Kare okunamadı. Çıkılıyor.")
            break

        # Her iki kameradan alınan görüntüyü x ekseninde (yatayda) çevir
        frame_left = cv.flip(frame_left, 1)
        frame_right = cv.flip(frame_right, 1)

        gray_left = cv.cvtColor(frame_left, cv.COLOR_BGR2GRAY)
        gray_right = cv.cvtColor(frame_right, cv.COLOR_BGR2GRAY)

        ret_corners_left, corners_left = cv.findChessboardCorners(gray_left, chessboard_size, None)
        ret_corners_right, corners_right = cv.findChessboardCorners(gray_right, chessboard_size, None)

        current_time = time.time()
        if ret_corners_left and ret_corners_right and (current_time - last_capture_time > capture_interval):
            corners_left_refined = cv.cornerSubPix(gray_left, corners_left, (11,11), (-1,-1), criteria)
            corners_right_refined = cv.cornerSubPix(gray_right, corners_right, (11,11), (-1,-1), criteria)
            objpoints.append(objp)
            imgpoints_left.append(corners_left_refined)
            imgpoints_right.append(corners_right_refined)
            last_capture_time = current_time

            # Köşeleri çiz
            cv.drawChessboardCorners(frame_left, chessboard_size, corners_left_refined, ret_corners_left)
            cv.drawChessboardCorners(frame_right, chessboard_size, corners_right_refined, ret_corners_right)
            print(f"{len(objpoints)} stereo görüntü kaydedildi")

        # Görüntüleri yan yana göster
        combined = np.hstack((frame_left, frame_right))
        cv.imshow("Stereo Kalibrasyon (Sol | Sağ)", combined)

        # Çıkmak için 'q' tuşu
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

        # Yeterli örnek toplandıysa otomatik çık
        if len(objpoints) >= required_samples:
            print("Yeterli veri toplandı.")
            break

except Exception as e:
    print(f"Bir hata oluştu: {e}")
finally:
    cap_left.release()
    cap_right.release()
    cv.destroyAllWindows()

def save_stereo_maps_to_xml(filename, stereo_maps):
    """Stereo harita verilerini XML formatında kaydet"""
    fs = cv.FileStorage(filename, cv.FILE_STORAGE_WRITE)
    for key, mat in stereo_maps.items():
        if isinstance(mat, (np.ndarray, np.generic)):
            fs.write(key, mat)
    fs.release()

if len(objpoints) > 0:
    print("Önce her iki kamera için ayrı ayrı kalibrasyon yapılıyor...")
    ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv.calibrateCamera(
        objpoints, imgpoints_left, gray_left.shape[::-1], None, None)
    ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv.calibrateCamera(
        objpoints, imgpoints_right, gray_right.shape[::-1], None, None)

    print("Stereo kalibrasyon yapılıyor...")
    flags = 0
    stereocalib_criteria = (cv.TERM_CRITERIA_MAX_ITER + cv.TERM_CRITERIA_EPS, 100, 1e-5)
    ret, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = cv.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right,
        mtx_left, dist_left, mtx_right, dist_right,
        gray_left.shape[::-1],
        criteria=stereocalib_criteria, flags=flags
    )

    # Stereo rectification (rektifikasyon)
    RL, RR, PL, PR, Q, roiL, roiR = cv.stereoRectify(
        mtx_left, dist_left, mtx_right, dist_right,
        gray_left.shape[::-1], R, T, alpha=0)

    # Stereo harita oluştur
    print("Stereo harita oluşturuluyor...")
    stereoMapL_x, stereoMapL_y = cv.initUndistortRectifyMap(mtx_left, dist_left, RL, PL, gray_left.shape[::-1], cv.CV_32FC1)
    stereoMapR_x, stereoMapR_y = cv.initUndistortRectifyMap(mtx_right, dist_right, RR, PR, gray_right.shape[::-1], cv.CV_32FC1)

    # Sonuçları kaydet
    output_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Normal kalibrasyon verilerini kaydet
    xml_path = os.path.join(output_dir, "stereo_calibration_data.xml")
    fs = cv.FileStorage(xml_path, cv.FILE_STORAGE_WRITE)
    fs.write("cameraMatrix_left", mtx_left)
    fs.write("distCoeffs_left", dist_left)
    fs.write("cameraMatrix_right", mtx_right)
    fs.write("distCoeffs_right", dist_right)
    fs.write("R", R)
    fs.write("T", T)
    fs.write("E", E)
    fs.write("F", F)
    fs.write("RL", RL)
    fs.write("RR", RR)
    fs.write("PL", PL)
    fs.write("PR", PR)
    fs.write("Q", Q)
    fs.release()

    # Stereo harita formatında kaydet (xml.txt formatında)
    stereo_maps_path = os.path.join(output_dir, "stereo_maps.xml")
    stereo_maps = {
        "stereoMapL_x": stereoMapL_x,
        "stereoMapL_y": stereoMapL_y,
        "stereoMapR_x": stereoMapR_x,
        "stereoMapR_y": stereoMapR_y
    }
    save_stereo_maps_to_xml(stereo_maps_path, stereo_maps)

    print(f"\nStereo kalibrasyon ve rektifikasyon tamamlandı.")
    print(f"Normal kalibrasyon verileri: '{xml_path}' olarak kaydedildi.")
    print(f"Stereo harita verileri: '{stereo_maps_path}' olarak kaydedildi.")
    print("\nSol Kamera Matrisi:\n", mtx_left)
    print("\nSağ Kamera Matrisi:\n", mtx_right)
    print("\nDönüşüm Matrisi (R):\n", R)
    print("\nÇeviri Vektörü (T):\n", T)
    print("\nRektifikasyon Matrisleri (RL, RR):\n", RL, RR)
else:
    print("Kalibrasyon başarısız: Geçerli stereo kareler yakalanamadı.")
