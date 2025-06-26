import cv2
import numpy as np
import os
import glob
from tqdm import tqdm

# 检查 OpenCV 版本
assert cv2.__version__[0] == '4', 'The fisheye module requires opencv version >= 4.0.0'

# 棋盘格内角点数
CHECKERBOARD = (11, 8)   # 根据实际修改！！！
# 亚像素角点检测的终止条件
subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
# 鱼眼相机标定标志
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW

# 生成棋盘格在世界坐标系下的 3D 坐标
# 格子大小为 4.5cm
square_size = 4.5   # 根据实际修改！！！
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * square_size

_img_shape = None
objpoints = []  # 3D 点在真实世界空间中的坐标
imgpoints = []  # 2D 点在图像平面上的坐标

# 图片路径
image_path = '/home/handsfree/GeoScan/GeoScan_Data/Image_Data/'   # 修改路径名称！！！
images = glob.glob(os.path.join(image_path, '*.bmp'))             # 修改文件后缀！！！
print(f"Found {len(images)} images in {image_path}")

# 使用 tqdm 显示进度条
for fname in tqdm(images, desc="Processing images", unit="image"):
    try:
        img = cv2.imread(fname)
        if img is None:
            print(f"Failed to read image: {fname}")
            continue
        
        if _img_shape is None:
            _img_shape = img.shape[:2]
        else:
            assert _img_shape == img.shape[:2], "All images must share the same size."
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # 查找棋盘格角点
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ret:
            print(f"Found corners in {fname}")
            objpoints.append(objp)
            cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), subpix_criteria)
            imgpoints.append(corners)
        else:
            print(f"Failed to find corners in {fname}")
    except Exception as e:
        print(f"Error processing {fname}: {e}")

N_OK = len(objpoints)
if N_OK < 10:
    print("Insufficient valid images for calibration. Check the corner detection results.")
else:
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

    # 进行鱼眼相机标定
    rms, _, _, _, _ = cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        gray.shape[::-1],
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
    print("Found " + str(N_OK) + " valid images for calibration")
    print("DIM=" + str(_img_shape[::-1]))
    print("K=np.array(" + str(K.tolist()) + ")")
    print("D=np.array(" + str(D.tolist()) + ")")

# 保存标定结果到文件 - 完全按照指定格式
    output_file = '/home/handsfree/auto-calibration/scripts/calib_result.txt'
    with open(output_file, 'w') as f:
        # 写入DIM信息
        width, height = _img_shape[::-1]
        f.write(f"DIM=({width}, {height})\n")
        
        # 写入K矩阵（精确格式）
        f.write("K=np.array([[")
        # 第一行
        f.write(f"{K[0,0]:.15f}, {K[0,1] if abs(K[0,1]) > 1e-10 else 0.0:.1f}, {K[0,2]:.15f}")
        f.write("], [")
        # 第二行
        f.write(f"{K[1,0] if abs(K[1,0]) > 1e-10 else 0.0:.1f}, {K[1,1]:.15f}, {K[1,2]:.15f}")
        f.write("], [")
        # 第三行
        f.write(f"{K[2,0] if abs(K[2,0]) > 1e-10 else 0.0:.1f}, {K[2,1] if abs(K[2,1]) > 1e-10 else 0.0:.1f}, {K[2,2]:.15f}")
        f.write("]])\n")
        
        # 写入D矩阵（精确格式）
        f.write("D=np.array([")
        # 第一项
        f.write(f"[{D[0,0]:.15f}]")
        f.write(", ")
        # 第二项（特殊处理科学计数法）
        if abs(D[1,0]) < 1e-5 and abs(D[1,0]) > 0:
            f.write(f"[{D[1,0]:.15e}]")
        else:
            f.write(f"[{D[1,0]:.15f}]")
        f.write(", ")
        # 第三项
        f.write(f"[{D[2,0]:.15f}]")
        f.write(", ")
        # 第四项
        f.write(f"[{D[3,0]:.15f}]")
        f.write("])\n")
    
    print(f"Calibration results saved to {output_file}")

    # 保存标定结果到文件
    np.savez('/home/handsfree/GeoScan/GeoScan_Calibration/camera_calibration/fisheye_calibration.npz', K=K, D=D, dims=_img_shape[::-1])

    # 可选：显示一些校正后的图像
    for fname in images[:2]:  # 只显示前2张图
        img = cv2.imread(fname)
        if img is None:
            continue
        h, w = img.shape[:2]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (w, h), cv2.CV_16SC2)
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        cv2.imshow('Undistorted Image', undistorted_img)
        cv2.waitKey(0)

    cv2.destroyAllWindows()
