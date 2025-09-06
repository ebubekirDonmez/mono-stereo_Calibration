# Stereo Camera Calibration System

**Professional stereo camera calibration toolkit for dual-camera setups using OpenCV and chessboard patterns with real-time calibration monitoring and distortion correction.**

## 🎯 Project Overview

This repository provides a comprehensive stereo camera calibration solution designed for computer vision applications requiring accurate dual-camera setups. The system enables precise camera calibration, lens distortion correction, and stereo rectification for industrial and research applications.

## ✨ Key Features

- **Dual Camera Calibration**: Simultaneous calibration of left and right cameras
- **Real-time Monitoring**: Live preview with automatic corner detection
- **Chessboard Detection**: Automatic detection of 18x12 internal corners
- **Distortion Correction**: Lens distortion correction and stereo rectification
- **XML Export**: Calibration parameters saved in OpenCV XML format
- **Verification Tools**: Built-in calibration verification and testing
- **Industrial Ready**: Designed for production environments

## 🛠️ Technology Stack

- **OpenCV**: Camera calibration and stereo vision algorithms
- **NumPy**: Numerical computations and matrix operations
- **Python 3.7+**: Main programming language
- **Chessboard Patterns**: 18x12 internal corners, 20mm square size

## 🚀 Quick Start

### Prerequisites
```bash
# Install required packages
pip install opencv-python numpy
```

### Basic Usage
```bash
# Run stereo calibration
python stereoCalibration1.py

# Verify calibration results
python stereoTest.py
```

## 📊 Calibration Process

### 1. Preparation
- Print chessboard pattern (18x12 internal corners, 20mm squares)
- Mount cameras in stereo configuration
- Ensure good lighting conditions

### 2. Calibration
- Run `stereoCalibration1.py`
- Move chessboard to different positions
- System automatically captures 35 stereo pairs
- Calibration completes automatically

### 3. Verification
- Run `stereoTest.py` to verify results
- Compare original vs. undistorted images
- Check calibration quality metrics

## 📁 Repository Structure

```
StereoCalibration/
├── stereoCalibration1.py         # Main calibration script
├── stereoTest.py                 # Calibration verification
├── stereo_calibration_data.xml   # Calibration parameters output
└── README.md                     # This file
```

## 🔧 Configuration

### Camera Settings
```python
# Camera IDs (adjust based on your setup)
cap_left = cv.VideoCapture(0)
cap_right = cv.VideoCapture(1)

# Resolution settings
cap_left.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
cap_left.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
```

### Chessboard Parameters
```python
chessboard_size = (18, 12)  # Internal corner count
square_size = 20            # Square size in mm
required_samples = 35       # Target calibration samples
```

### Exposure Control
```python
# Manual exposure for better calibration
cap_left.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap_left.set(cv.CAP_PROP_EXPOSURE, -7)
cap_left.set(cv.CAP_PROP_AUTO_WB, 0)
cap_left.set(cv.CAP_PROP_WB_TEMPERATURE, 5000)
```

## 📊 Output Files

### stereo_calibration_data.xml
Contains essential calibration parameters:
- `cameraMatrix_left/right`: Camera intrinsic matrices
- `distCoeffs_left/right`: Distortion coefficients
- `R`: Rotation matrix between cameras
- `T`: Translation vector between cameras
- `E`: Essential matrix
- `F`: Fundamental matrix
- `RL/RR`: Rectification rotation matrices
- `PL/PR`: Rectification projection matrices
- `Q`: Disparity-to-depth mapping matrix

## 🎮 Controls

- **'q' Key**: Exit calibration process
- **Automatic Exit**: When 35 samples are collected
- **Real-time Preview**: Live camera feed with corner detection

## 📈 Quality Metrics

| Metric | Excellent | Good | Acceptable |
|--------|-----------|------|------------|
| Reprojection Error | <0.5px | 0.5-1.0px | 1.0-2.0px |
| Stereo RMS Error | <0.5px | 0.5-1.0px | 1.0-2.0px |
| Calibration Samples | 50+ | 35-50 | 25-35 |

## 🔍 Calibration Tips

### Effective Calibration Process
1. **Chessboard Quality**: Use high-quality, flat chessboard
2. **Positioning**: Cover entire field of view of both cameras
3. **Movement**: Vary angles and distances systematically
4. **Lighting**: Ensure even, consistent lighting
5. **Stability**: Hold chessboard steady during detection

### Quality Indicators
- High corner detection success rate
- Diverse chessboard positions and angles
- Both cameras detecting corners simultaneously
- Low reprojection error in results

## 🐛 Troubleshooting

### Common Issues
- **Camera Access**: Check camera IDs and connections
- **Poor Corner Detection**: Improve lighting and chessboard quality
- **Calibration Fails**: Increase sample count and improve positioning
- **High Error**: Use more samples and better chessboard positioning

### Performance Optimization
- Use higher resolution cameras for better accuracy
- Ensure stable camera mounting
- Maintain consistent lighting conditions
- Use high-quality chessboard prints

## 🎯 Use Cases

- **Stereo Vision**: Depth estimation and 3D reconstruction
- **Robotic Vision**: Dual-camera robotic systems
- **Quality Control**: Industrial inspection systems
- **Research**: Computer vision research projects
- **Education**: Learning stereo vision concepts

## 🔬 Technical Details

### Calibration Algorithm
1. **Single Camera Calibration**: Individual calibration of each camera
2. **Stereo Calibration**: Joint calibration using stereo pairs
3. **Stereo Rectification**: Image rectification for epipolar geometry
4. **Map Generation**: Undistortion and rectification maps

### Coordinate Systems
- **Camera Coordinates**: 3D points relative to camera
- **Image Coordinates**: 2D pixel coordinates
- **World Coordinates**: 3D points in real-world space

## 📝 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 👨‍💻 Developer

**Ebubekir** - *Internship Project*

## 🙏 Acknowledgments

- [OpenCV](https://opencv.org/) - Computer vision library
- [NumPy](https://numpy.org/) - Numerical computing library

---

**Note**: This calibration system is designed for professional stereo vision applications. Ensure proper camera synchronization and stable mounting for best results.

---

## 📖 Detailed Description

This stereo camera calibration system provides a complete solution for calibrating dual-camera setups used in computer vision applications. The system uses chessboard patterns to establish precise camera parameters, including intrinsic matrices, distortion coefficients, and stereo geometry.

The calibration process involves capturing multiple stereo image pairs with the chessboard in different positions and orientations. The system automatically detects chessboard corners, refines their positions, and computes camera parameters using OpenCV's proven calibration algorithms.

Key features include real-time corner detection, automatic sample collection, distortion correction, and stereo rectification. The output provides both standard calibration parameters and stereo rectification maps, making it suitable for depth estimation and 3D reconstruction applications.

This solution is particularly useful for industrial applications requiring accurate stereo vision, such as quality control, robotic vision, and automated inspection systems where precise depth perception is critical for operational success.

The system's real-time monitoring capabilities allow users to see the calibration progress and ensure optimal chessboard positioning, resulting in high-quality calibration parameters that are essential for accurate stereo vision applications.
