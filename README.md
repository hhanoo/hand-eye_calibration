# Hand-Eye Calibration ROS2

**Multi-robot hand-eye calibration system with PyQt5 GUI**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-BSD--3--Clause-orange?logo=opensourceinitiative&logoColor=white)](LICENSE)
[![Docker](https://img.shields.io/badge/Docker-Supported-2496ED?logo=docker&logoColor=white)](docker/)

## 목차

- [개요](#개요)
- [주요 기능](#주요-기능)
- [시스템 구조](#시스템-구조)
- [프로젝트 구조](#프로젝트-구조)
- [빠른 시작](#빠른-시작)
- [시스템 요구사항](#시스템-요구사항)
- [설치](#설치)
- [빌드](#빌드)
- [실행](#실행)
- [사용법](#사용법)
- [설정](#설정)
- [ROS2 인터페이스](#ros2-인터페이스)
- [캘리브레이션 알고리즘](#캘리브레이션-알고리즘)
- [문제 해결](#문제-해결)
- [라이선스](#라이선스)

---

## 개요

### 프로젝트 목적

로봇 엔드이펙터와 카메라 사이의 변환 관계(AX=XB)를 구하는 Hand-Eye Calibration 시스템입니다. ROS2 Humble 기반의 PyQt5 GUI에서 데이터 수집, 캘리브레이션 실행, 결과 저장까지 한번에 처리합니다.

### 주요 구성요소

- **gui_node**: ROS2 노드 + PyQt5 GUI — 카메라 영상 표시, 포즈 수집, 캘리브레이션 실행 (Python)
- **realsense2_camera**: Intel RealSense 카메라 ROS2 드라이버 (C++, VCS로 가져옴)
- **calibration**: ArUco 마커 검출, Tsai-Lenz / DQ RANSAC 솔버 모듈 (Python)
- **robot_interface**: UR Direct (read-only 소켓) 또는 ROS2 TF를 통한 로봇 포즈 획득 (Python)

### 적용 가능 영역

- Eye-in-hand 로봇 캘리브레이션 (카메라가 엔드이펙터에 부착된 경우)
- 산업 자동화 시스템의 로봇-카메라 정밀 정합
- 다중 로봇 환경 (UR, Doosan 등)
- 연구 개발 및 교육

---

## 주요 기능

- **다중 로봇 지원** — UR Direct (read-only 소켓) 또는 ROS2 TF를 통해 펜던트 제어를 유지하며 포즈 획득
- **2가지 캘리브레이션 알고리즘** — DQ RANSAC (이상치에 강건), Tsai-Lenz (빠른 결과 확인)
- **PyQt5 GUI** — 카메라 영상 실시간 표시, ArUco 마커 오버레이, 포즈 수집/삭제, 캘리브레이션 실행을 한 화면에서 처리
- **ROS2 네이티브** — RealSense 카메라는 ROS2 토픽으로 수신, TF2로 로봇 포즈 획득 가능
- **Docker 지원** — 빌드 및 실행 스크립트 포함, 원클릭 컨테이너 환경
- **데이터 저장/로드** — CSV 포맷으로 포즈 데이터 저장, 나중에 불러와 재캘리브레이션 가능

---

## 시스템 구조

```
    Intel RealSense D415/D435
              │
              │ USB 3.0
              ▼
    ┌──────────────────────┐
    │ realsense2_camera    │  (ROS2 패키지, VCS로 가져옴)
    │ - Color streaming    │
    │ - Camera info pub    │
    └──────────┬───────────┘
               │ /camera/color/image_raw
               │ /camera/color/camera_info
               ▼
    ┌──────────────────────────────────────────┐
    │ hand_eye_calibration (gui_node)          │
    │                                          │
    │  ┌─────────────┐  ┌───────────────────┐  │
    │  │ ArUco       │  │  Robot Interface  │  │
    │  │ Detector    │  │  (UR Direct /     │  │
    │  │             │  │   ROS2 TF)        │  │
    │  └──────┬──────┘  └───────┬───────────┘  │
    │         │                 │              │
    │         ▼                 ▼              │
    │  ┌────────────────────────────────┐      │
    │  │ Capture Pose                   │      │
    │  │ (marker T + robot T 저장)       │      │
    │  └──────────┬─────────────────────┘      │
    │             │                            │
    │             ▼                            │
    │  ┌────────────────────────────────┐      │
    │  │ Calibration Solver             │      │
    │  │ - DQ RANSAC (기본)              │      │
    │  │ - Tsai-Lenz                    │      │
    │  └──────────┬─────────────────────┘      │
    │             │                            │
    │             ▼                            │
    │  Result: X (4x4, camera ↔ end-effector)  │
    └──────────────────────────────────────────┘
```

---

## 프로젝트 구조

```
Hand-eye_calibration/                           # ROS2 워크스페이스 루트
├── .github/workflows/
│   └── release.yml                             # 태그 push 시 GitHub Release 자동 생성
├── realsense.repos                             # VCS: realsense-ros v4.55.1
├── README.md
├── docker/
│   ├── Dockerfile                              # ROS2 Humble + 의존성
│   ├── config.sh.example                       # Docker 공통 설정 (이미지명, ROS_DOMAIN_ID)
│   ├── build.sh                                # Docker 이미지 빌드
│   ├── run.sh                                  # Docker 컨테이너 실행
│   └── entrypoint.sh                           # ROS2 환경 설정 + alias
│
└── src/
    ├── realsense-ros/                          # VCS로 가져온 RealSense ROS2 드라이버
    │
    └── hand_eye_calibration/                   # 캘리브레이션 패키지 (Python)
        ├── package.xml
        ├── setup.py
        ├── config/
        │   └── default.yaml                    # ROS2 파라미터 (토픽, 마커, 로봇 설정)
        ├── launch/
        │   └── calibration.launch.py           # RealSense + GUI 동시 실행
        │
        └── hand_eye_calibration/               # Python 패키지
            ├── gui_node.py                     # 메인 ROS2 노드 + PyQt5 GUI
            │
            ├── calibration/                    # 캘리브레이션 알고리즘
            │   ├── aruco_detector.py           # ArUco 마커 검출
            │   ├── tsai_lenz.py                # Tsai-Lenz 솔버
            │   ├── dual_quaternion_ransac.py   # DQ RANSAC 래퍼
            │   └── data_io.py                  # 포즈 데이터 CSV 저장/로드
            │
            ├── robot_interface/                # 로봇 포즈 획득
            │   ├── pose_reader_interface.py    # PoseReader ABC
            │   ├── ur_direct_pose_reader.py    # UR read-only 소켓 (포트 30003)
            │   └── ros2_tf_pose_reader.py      # ROS2 TF 방식
            │
            ├── hand_eye_calibration_lib/       # ethz-asl DQ 라이브러리 (추출)
            │   ├── dual_quaternion.py
            │   ├── quaternion.py
            │   └── dual_quaternion_hand_eye_calibration.py
            │
            └── handeye_4dof_lib/               # 4-DOF 솔버 (추출)
                ├── calibrator.py
                └── pose_selector.py
```

---

## 빠른 시작

### Option 1: Docker (권장)

```bash
# 1. Docker 이미지 빌드
cd hand-eye_calibration/docker
chmod +x build.sh run.sh
./build.sh

# 2. 컨테이너 실행
./run.sh

# 3. 컨테이너 내부에서 빌드
cd /ros2_ws
vcs import src < realsense.repos
colcon build --symlink-install
source install/setup.bash

# 4. 실행
ros2 launch hand_eye_calibration calibration.launch.py
```

### Option 2: Native Installation

```bash
# 1. 워크스페이스 설정
cd Hand-eye_calibration
vcs import src < realsense.repos

# 2. 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 3. 빌드
colcon build --symlink-install
source install/setup.bash

# 4. 실행
ros2 launch hand_eye_calibration calibration.launch.py
```

---

## 시스템 요구사항

### 필수

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **Camera**: Intel RealSense D435 or D455 (USB 3.0)

### 소프트웨어 의존성

**ROS2 패키지:**

- rclpy, sensor_msgs, cv_bridge, tf2_ros, geometry_msgs
- realsense2_camera (VCS로 v4.55.1 가져옴)

**Python 패키지:**

- numpy (<2.0), scipy, sympy
- opencv-contrib-python-headless (ArUco 모듈 포함)
- PyQt5, matplotlib

---

## 설치

### Docker (권장)

Docker를 사용하면 모든 의존성이 자동으로 설치됩니다:

```bash
cd Hand-eye_calibration/docker
./build.sh
```

### Native

```bash
# ROS2 패키지 의존성
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-image-transport \
  ros-humble-realsense2-*

# Python 패키지
pip3 install \
  'numpy>=1.21.0,<2.0' \
  scipy sympy \
  opencv-contrib-python-headless==4.10.0.84 \
  PyQt5 PyQt5-sip matplotlib
```

---

## 빌드

```bash
cd Hand-eye_calibration

# RealSense ROS2 가져오기
vcs import src < calibration.repos

# 빌드
colcon build --symlink-install
source install/setup.bash
```

---

## 실행

### Launch (카메라 + GUI 동시 실행)

```bash
ros2 launch hand_eye_calibration calibration.launch.py
```

### GUI만 실행 (카메라를 별도로 실행한 경우)

```bash
ros2 run hand_eye_calibration gui_node
```

### Docker alias

```bash
# 컨테이너 내부에서
camera    # RealSense 카메라만 실행
gui       # GUI만 실행
launch    # 카메라 + GUI 동시 실행
```

---

## 사용법

### 1. 로봇 연결

GUI 상단에서:

- **로봇 모드** 선택: `UR Direct` 또는 `ROS2 TF`
- UR Direct: IP/Port 입력 → **Connect** (read-only 소켓, 펜던트 제어 유지)
- ROS2 TF: Base Frame / EE Frame 입력 → **Connect** (로봇 드라이버가 TF를 퍼블리시해야 함)

### 2. 데이터 수집

1. 카메라 영상에 ArUco 마커가 보이는지 확인 (초록색 축이 그려짐)
2. 로봇을 **펜던트로 다양한 자세로 이동**
3. **Capture Pose** 클릭 → 로봇 포즈 + 마커 포즈가 저장됨
4. 15~30개 포즈를 다양한 각도로 수집
5. 잘못된 데이터는 선택 후 **Delete Selected**

### 3. 캘리브레이션 실행

1. 알고리즘 선택: **DQ RANSAC** (기본, 권장) 또는 **Tsai-Lenz**
2. **Calibrate** 클릭
3. 결과: 4x4 변환 행렬 + RMSE 표시

### 4. 결과 저장

- **Save Result**: 캘리브레이션 결과를 `data/calibration_result.txt`에 저장
- **Save Data**: 포즈 데이터를 `data/pose_pairs.csv`에 저장
- **Load Data**: 이전에 저장한 CSV 데이터를 불러와 재캘리브레이션 가능

---

## 설정

### config/default.yaml

```yaml
hand_eye_calibration:
  ros__parameters:
    # 카메라 토픽
    image_topic: "/camera/camera/color/image_raw"
    camera_info_topic: "/camera/camera/color/camera_info"

    # ArUco 마커 설정
    marker_dict: 10 # cv2.aruco.DICT_6X6_250
    board_grid_shape: [5, 7] # 그리드 보드 (cols, rows)
    marker_length: 0.037 # 마커 크기 (m)
    marker_separation: 0.003 # 마커 간격 (m)

    # 로봇 설정
    robot_mode: "ur_direct" # "ur_direct" 또는 "ros2_tf"

    # ROS2 TF 설정
    tf_base_frame: "base_link"
    tf_ee_frame: "tool0"

    # 데이터 저장 경로
    data_dir: "data/"
```

### 주요 파라미터 설명

| 파라미터            | 타입   | 기본값      | 설명                                                  |
| ------------------- | ------ | ----------- | ----------------------------------------------------- |
| `marker_length`     | float  | 0.037       | ArUco 마커 한 변의 실제 크기 (m) — 정확도에 매우 중요 |
| `marker_separation` | float  | 0.003       | 그리드 보드에서 마커 간 간격 (m)                      |
| `board_grid_shape`  | list   | [5, 7]      | 그리드 보드의 (열, 행) 수                             |
| `robot_mode`        | string | "ur_direct" | 로봇 포즈 획득 방식 ("ur_direct" / "ros2_tf")         |

---

## ROS2 인터페이스

### Subscribed Topics

| Topic                              | Type                     | Description          |
| ---------------------------------- | ------------------------ | -------------------- |
| `/camera/camera/color/image_raw`   | `sensor_msgs/Image`      | RGB 카메라 이미지    |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 카메라 내부 파라미터 |

### Parameters

모든 파라미터는 `config/default.yaml` 또는 launch 인수로 설정 가능합니다.

---

## 캘리브레이션 알고리즘

### DQ RANSAC (기본, 권장)

- **출처**: Daniilidis, "Hand-Eye Calibration Using Dual Quaternions", IEEE 1999
- **라이브러리**: [ethz-asl/hand_eye_calibration](https://github.com/ethz-asl/hand_eye_calibration)
- **방식**: 회전과 이동을 Dual Quaternion으로 동시에 풀이, RANSAC으로 이상치 자동 제거
- **장점**: 노이즈/이상치에 강건
- **권장 포즈 수**: 15개 이상

### Tsai-Lenz

- **출처**: Tsai & Lenz, IEEE 1989
- **방식**: 회전(R)을 SVD로 먼저 풀고, 이동(t)을 최소자승법으로 풀이
- **장점**: 매우 빠름, 간단
- **단점**: 이상치 처리 없음 — 데이터가 깨끗해야 함

### 4-DOF Calibrator (향후)

- **출처**: [QuantuMope/handeye-4dof](https://github.com/QuantuMope/handeye-4dof)
- SCARA 등 4축 로봇용, 현재 라이브러리만 포함 (GUI 통합 예정)

---

## 문제 해결

### 1. Qt 플러그인 충돌

**증상:**

```
qt.qpa.plugin: Could not load the Qt platform plugin "xcb"
```

**해결:**

- Dockerfile에서 `opencv-contrib-python-headless` 사용 (OpenCV 번들 Qt 제거)
- 또는 `gui_node.py` 상단에 `os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)` 확인

### 2. RealSense 카메라 인식 안 됨

```bash
# USB 연결 확인
lsusb | grep Intel

# 권한 확인
sudo usermod -aG video $USER
sudo usermod -aG plugdev $USER
```

### 3. ArUco 마커 검출 안 됨

- 조명이 충분한지 확인
- 카메라에서 마커까지 거리: 0.3m ~ 2m 권장
- `marker_length` 값이 실제 마커 크기와 일치하는지 확인

### 4. cv_bridge NumPy 버전 충돌

```bash
pip3 install 'numpy>=1.21.0,<2.0'
```

---

## 외부 라이브러리 출처

| 라이브러리           | 출처                                                   | 용도                    |
| -------------------- | ------------------------------------------------------ | ----------------------- |
| hand_eye_calibration | https://github.com/ethz-asl/hand_eye_calibration       | DQ RANSAC 솔버          |
| handeye-4dof         | https://github.com/QuantuMope/handeye-4dof             | 4-DOF 캘리브레이션      |
| realsense-ros        | https://github.com/realsenseai/realsense-ros (v4.55.1) | RealSense ROS2 드라이버 |

---

## 라이선스

이 프로젝트는 BSD-3-Clause 라이선스로 배포됩니다.

---

**Maintainer**: hhanoo (woo980711@gmail.com)
