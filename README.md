# Hand-Eye Calibration

**Multi-robot hand-eye calibration system with PyQt5 GUI**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C++-17-00599C?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.10-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)
[![Docker](https://img.shields.io/badge/Docker-Supported-2496ED?logo=docker&logoColor=white)](docker/)
[![License](https://img.shields.io/badge/License-MIT-F99304?logo=opensourceinitiative&logoColor=white)](LICENSE)

---

## 목차

- [데모](#데모)
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
- [API / ROS2 인터페이스](#api--ros2-인터페이스)
- [문제 해결](#문제-해결)
- [라이선스](#라이선스)
- [Maintainer](#maintainer)

---

## 데모

<details>
<summary>GUI 화면</summary>

![Hand-Eye Calibration GUI](docs/hand-eye_calibration_GUI.png)

</details>

<details>
<summary>Pose Pairs 시각화</summary>

![Pose Pairs Visualization](docs/pose_pairs_visualization.png)

</details>

<details>
<summary>캘리브레이션 결과 시각화</summary>

![Calibration Result Visualization](docs/calibration_result_visualization.png)

</details>

---

## 개요

### 프로젝트 목적

로봇 엔드이펙터와 카메라 사이의 변환 관계(AX=XB)를 구하는 Hand-Eye Calibration 시스템입니다. ROS2 Humble 기반의 PyQt5 GUI에서 데이터 수집, 캘리브레이션 실행, 결과 저장까지 한번에 처리합니다.

### 주요 구성요소

- **gui_node** (Python): ROS2 노드 + PyQt5 GUI, 카메라 영상 표시, 포즈 수집, 캘리브레이션 실행
- **카메라 드라이버**: 사용 센서에 따라 VCS로 선택  
  (RealSense: `realsense2_camera`, Orbbec: `OrbbecSDK_ROS2`)
- **calibration** (Python): ArUco 마커 검출, Tsai-Lenz / DQ RANSAC 솔버 모듈
- **robot_interface** (Python): UR Direct (read-only 소켓) 또는 ROS2 TF를 통한 로봇 포즈 획득
- **dsr_pose_reader** (C++): Doosan 로봇 전용 포즈 리더, DRFL 모니터링으로 펜던트 제어 유지

### 캘리브레이션 알고리즘

**DQ RANSAC (기본, 권장)**

- **출처**: Daniilidis, "Hand-Eye Calibration Using Dual Quaternions", IEEE 1999
- **라이브러리**: [ethz-asl/hand_eye_calibration](https://github.com/ethz-asl/hand_eye_calibration)
- **방식**: 회전과 이동을 Dual Quaternion으로 동시에 풀이, RANSAC으로 이상치 자동 제거
- **장점**: 노이즈/이상치에 강건
- **권장 포즈 수**: 15개 이상

**Tsai-Lenz**

- **출처**: Tsai & Lenz, IEEE 1989
- **방식**: 회전(R)을 SVD로 먼저 풀고, 이동(t)을 최소자승법으로 풀이
- **장점**: 매우 빠름, 간단
- **단점**: 이상치 처리 없음 -- 데이터가 깨끗해야 함

**4-DOF Calibrator (향후)**

- **출처**: [QuantuMope/handeye-4dof](https://github.com/QuantuMope/handeye-4dof)
- SCARA 등 4축 로봇용, 현재 라이브러리만 포함 (GUI 통합 예정)

### 적용 가능 영역

- Eye-in-hand 로봇 캘리브레이션 (카메라가 엔드이펙터에 부착된 경우)
- 산업 자동화 시스템의 로봇-카메라 정밀 정합
- 다중 로봇 환경 (UR, Doosan 등)
- 연구 개발 및 교육

---

## 주요 기능

- **다중 로봇 지원**: UR Direct (read-only 소켓), Doosan DRFL (모니터링), ROS2 TF를 통해 펜던트 제어를 유지하며 포즈 획득
- **2가지 캘리브레이션 알고리즘**: DQ RANSAC (이상치에 강건), Tsai-Lenz (빠른 결과 확인)
- **PyQt5 GUI**: 카메라 영상 실시간 표시, ArUco 마커 오버레이, 포즈 수집/삭제, 캘리브레이션 실행을 한 화면에서 처리
- **3D 시각화**: 수집된 포즈 쌍과 캘리브레이션 결과를 Matplotlib 3D 플롯으로 확인
- **ROS2 네이티브**: 카메라 영상은 ROS2 토픽으로 수신 (RealSense / Orbbec 등), TF2로 로봇 포즈 획득 가능
- **Docker 지원**: 빌드 및 실행 스크립트 포함, 원클릭 컨테이너 환경
- **데이터 저장/로드**: CSV 포맷으로 포즈 데이터 저장, 나중에 불러와 재캘리브레이션 가능

---

## 시스템 구조

```
┌───────────────────────┐  ┌──────────────────┐  ┌────────────────────────┐
│ Vision Sensor         │  │ UR Robot         │  │ Doosan Robot           │
│ RealSense / Orbbec    │  │                  │  │                        │
└───────────┬───────────┘  └────────┬─────────┘  └───────────┬────────────┘
            │ USB 3.0               │ TCP:30003               │ TCP:12345
            │                       │ (read-only)             │ (DRFL monitoring)
┌───────────┴───────────┐           │          ┌──────────────┴────────────┐
│ Camera ROS2 Driver    │           │          │ dsr_pose_reader (C++)     │
│ - Color streaming     │           │          │ - No access control       │
│ - Camera info pub     │           │          │ - Pendant preserved       │
└───────────┬───────────┘           │          └──────────────┬────────────┘
            │                       │                         │
            │ /camera/color/        │                         │ /tf, /tcp_pose
            │   image_raw           │                         │
            │   camera_info         │                         │
            │                       │                         │
┌───────────┴───────────────────────┴─────────────────────────┴───────────┐
│ hand_eye_calibration (gui_node)                                         │
│                                                                         │
│  ┌──────────────┐  ┌──────────────────────────────────────────┐         │
│  │ ArUco        │  │ Robot Interface                          │         │
│  │ Detector     │  │ - UR Direct (read-only socket)           │         │
│  │              │  │ - ROS2 TF (dsr_pose_reader, etc.)        │         │
│  └──────┬───────┘  └────────────────────┬─────────────────────┘         │
│         │                               │                               │
│  ┌──────┴───────────────────────────────┴──────┐                        │
│  │ Capture Pose (marker T + robot T 저장)       │                        │
│  └──────────────────────┬──────────────────────┘                        │
│                         │                                               │
│  ┌──────────────────────┴──────────────────────┐                        │
│  │ Calibration Solver                          │                        │
│  │ - DQ RANSAC (기본)  - Tsai-Lenz              │                        │
│  └──────────────────────┬──────────────────────┘                        │
│                         │                                               │
│  Result: X (4x4, camera <-> end-effector)                               │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 프로젝트 구조

```
Hand-eye_calibration/                           # ROS2 워크스페이스 루트
├── .github/workflows/
│   └── release.yml                             # 태그 push 시 GitHub Release 자동 생성
├── camera_realsense.repos                      # VCS: realsense-ros v4.55.1
├── camera_orbbec.repos                         # VCS: OrbbecSDK_ROS2 v2-main
├── docker/
│   ├── Dockerfile                              # ROS2 Humble + 의존성
│   ├── config.sh.example                       # Docker 공통 설정 (이미지명, ROS_DOMAIN_ID)
│   ├── build.sh                                # Docker 이미지 빌드
│   ├── run.sh                                  # Docker 컨테이너 실행
│   ├── entrypoint.sh                           # ROS2 환경 설정
│   └── aliases.sh                              # 컨테이너 내 alias 정의
│
└── src/
    ├── (camera driver)                         # VCS로 가져옴: realsense-ros 또는 OrbbecSDK_ROS2
    │
    ├── dsr_pose_reader/                        # Doosan 로봇 포즈 리더 (C++)
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   ├── doosan_api/                         # DRFL 라이브러리 (Doosan Robotics, BSD)
    │   │   ├── include/                        # DRFL.h, DRFC.h, DRFLEx.h, DRFS.h
    │   │   └── lib/                            # libDRFL.a, libPoco*.so
    │   ├── include/dsr_pose_reader/
    │   │   └── pose_reader_node.hpp
    │   ├── config/
    │   │   └── default.yaml                    # Doosan 연결 파라미터
    │   ├── launch/
    │   │   └── dsr_pose_reader.launch.py
    │   └── src/
    │       └── pose_reader_node.cpp
    │
    └── hand_eye_calibration/                   # 캘리브레이션 패키지 (Python)
        ├── package.xml
        ├── setup.py
        ├── config/
        │   └── default.yaml                    # ROS2 파라미터 (토픽, 마커, 로봇 설정)
        │
        └── hand_eye_calibration/               # Python 패키지
            ├── gui_node.py                     # 메인 ROS2 노드 + PyQt5 GUI
            │
            ├── calibration/                    # 캘리브레이션 알고리즘
            │   ├── aruco_detector.py           # ArUco 마커 검출
            │   ├── tsai_lenz.py                # Tsai-Lenz 솔버
            │   ├── dual_quaternion_ransac.py   # DQ RANSAC 래퍼
            │   ├── visualization.py            # 3D 시각화 (Matplotlib)
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
# 1. Docker 이미지 가져오기
docker pull hhanoo/project:hand-eye-calibration-humble

# 2. 컨테이너 실행 (X11 포워딩 포함)
cd Hand-eye_calibration/docker
./run.sh

# 3. 빌드 (컨테이너 내부)
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash

# 4. 카메라와 GUI를 각각 실행 (터미널 2개)
#    터미널 1: 카메라 (RealSense 또는 Orbbec)
camera_realsense          # 또는 camera_orbbec
#    터미널 2: GUI
gui
```

> 센서별 launch 옵션·토픽은 [실행 > 개별 모듈](#개별-모듈) 참고.

### Option 2: Native

사용 센서의 드라이버를 먼저 설치 — 위 [설치 > Method 2 > 시스템 의존성](#1-시스템-의존성-센서-공통) 참고.

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

이후 카메라와 GUI를 각각 실행 — [실행 > 개별 모듈](#개별-모듈) 참고.

---

## 시스템 요구사항

### 필수

| 항목       | 요구사항                                                                   |
| ---------- | -------------------------------------------------------------------------- |
| **OS**     | Ubuntu 22.04                                                               |
| **ROS2**   | Humble                                                                     |
| **Python** | 3.10                                                                       |
| **Camera** | Intel RealSense D415 / D435, Orbbec Femto Mega/Bolt, Gemini 2 등 (USB 3.0) |

### 하드웨어 (지원 로봇)

| 로봇   | 연결 방식                        | 비고                  |
| ------ | -------------------------------- | --------------------- |
| UR     | TCP 소켓 (포트 30003, read-only) | 펜던트 제어 유지      |
| Doosan | DRFL 모니터링 (dsr_pose_reader)  | 펜던트 제어 유지      |
| 기타   | ROS2 TF 방식                     | TF 퍼블리시 노드 필요 |

### 소프트웨어 의존성

**ROS2 패키지:**

- rclpy, rclcpp, sensor_msgs, geometry_msgs, cv_bridge, tf2_ros
- 카메라 드라이버: `realsense2_camera` (v4.55.1) 또는 `OrbbecSDK_ROS2` (v2-main) — VCS로 선택 설치

**Python 패키지:**

- numpy (<2.0), scipy, sympy
- opencv-contrib-python-headless 4.10.0.84 (ArUco 모듈 포함)
- PyQt5, matplotlib

**C++ 라이브러리 (vendored):**

- DRFL (Doosan Robotics, BSD)
- Poco (PocoFoundation, PocoNet)

### 외부 패키지

| 패키지               | 출처                                                   | 용도                    |
| -------------------- | ------------------------------------------------------ | ----------------------- |
| hand_eye_calibration | https://github.com/ethz-asl/hand_eye_calibration       | DQ RANSAC 솔버          |
| handeye-4dof         | https://github.com/QuantuMope/handeye-4dof             | 4-DOF 캘리브레이션      |
| realsense-ros        | https://github.com/realsenseai/realsense-ros (v4.55.1) | RealSense ROS2 드라이버 |
| OrbbecSDK_ROS2       | https://github.com/orbbec/OrbbecSDK_ROS2 (v2-main)     | Orbbec ROS2 드라이버    |
| DRFL                 | https://robotlab.doosanrobotics.com/ko/Index           | Doosan 로봇 모니터링    |

---

## 설치

### Method 1: Docker (권장)

```bash
# 1. Docker 이미지 가져오기
docker pull hhanoo/project:hand-eye-calibration-humble

# 2. 컨테이너 실행
cd Hand-eye_calibration/docker
./run.sh
```

<details>
<summary>직접 빌드 (개발자용)</summary>

```bash
# 0. 프로젝트 루트로 이동
cd Hand-eye_calibration/docker

# 1. 설정 파일 생성 후 IMAGE_NAME을 로컬 이름으로 변경
cp config.sh.example config.sh
# config.sh에서 IMAGE_NAME="hand-eye-calibration-humble" 로 수정

# 2. Docker 이미지 빌드
./build.sh

# 3. 컨테이너 실행
./run.sh
```

</details>

### Method 2: Native

#### 1. 시스템 의존성 (센서 공통)

```bash
# ROS2 공통 패키지 의존성
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-image-transport
```

#### 2. Python 패키지

```bash
pip3 install \
  'numpy>=1.21.0,<2.0' \
  scipy sympy \
  opencv-contrib-python-headless==4.10.0.84 \
  PyQt5 PyQt5-sip matplotlib
```

#### 3. 비전 센서별 추가 설치

##### Intel RealSense (D415 / D435 등)

```bash
sudo apt update && sudo apt install -y ros-humble-realsense2-*

# 실행 확인
ros2 launch realsense2_camera rs_launch.py
```

USB 3.0 연결 필수. 인식 불가 시 [문제 해결 > RealSense 카메라 인식 안 됨](#2-realsense-카메라-인식-안-됨) 참고.

##### Orbbec (Femto Mega/Bolt, Gemini 2 / 330 시리즈 등)

```bash
sudo apt update && sudo apt install -y \
  ros-humble-orbbec-camera \
  ros-humble-orbbec-description

# 장치 인식 확인 & 실행
ros2 run orbbec_camera list_devices_node
ros2 launch orbbec_camera <model>.launch.py
```

udev rules은 apt 패키지에 포함되어 자동 설치됨. 인식 불가 시 `sudo udevadm control --reload-rules && sudo udevadm trigger` 재실행 또는 장치 재연결.

주요 `<model>` launch 파일 매핑:

| 모델                          | launch 파일                   |
| ----------------------------- | ----------------------------- |
| Femto Mega / Mega I           | `femto_mega.launch.py`        |
| Femto Bolt                    | `femto_bolt.launch.py`        |
| Gemini 2                      | `gemini2.launch.py`           |
| Gemini 2 L                    | `gemini2L.launch.py`          |
| Gemini 330 / 335 / 336 시리즈 | `gemini_330_series.launch.py` |
| Astra 2                       | `astra2.launch.py`            |

> Docker 사용 시 [설정 > Docker 설정](#docker-설정)의 `ORBBEC_MODEL`을 사용 모델로 지정하면 컨테이너 내부에서 `camera_orbbec` alias로 바로 실행 가능.
>
> 특정 버전 핀 고정이나 최신 소스 빌드가 필요한 경우 `camera_realsense.repos` / `camera_orbbec.repos`를 이용한 `vcs import` + `colcon build` 방식도 사용 가능.

---

## 빌드

### 전체 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 특정 모듈 빌드

```bash
colcon build --symlink-install --packages-select hand_eye_calibration
colcon build --symlink-install --packages-select dsr_pose_reader
```

### 클린 빌드

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

---

## 실행

카메라와 GUI는 항상 별도 터미널(또는 alias)로 실행합니다. 사용하는 비전 센서마다 launch 명령이 다르기 때문에, 통합 launch 파일은 제공하지 않습니다.

### 개별 모듈

**카메라: Intel RealSense (D415 / D435 등)**

```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=1280x720x30 \
  depth_module.depth_profile:=1280x720x30 \
  align_depth.enable:=true
```

- 토픽: `/camera/camera/color/image_raw`, `/camera/camera/color/camera_info`
- Docker: `camera_realsense` alias로 동일 명령 실행

**카메라: Orbbec (Femto Mega/Bolt, Gemini 2 / 330 시리즈 등)**

```bash
ros2 run orbbec_camera list_devices_node                # 장치 인식 확인
ros2 launch orbbec_camera <model>.launch.py \
  color_width:=1280 color_height:=720 color_fps:=30
```

- `<model>`: `femto_mega`, `femto_bolt`, `gemini2`, `gemini2L`, `gemini_330_series`, `astra2` 등
- Docker: `camera_orbbec` alias 사용 (모델·해상도는 [config.sh](docker/config.sh.example)의 `ORBBEC_MODEL`, `ORBBEC_COLOR_*`로 지정)
- 토픽 prefix가 RealSense와 다르므로 [config/default.yaml](src/hand_eye_calibration/config/default.yaml)의 `image_topic` / `camera_info_topic` 조정 필요

**GUI 단독 실행:**

```bash
ros2 run hand_eye_calibration gui_node
```

**UR 로봇 (UR Direct 모드):**

GUI에서 UR Direct 모드로 직접 연결하므로 별도 드라이버가 필요 없습니다.

**Doosan 로봇 (dsr_pose_reader + ROS2 TF 모드):**

dsr_pose_reader를 먼저 실행하고, GUI에서 ROS2 TF 모드로 연결합니다.

```bash
# 터미널 1: Doosan 포즈 리더
ros2 launch dsr_pose_reader dsr_pose_reader.launch.py

# 커스텀 yaml 지정
ros2 launch dsr_pose_reader dsr_pose_reader.launch.py config_file:=/path/to/custom.yaml

# 터미널 2/3: 카메라 + GUI (위 "카메라" 항목 참고)
camera_realsense   # 또는 camera_orbbec
gui
```

### Docker

```bash
cd docker
./run.sh

# 컨테이너 내부
cd /ros2_ws
colcon build --symlink-install && source install/setup.bash

# 카메라와 GUI를 별도 셸에서 실행
camera_realsense   # 또는 camera_orbbec
gui
```

전체 alias 정의는 [aliases.sh](docker/aliases.sh)를 참고하세요.

| Alias              | 설명                        | 참고                                                                               |
| ------------------ | --------------------------- | ---------------------------------------------------------------------------------- |
| `camera_realsense` | RealSense 카메라 실행       | —                                                                                  |
| `camera_orbbec`    | Orbbec 카메라 실행          | `config.sh`의 `ORBBEC_MODEL` (모델), `ORBBEC_COLOR_WIDTH/HEIGHT/FPS` (해상도) 사용 |
| `gui`              | GUI 실행                    | [gui_node.py](src/hand_eye_calibration/hand_eye_calibration/gui_node.py)           |
| `doosan`           | Doosan 포즈 리더 실행       | [dsr_pose_reader.launch.py](src/dsr_pose_reader/launch/dsr_pose_reader.launch.py)  |
| `build`            | 워크스페이스 빌드           | —                                                                                  |
| `cmd_help`         | 사용 가능한 alias 목록 출력 | 컨테이너 접속 시 자동 출력                                                         |

---

## 사용법

### 워크플로우

```
로봇 연결  ────▶ 마커 확인 ────▶ 포즈 수집 (15~30) ────▶ 캘리브레이션 ────▶ 결과 확인 ────▶ 저장

```

### 1. 로봇 연결

GUI 상단에서:

- **로봇 모드** 선택: `UR Direct` 또는 `ROS2 TF`
- UR Direct: IP/Port 입력 → **Connect** (read-only 소켓, 펜던트 제어 유지)
- ROS2 TF: Base Frame / EE Frame 입력 → **Connect** (dsr_pose_reader 등 TF 퍼블리시 노드 필요)

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

### 4. 결과 확인

- **Calibration Result**: 캘리브레이션 결과를 3D 좌표계로 시각화
- **Data Poses**: 수집된 포즈 쌍을 3D 플롯으로 확인

### 5. 결과 저장

- **Save Result**: 캘리브레이션 결과를 `data/calibration_result_*.txt`에 저장
- **Save Data**: 포즈 데이터를 `data/pose_pairs.csv`에 저장
- **Load Data**: 이전에 저장한 CSV 데이터를 불러와 재캘리브레이션 가능

---

## 설정

### Docker 설정

[config.sh](docker/config.sh.example)

```bash
IMAGE_NAME="hhanoo/project:hand-eye-calibration-humble"  # Docker Hub 이미지 (기본값)
CONTAINER_NAME="hand-eye-calibration-humble"              # Docker 컨테이너 이름
ROS_DOMAIN_ID=98                                          # ROS2 domain
ORBBEC_MODEL="femto_bolt"                                 # camera_orbbec alias가 실행할 Orbbec 모델
                                                          # (femto_mega, femto_bolt, gemini2, gemini2L, gemini_330_series 등)
ORBBEC_COLOR_WIDTH=1280                                   # Orbbec color 스트림 너비
ORBBEC_COLOR_HEIGHT=720                                   # Orbbec color 스트림 높이
ORBBEC_COLOR_FPS=30                                       # Orbbec color 스트림 FPS
```

> `run.sh` 실행 전 `docker pull hhanoo/project:hand-eye-calibration-humble`로 이미지를 가져오세요.
>
> 직접 빌드하려면 `IMAGE_NAME`을 `"hand-eye-calibration-humble"` 등으로 변경 후 `./build.sh`를 실행하세요.

### hand_eye_calibration/config/default.yaml

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

### dsr_pose_reader/config/default.yaml

```yaml
dsr_pose_reader:
  ros__parameters:
    robot_ip: "192.168.137.100" # Doosan 컨트롤러 IP
    robot_port: 12345 # Doosan 컨트롤러 포트
    base_frame: "base_link" # TF base frame
    ee_frame: "tool0" # TF end-effector frame
    publish_rate: 30.0 # Hz
    publish_tf: true # TF 브로드캐스트 활성화
```

### 주요 파라미터 설명

| 파라미터            | 타입   | 기본값      | 설명                                                   |
| ------------------- | ------ | ----------- | ------------------------------------------------------ |
| `marker_length`     | float  | 0.037       | ArUco 마커 한 변의 실제 크기 (m) -- 정확도에 매우 중요 |
| `marker_separation` | float  | 0.003       | 그리드 보드에서 마커 간 간격 (m)                       |
| `board_grid_shape`  | list   | [5, 7]      | 그리드 보드의 (열, 행) 수                              |
| `robot_mode`        | string | "ur_direct" | 로봇 포즈 획득 방식 ("ur_direct" / "ros2_tf")          |

### Launch 인자

**dsr_pose_reader.launch.py:**

| 인자          | 기본값                | 설명                    |
| ------------- | --------------------- | ----------------------- |
| `config_file` | `config/default.yaml` | 파라미터 YAML 파일 경로 |

```bash
ros2 launch dsr_pose_reader dsr_pose_reader.launch.py config_file:=/path/to/custom.yaml
```

---

## API / ROS2 인터페이스

### 노드

| 노드                   | 언어   | 패키지               | 설명                          |
| ---------------------- | ------ | -------------------- | ----------------------------- |
| `hand_eye_calibration` | Python | hand_eye_calibration | 캘리브레이션 GUI + ROS2 노드  |
| `dsr_pose_reader`      | C++    | dsr_pose_reader      | Doosan 로봇 TCP 포즈 퍼블리셔 |

### Subscribed 토픽

| Topic                              | Type                         | 노드                 | 설명                 |
| ---------------------------------- | ---------------------------- | -------------------- | -------------------- |
| `/camera/camera/color/image_raw`   | `sensor_msgs/msg/Image`      | hand_eye_calibration | RGB 카메라 이미지    |
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | hand_eye_calibration | 카메라 내부 파라미터 |

### Published 토픽

| Topic      | Type                            | 노드            | 설명                          |
| ---------- | ------------------------------- | --------------- | ----------------------------- |
| `tcp_pose` | `geometry_msgs/msg/PoseStamped` | dsr_pose_reader | Doosan TCP 포즈 (base 기준)   |
| `/tf`      | `tf2_msgs/msg/TFMessage`        | dsr_pose_reader | base_link → tool0 변환 (옵션) |

### 서비스

해당 프로젝트는 ROS2 서비스를 사용하지 않습니다.

### 커스텀 메시지

해당 프로젝트는 커스텀 메시지를 정의하지 않습니다. 모든 인터페이스는 표준 ROS2 메시지 타입을 사용합니다.

### 네트워크 구성

| 대상             | 프로토콜 | 포트  | 방향       | 비고                      |
| ---------------- | -------- | ----- | ---------- | ------------------------- |
| UR 로봇          | TCP      | 30003 | read-only  | 로봇에 명령 전송하지 않음 |
| Doosan 로봇      | TCP      | 12345 | monitoring | DRFL 모니터링 전용        |
| RealSense 카메라 | USB 3.0  | -     | -          | ROS2 토픽으로 수신        |

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

**증상:**

```
lsusb | grep Intel  # 결과 없음
```

**해결:**

```bash
sudo usermod -aG video $USER
sudo usermod -aG plugdev $USER
```

### 3. ArUco 마커 검출 안 됨

- 조명이 충분한지 확인
- 카메라에서 마커까지 거리: 0.3m ~ 2m 권장
- `marker_length` 값이 실제 마커 크기와 일치하는지 확인

### 4. cv_bridge NumPy 버전 충돌

**증상:**

```
AttributeError: module 'numpy' has no attribute 'bool'
```

**해결:**

```bash
pip3 install 'numpy>=1.21.0,<2.0'
```

### 5. Docker에서 GUI 표시 안 됨

**증상:**

```
cannot open display: :0
```

**해결:**

```bash
# 호스트에서 X11 접근 허용
xhost +local:docker

# DISPLAY 환경변수 확인
echo $DISPLAY
```

---

## 라이선스

이 프로젝트는 MIT 라이선스로 배포됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

---

## Maintainer

**hhanoo** (woo980711@gmail.com)
