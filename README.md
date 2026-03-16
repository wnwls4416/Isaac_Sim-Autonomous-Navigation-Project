# Isaac_Sim-Autonomous-Navigation-Project
ROS 2와 NVIDIA Isaac Sim을 활용한 Ackermann 차량 자율주행 프로젝트입니다. A* 알고리즘 기반 전역 경로 탐색과 OpenCV 차선 인식(Vision) 기술을 융합하여 교차로와 직진 구간을 주행합니다.

# 🚗 ROS 2 & Isaac Sim Autonomous Navigation Project

본 프로젝트는 **NVIDIA Isaac Sim**과 **ROS 2 (Humble)** 를 연동하여, 가상 환경 내에서 Ackermann 조향 차량의 자율주행을 구현한 시스템입니다. A* 알고리즘을 활용한 전역 경로 계획과 OpenCV 기반의 차선 인식(Vision) 알고리즘을 결합하여 교차로와 직진 구간을 자율적으로 주행합니다.

---

## 🌟 주요 기능 (Key Features)

* **A* 기반 전역 경로 계획 (Global Path Planning)**: 사전에 정의된 노드(Map Database)와 간선(Edge)을 바탕으로 목표 지점까지의 최단 경로를 실시간으로 계산합니다.
* **비전 기반 차선 유지 보조 (Vision-based Lane Keeping)**: HSV 색상 필터링과 관심 영역(ROI) 설정을 통해 차선(파란색 선)을 인식하고, 이미지 모멘트를 활용하여 조향 오차를 보정합니다.
* **듀얼 네비게이션 모드 (Dual Navigation Mode)**:
  * `VISION Mode`: 장거리 직진 도로에서 카메라 데이터를 기반으로 차선을 따라 주행합니다.
  * `BLIND Mode`: 교차로 통과 및 회전 시, 목표 노드와의 각도(Yaw) 및 오도메트리(Odometry)를 계산하여 하드코딩된 조향각으로 안전하게 회전합니다.
* **실시간 2D 맵 UI (Real-Time 2D Map UI)**: `matplotlib` 및 `networkx`를 활용하여 현재 차량의 위치와 A* 알고리즘으로 생성된 경로를 별도의 창에 실시간으로 시각화합니다.
* **Isaac Sim & ROS 2 Bridge**: 고품질 물리 엔진인 Isaac Sim 환경에서 카메라 영상(`/camera_left/image_raw`)과 오도메트리(`/odom`)를 받아, 차량 제어 명령(`/ackermann_cmd`)을 퍼블리시합니다.

---

## 🏗️ 시스템 설계 (System Architecture)

* **Simulator (NVIDIA Isaac Sim)**
  * `map_car.py` 스크립트를 통해 `map.usd`와 `ackermann_car_fixed_cam.usd`를 로드합니다.
  * **Sensors**: Camera (RGB), Odometry
  * **Actuators**: Ackermann Steering Controller
* **ROS 2 Node (`autonomous_nav_node`)**
  * **Subscribers**: 
    * `/camera_left/image_raw` (sensor_msgs/Image) - 차선 인식용
    * `/odom` (nav_msgs/Odometry) - 차량 위치 및 자세 추정용
    * `/set_goal` (std_msgs/String) - 목적지 수신용
  * **Publishers**:
    * `/ackermann_cmd` (ackermann_msgs/AckermannDriveStamped) - 차량 조향 및 속도 제어
    * `/camera_left/lane_overlay` (sensor_msgs/Image) - 디버깅용 차선 인식 결과 이미지

---

## 🔄 알고리즘 플로우 차트 (Logic Flow)

1. **목적지 입력**: GUI Prompt 또는 `/set_goal` 토픽을 통해 목적지 문자열 수신.
2. **경로 탐색**: A* 알고리즘으로 `start` -> `goal` 까지의 노드 리스트 생성.
3. **주행 루프 시작 (0.05초 주기)**:
   * **현재 구간 판별**: 현재 노드와 다음 노드가 '교차로(Intersection)'나 '센터(Center)'인지 판별.
   * **모드 분기**:
     * **[VISION 모드]**: 직진 구간. OpenCV로 차선의 중심점을 찾아 `lane_offset` 계산 -> 오차에 따라 PID(P제어 기반) 조향 및 속도 조절.
     * **[BLIND 모드]**: 교차로/회전 구간. 현재 Yaw 값과 목표 노드의 각도를 비교하여 Type 1/2/3의 고정 조향각(`fixed_turn_steer`) 적용.
4. **노드 도달 확인**: `/odom` 기반 현재 좌표와 목표 노드 간의 거리가 허용 오차(`tolerance`) 이내인지 확인.
5. **업데이트**: 다음 노드로 타겟 변경 및 3번으로 회귀 (최종 목적지 도착 시 주행 종료).

---

## 📂 디렉토리 구조 (Directory Structure)

    src/project/
    ├── project/
    │   ├── line_detecing.py      # ROS 2 자율주행 알고리즘 노드
    │   └── map_car.py            # Isaac Sim 맵 및 차량 로드 스크립트
    └── resource/                 # 3D 모델 및 에셋 디렉토리
        ├── map.usd
        ├── ackermann_car_fixed_cam.usd
        └── assets/
        
### assets은 https://github.com/NVIDIA-Omniverse/sample-ackermann-amr/tree/main 에서 다운
### map.zip은 /resource 안에 압축해제
---

## 💻 개발 환경 (Environment)

* **OS**: Ubuntu 22.04 LTS
* **Middleware**: ROS 2 Humble
* **Simulator**: NVIDIA Isaac Sim
* **Language**: Python 3.10+

---

## 🛠️ 사용 장비 (Hardware Setup)

* **CPU**: (사용하신 CPU 모델명 입력, 예: Intel Core i7)
* **GPU**: NVIDIA RTX 시리즈 (Isaac Sim 구동을 위해 필수)
* **RAM**: 32GB 이상 권장

---

## 📦 의존성 설치 (Installation)

ROS 2 Humble 및 Isaac Sim이 설치되어 있어야 하며, 추가적인 Python 패키지 설치가 필요합니다.

    # 필요한 Python 패키지 설치
    pip3 install numpy opencv-python matplotlib networkx

---

## 🚀 실행 순서 (How to Run)

시스템은 원활한 연동을 위해 두 개의 터미널을 분리하여 실행합니다.

### 1. 시뮬레이터 환경 실행 (Terminal 1)

Isaac Sim을 구동하고 맵과 차량을 스폰합니다. ROS 2 Bridge 익스텐션이 포함되어 있습니다.

    # ROS 2 및 Isaac Sim 환경 변수 설정
    export ROS_DISTRO=humble
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/rokey/isaacsim/exts/isaacsim.ros2.bridge/humble/lib

    # Isaac Sim 파이썬 스크립트 실행
    /home/rokey/isaacsim/python.sh /home/rokey/IsaacSim-ros_workspaces/humble_ws/src/project/project/map_car.py

*(기다리면 Isaac Sim 창이 열리고 맵과 차량이 로드됩니다.)*

### 2. 자율주행 알고리즘 노드 실행 (Terminal 2)

시뮬레이터가 완전히 로드된 후, 자율주행 노드를 실행합니다.

    # ROS 2 워크스페이스 환경 설정
    source /opt/ros/humble/setup.bash

    # 스크립트가 있는 디렉토리로 이동
    cd /home/rokey/IsaacSim-ros_workspaces/humble_ws/src/project/project

    # 자율주행 노드 실행
    python3 line_detecing.py

* **실행 시 참고사항**: 실행 시 목적지 입력을 묻는 작은 GUI 창이 뜹니다. (예: `fire_station`, `home`, `opistel` 등)
* 입력 후 확인을 누르면 차량이 자율주행을 시작하며, 2D 맵 UI 창을 통해 실시간 위치 및 경로를 확인할 수 있습니다.
