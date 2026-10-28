# rgbd_slam — DR + rtabmap 센서퓨전 SLAM (ROS 2 Jazzy)
휠 오도메트리 + 자이로 추측항법(DR)에 rtabmap 의 시각 오도메트리(VO) 링크와 루프 링크를 GTSAM 팩터그래프로
묶어 보정한 차체 포즈를 출력함


RGB-D ──► rgbd_odometry ──/vo/odom──► rtabmap ──/mapGraph, /info──┐
IMU (자이로) ────────────────────────────────────────► slam_node ◄─┘ ──► /fused_odom(현재 차체 포즈 쿼터니언), /current_pose(오일러각), /fused_path(최적화 과거 궤적 전체)
휠 오도메트리 ───────────────────────────────────────►


- rgbd_odometry·rtabmap 은 apt 의 공식 rtabmap 0.22.1 그대로 쓴다.
- slam_node: IMU 틱마다 DR 적분, 10 틱(10 Hz)마다 그래프 노드 생성. 팩터는 셋(dr, vo, loop)이고 DR 외엔 `param.yaml` 에서 켜고 끔.


  평소 iSAM2, 루프가 붙으면 배치 최적화.
  rtabmap의  /mapGraph는 이웃링크, 글로벌링크, 근접링크를 포함함.
    근접 링크(ProximityBySpace)는 꺼 둠. VO 가 로스트돼도 DR 로 이어짐.

## 설정 파일 (config/)
실수 값은 정수라도 소수점을 붙일 것 (`45`x  `45.0`o).

* param.yaml: ⓪ 입력 토픽 전부(IMU, 휠, RGB, depth, camera_info) ① 차체↔카메라 변환(`cam_rpy_deg`, `lever_arm`, `base_height`) ② 입력·출력 좌표계(`enu_input`, `enu_output`) ③ 초반 자동 측정(`gyro_bias_calib`, `mount_calib` …) ④ 팩터별 노이즈 ⑤ 팩터 on/off ⑥ 루프 잔차 게이트  ⑦ 그래프·입력 처리 (graph_idx 등)
* rtab.yaml: rgbd_odometry·rtabmap: VO·루프 특징점(`gftt` \| `sift`), SIFT 임계, 키프레임 주기·루프 임계 
* mount_examples/: 자칼로 찍은 데이터셋에서 쓰던 사례 

## 입력 / 출력
입력 토픽 이름은 전부 `param.yaml`에서 수정

* IMU `/gv7/imu/data` 
* 휠 `/j100_0548/platform/odom` 
* RGB `/camera/camera/color/image_raw` 
* depth `/camera/camera/aligned_depth_to_color/image_raw` 
* `/camera/camera/color/camera_info` 


* `/fused_odom`: nav_msgs/Odometry | IMU 매 틱 — 차체 절대포즈 | 쿼터니언
* `/current_pose`: rgbd_slam/CurrentPose | IMU 매 틱 — 위와 같은 포즈 | 위치 m + roll/pitch/yaw deg (ZYX)
* `/fused_path`: nav_msgs/Path | 루프로 배치 최적화가 돈 때 — 전체 노드 궤적 | 쿼터니언

실행 중 `[VO] 로스트 시작/복귀`, `[루프] …`(채택), `[루프거부] …`(잔차 게이트), `[사전측정] …` 이 찍히고,
종료 시 요약(노드·VO엣지·루프 수, DR/Fused 시작→종점 변위, VO 로스트)을 찍음.(디버그용으로 쓰던거)

## 빌드
```bash
sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-gtsam
mkdir -p ~/ws/src && ln -s <이 폴더> ~/ws/src/rgbd_slam
cd ~/ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select rgbd_slam --symlink-install && source install/setup.bash
```
GTSAM 을 소스로 따로 설치하지 말 것 (rtabmap 과 충돌). 혹은 버전 조정해야 함.

## 실행
`bag:=` 을 주면 bag 재생, 안 주면 실제 센서 토픽을 받음.

```bash
# bag — bag 시계로 돌고, 재생이 끝나면 저절로 종료
ros2 launch rgbd_slam rgbd_slam.launch.py bag:=<bag폴더> mount:=3

# 실시간 — Ctrl-C 로 종료
ros2 launch rgbd_slam rgbd_slam.launch.py mount:=3
```

둘 다 종료 때 slam_node 가 요약을 찍고, rtabmap 지도(db)는 매 실행 새로 만듬.
bag 이면 시작 전에 `param.yaml` 의 토픽이 bag 에 있는지 확인해 없는 것을 찍음.

| 인자 | 기본 | 내용 |
|---|---|---|
| `bag` | (빈칸 = 실시간) | ros2 bag 폴더 |
| `mount` | (없음) | 장착값 yaml. 파일 경로 또는 `config/mount_examples/` 의 이름(`1`, `2`, `3`) |
| `param`, `rtab` | `config/` 의 것 | 다른 설정 파일 |
| `rviz` | false | Fused 궤적 보기 (`config/rgbd_slam.rviz`) |
| `viz` | false | rtabmap_viz (특징점 매칭) |
| `rate`, `delay` | 1.0, 5.0 | bag 재생 속도, 노드 기동 후 재생까지 [s] |
| `qos_image` | 0 | 영상 QoS 0=기본 1=reliable 2=best_effort |
| `rgb_transport`, `depth_transport` | raw | WiFi 로 받을 때 `compressed`, `compressedDepth` |
| `odom_output` | own_log | rgbd_odometry 로그를 화면에도 내려면 `screen` |

## 좌표계 · 장착값
내부는 차체 FRD(x 전방, y 우측, z 하방), 글로벌 = 시작 시점 차체 자세 (`include/rgbd_slam/frames.hpp`).
`cam_rpy_deg` = 차체 기준 카메라 roll, pitch, yaw [deg] (항공 ZYX, 하방 틸트 = pitch 음수),
`lever_arm` = 차체원점 → 카메라 광학원점 [m] FRD (위쪽 마운트 = z 음수).

사전에 마운트 측정해 `param.yaml`(또는 mount yaml)에 넣고, `mount_calib: true`(기본 false) 로 자동 측정도 가능 (아래). 자동 측정은 사전값과 5°·3 cm 이상
다르면 거부

### 초반 자동 측정 (`src/premeasure.cpp`)
기본은 사전값(`gyro_bias`, `cam_rpy_deg`·`lever_arm`)을 씀. 켜면 시작 후 `premeasure_sec`(기본 5 s) 동안
정지(휠 |v| ≤ 0.01 m/s, |ω| < 1 deg/s)가 이어질 때만 재서 바꾸고, 그 전에 움직이면 사전값을 유지한다.

* `gyro_bias_calib: true` 일 때: 자이로 바이어스 = 정지 동안 자이로 평균 
* `mount_calib: true` 일 때: depth 바닥 평면으로 카메라 roll, pitch, lever z (x, y, yaw 는 사전값) 

장착은 평면이 하나뿐일 때만 적용함. 정지 구간에서 고르게 최대 30프레임을 RANSAC 평면 적합해

- 모든 프레임의 평면 인라이어 비율(거리 1 cm 이내) ≥ `mount_inlier_min`(0.90)
- 프레임 간 법선 흔들림 ≤ 1°, 높이 흔들림 ≤ 1 cm, 프레임 10개 이상
- 사전값과 차이 roll·pitch ≤ 5°, lever z ≤ 3 cm (바닥이 아닌 벽 등을 잰 경우를 막음)

이면 적용하고, 하나라도 어기면 이유와 측정값을 찍고 사전값을 씀. lever z 는 카메라 높이에서 `base_height`
(차체원점의 지면 높이)를 빼서 구한다. `base_height` 는 플랫폼별 값. — 기본 0.0635 m 는 자칼 base_link 기준. (차체원점을 바닥에 두면 0).













## 검증 환경
Ubuntu 24.04 / ROS 2 Jazzy / gcc 13.3.0 / cmake 3.28.3
`ros-jazzy-rtabmap` (+`ros`/`msgs`/`odom`/`slam`) **0.22.1-1noble**,
`ros-jazzy-gtsam` **4.2.0-4noble**, libeigen3-dev 3.4.0.

정상 상태 확인용
```bash
ldd $(ros2 pkg prefix rgbd_slam)/lib/rgbd_slam/slam_node | grep gtsam
ldd /opt/ros/jazzy/lib/rtabmap_slam/rtabmap | grep gtsam
# 둘 다 /opt/ros/jazzy/lib/x86_64-linux-gnu/libgtsam.so.4 여야 한다
ls /usr/local/lib | grep -i gtsam   # 아무것도 안 나와야 한다
```

- 타일 바닥·빠른 곡선 주행에서 VO 로스트가 잦고 rtabmap 루프도 적게 잡힘. 로스트 동안은 DR 만으로 간다.
- rgbd_odometry 가 RGB·depth 시각 차 경고를 낼 수 있음(`approx_sync`).
- 실시간 실행은 bag 보다 덜 검증.
