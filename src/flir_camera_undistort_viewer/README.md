# flir_camera_undistort_viewer

캘리브레이션 결과를 적용한 왜곡 보정 compressed 이미지를 다시 퍼블리시하는 패키지다.

## 역할

- `/image_rgb/compressed` 구독
- `/camera_info` 구독
- 왜곡 보정 후 `/image_rgb/undistorted/compressed` 퍼블리시

## 입력과 출력

입력:

- `/image_rgb/compressed`
- `/camera_info`

출력:

- `/image_rgb/undistorted/compressed`

## 동작 방식

- compressed 이미지를 디코드
- `/camera_info`의 `K`, `D`, `R`, `P`로 undistort map 생성/캐시
- 왜곡 보정 후 다시 compressed 이미지로 인코드
- 입력이 JPEG면 JPEG로, PNG면 PNG로 다시 출력

## 실행

```bash
export FLIR_ROS_DISTRO=noetic
source scripts/setup_flir_env.bash
catkin build flir_camera_undistort_viewer
roslaunch flir_camera_undistort_viewer noetic_undistort_viewer.launch
```

토픽 override 예:

```bash
roslaunch flir_camera_undistort_viewer noetic_undistort_viewer.launch \
  input_topic:=/image_rgb/compressed \
  camera_info_topic:=/camera_info \
  output_topic:=/image_rgb/undistorted/compressed
```

## 멀티캠 실행

Noetic에서는 카메라별로 node 이름과 topic을 지정해서 viewer를 띄운다.

```bash
roslaunch flir_camera_undistort_viewer noetic_undistort_viewer.launch \
  node_name:=camera0_undistort_viewer \
  input_topic:=/camera0/image_rgb/compressed \
  camera_info_topic:=/camera0/camera_info \
  output_topic:=/camera0/image_rgb/undistorted/compressed

roslaunch flir_camera_undistort_viewer noetic_undistort_viewer.launch \
  node_name:=camera1_undistort_viewer \
  input_topic:=/camera1/image_rgb/compressed \
  camera_info_topic:=/camera1/camera_info \
  output_topic:=/camera1/image_rgb/undistorted/compressed
```

각 viewer는 namespace 내부의 relative topic을 사용한다.

- `/camera0/image_rgb/compressed` + `/camera0/camera_info`
- `/camera0/image_rgb/undistorted/compressed`
- `/camera1/image_rgb/compressed` + `/camera1/camera_info`
- `/camera1/image_rgb/undistorted/compressed`

## 메모

- 기본 출력 QoS는 `best_effort`다.
- `rqt_image_view`에서는 `/image_rgb/undistorted/compressed`를 바로 열면 된다.
- 이 패키지는 `/camera_info`가 먼저 정상적으로 들어와 있어야 의미 있게 동작한다.
