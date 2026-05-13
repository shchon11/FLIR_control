# FLIR ROS Noetic Workspace

FLIR/Teledyne Spinnaker 카메라를 ROS 1 Noetic에서 쓰기 위한 워크스페이스다.

이 저장소는 크게 세 가지를 맡는다.

- 카메라 스트리밍
- 체스보드 기반 캘리브레이션
- 캘리브레이션 결과를 적용한 왜곡 보정 스트림

## 환경과 빌드

필수 환경:

- Ubuntu
- ROS 1 Noetic
- [Spinnaker SDK 4.x](https://www.teledynevisionsolutions.com/support/support-center/software-firmware-downloads/iis/spinnaker-sdk-download/spinnaker-sdk--download-files/?pn=Spinnaker+SDK&vn=Spinnaker+SDK)
- OpenCV
- linuxptp/PTP를 쓸 경우 `ptp4l`

새 PC에서 처음 받을 때:

```bash
sudo apt install python3-pip python3-catkin-tools python3-rosdep linuxptp
python3 -m pip install --user -r requirements.txt
```

`rosdep`를 처음 쓰는 PC라면 한 번 초기화한다.

```bash
sudo rosdep init
rosdep update
```

ROS 의존성은 `package.xml` 기준으로 설치한다.

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Spinnaker SDK는 위 Teledyne/FLIR 공식 다운로드 페이지에서 OS에 맞는 배포본을 받아
설치해야 한다. 기본 위치가 아니면 `SPINNAKER_ROOT`를 직접 지정한다.

```bash
export SPINNAKER_ROOT=/opt/spinnaker
```

카메라 NIC와 PTP 권한은 helper로 맞출 수 있다.

```bash
scripts/setup_camera_nic.bash --interface enp5s0 --host-cidr 192.168.1.10/24
```

환경 스크립트는 이 브랜치에서 Noetic을 우선 source한다.
명시적으로 고르려면 `FLIR_ROS_DISTRO=noetic`을 지정한다.

```bash
export FLIR_ROS_DISTRO=noetic
source scripts/setup_flir_env.bash
```

이 스크립트는:

- ROS 1 Noetic source
- 워크스페이스의 `.spinnaker-*/opt/spinnaker` 또는 `/opt/spinnaker` 기준 `SPINNAKER_ROOT` 설정
- Spinnaker library path와 `SPINNAKER_GENTL64_CTI` 설정
- `devel/setup.bash` source
- `FLIR_ROS_WS` export

Noetic 빌드:

```bash
export FLIR_ROS_DISTRO=noetic
source scripts/setup_flir_env.bash
catkin build
source devel/setup.bash
```

Noetic 경로는 현재 카메라 스트리밍, 멀티캠, extrinsic TF, undistort viewer를 지원한다.

```bash
roslaunch flir_spinnaker_camera noetic_flir_camera.launch
roslaunch flir_spinnaker_camera noetic_multicam.launch
roslaunch flir_spinnaker_camera noetic_extrinsics_tf.launch

roslaunch flir_camera_undistort_viewer noetic_undistort_viewer.launch \
  input_topic:=/camera0/image_rgb/compressed \
  camera_info_topic:=/camera0/camera_info \
  output_topic:=/camera0/image_rgb/undistorted/compressed
```

## 패키지 맵

### `flir_spinnaker_camera`

메인 카메라 노드다.

- 실제 FLIR 카메라를 Spinnaker로 열고 프레임을 받음
- `/image_raw`, `/camera_info`, `/image_raw/metadata`, `/image_rgb/compressed` 퍼블리시
- `camera_info_yaml_path` launch 인자로 저장된 캘리브레이션 YAML을 `/camera_info`에 반영 가능

자세한 내용:

- [`src/flir_spinnaker_camera/README.md`](src/flir_spinnaker_camera/README.md)

### `flir_camera_calibration`

캘리브레이션 샘플 수집용 보조 패키지다.

- `/image_rgb/compressed`를 구독
- 체스보드가 보이면 `/calibration/image_annotated/compressed` 퍼블리시
- `space`로 샘플 캡처, `c`로 캘리브레이션 실행
- 결과를 `calibration/flir_camera_info.yaml`로 저장

자세한 내용:

- [`src/flir_camera_calibration/README.md`](src/flir_camera_calibration/README.md)

### `flir_camera_undistort_viewer`

캘리브레이션 결과를 적용한 왜곡 보정 스트림 퍼블리셔다.

- `/image_rgb/compressed`와 `/camera_info`를 구독
- 왜곡을 푼 결과를 `/image_rgb/undistorted/compressed`로 퍼블리시
- 출력은 기본 `best_effort`

자세한 내용:

- [`src/flir_camera_undistort_viewer/README.md`](src/flir_camera_undistort_viewer/README.md)

## 추천 워크플로

### 1. 카메라만 먼저 보기

```bash
export FLIR_ROS_DISTRO=noetic
source scripts/setup_flir_env.bash
catkin build flir_spinnaker_camera
roslaunch flir_spinnaker_camera noetic_flir_camera.launch
```

### 2. 캘리브레이션 수행하기

```bash
export FLIR_ROS_DISTRO=noetic
source scripts/setup_flir_env.bash
catkin build flir_camera_calibration
roslaunch flir_camera_calibration noetic_calibration.launch \
  input_topic:=/image_rgb/compressed \
  camera_serial:=25415255 \
  camera_name:=flir_camera \
  frame_id:=flir_camera_optical_frame
```

OpenCV 창에서:

- `space`: 현재 보드 샘플 캡처
- `c`: 캘리브레이션 실행 및 YAML 저장
- `r`: 샘플 초기화
- `q` 또는 `Esc`: 종료

### 3. 캘리브레이션 결과를 메인 카메라에 반영하기

Noetic launch 인자로 넘긴다.

```bash
roslaunch flir_spinnaker_camera noetic_flir_camera.launch \
  camera_info_yaml_path:=calibration/flir_camera_info.yaml
```

### 4. 왜곡 보정된 스트림 퍼블리시하기

```bash
export FLIR_ROS_DISTRO=noetic
source scripts/setup_flir_env.bash
catkin build flir_camera_undistort_viewer
roslaunch flir_camera_undistort_viewer noetic_undistort_viewer.launch
```

`rqt_image_view`에서는 `/image_rgb/undistorted/compressed`를 바로 열면 된다.

## 멀티캠 워크플로

멀티캠은 `camera0`, `camera1` 같은 단순 namespace로 구분한다. 실제 장치 매핑은
`src/flir_spinnaker_camera/config/multicam_cameras.yaml`의 serial이 책임진다.

카메라 inventory 예:

```yaml
flir_multicam:
  ros__parameters:
    cameras:
      - name: camera0
        serial: "24100001"
        namespace: "camera0"
        frame_id: "camera0_optical_frame"
      - name: camera1
        serial: "24100002"
        namespace: "camera1"
        frame_id: "camera1_optical_frame"
```

멀티캠 실행:

```bash
### TERMINAL 1 - PC를 PTP Grandmaster로 설정
sudo ptp4l -i enp3s0f1 -m -S

### TERMINAL 2
export FLIR_ROS_DISTRO=noetic
source scripts/setup_flir_env.bash
catkin build flir_spinnaker_camera
roslaunch flir_spinnaker_camera noetic_multicam.launch
```

`Unable to contact my own server`가 나오면 현재 PC에 없는 `ROS_IP`가 잡힌 상태다.
환경 스크립트를 다시 source하면 잘못된 `ROS_IP`를 지우고 local master로 맞춘다.

```bash
export FLIR_ROS_DISTRO=noetic
source scripts/setup_flir_env.bash
roslaunch flir_spinnaker_camera noetic_multicam.launch
```

토픽은 아래처럼 분리된다.

- `/camera0/image_rgb/compressed`
- `/camera0/camera_info`
- `/camera1/image_rgb/compressed`
- `/camera1/camera_info`

카메라별 intrinsic calibration:

```bash
roslaunch flir_camera_calibration noetic_multicam_calibration.launch camera_name:=camera0
roslaunch flir_camera_calibration noetic_multicam_calibration.launch camera_name:=camera1
roslaunch flir_camera_calibration noetic_multicam_calibration.launch camera_name:=camera2
```

결과는 `calibration/flir_camera_info.yaml`의 `camera_info_by_serial` 아래에 serial별로 저장된다.
한 카메라를 다시 calibration해도 다른 serial entry는 유지된다.

멀티캠 undistort viewer:

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

roslaunch flir_camera_undistort_viewer noetic_undistort_viewer.launch \
  node_name:=camera2_undistort_viewer \
  input_topic:=/camera2/image_rgb/compressed \
  camera_info_topic:=/camera2/camera_info \
  output_topic:=/camera2/image_rgb/undistorted/compressed
```

출력:

- `/camera0/image_rgb/undistorted/compressed`
- `/camera1/image_rgb/undistorted/compressed`

멀티캠 runtime control:

```bash
source scripts/setup_flir_env.bash
python3 scripts/multicam_param_set.py camera.ExposureAuto Off
python3 scripts/multicam_param_set.py camera.ExposureTime 8000
python3 scripts/multicam_param_set.py camera.GainAuto Off
python3 scripts/multicam_param_set.py stream.StreamBufferCountManual 64
```

이 helper는 각 `/cameraN/set_camera_control` service를 호출해서 실행 중인
Spinnaker node map에 값을 적용한다. 한 카메라만 바꾸려면
`--camera camera1`처럼 이름, namespace, serial 중 하나를 지정한다.

멀티캠 extrinsic calibration:

```bash
roslaunch flir_camera_calibration noetic_multicam_extrinsic_calibration.launch
```

overlap region에 체커보드를 두고, 보드를 같이 보는 카메라 pair마다 OpenCV 창의
`space`를 눌러 observation을 수동 캡처한다. 예를 들어 `camera0-camera1`,
`camera1-camera2`처럼 겹치는 구간을 각각 `min_observations` 이상 모으면 노드가
pairwise graph를 `reference_camera` 기준 rig frame으로 합성한다. `c`를 누르면
`calibration/flir_camera_extrinsics.yaml`에 `flir_rig_frame -> cameraN_optical_frame`
transform 결과가 저장된다. 저장된 TF를 publish하려면 아래를 실행한다.

```bash
roslaunch flir_spinnaker_camera noetic_extrinsics_tf.launch
```

### ROS 1 bag 기록하기

기본 저장 위치는 `bags/` 아래이고, bag 이름은 timestamp를 붙여 충돌을 피한다.
nuScenes 변환만 할 거면 compressed RGB, `camera_info`, metadata를 같이 기록한다.

```bash
mkdir -p bags

rosbag record \
  -O bags/flir_multicam_$(date +%Y%m%d_%H%M%S).bag \
  /camera0/image_rgb/compressed /camera0/camera_info /camera0/image_raw/metadata \
  /camera1/image_rgb/compressed /camera1/camera_info /camera1/image_raw/metadata \
  /camera2/image_rgb/compressed /camera2/camera_info /camera2/image_raw/metadata
```

raw Bayer까지 같이 남겨서 나중에 raw/RGB를 함께 export하려면 `image_raw`도 포함한다.

```bash
mkdir -p bags

rosbag record \
  -O bags/flir_multicam_raw_rgb_$(date +%Y%m%d_%H%M%S).bag \
  /camera0/image_raw /camera0/image_rgb/compressed /camera0/camera_info /camera0/image_raw/metadata \
  /camera1/image_raw /camera1/image_rgb/compressed /camera1/camera_info /camera1/image_raw/metadata \
  /camera2/image_raw /camera2/image_rgb/compressed /camera2/camera_info /camera2/image_raw/metadata
```

### nuScenes 변환 참고

`scripts/bag_to_nuscenes.py`는 Noetic `rosbag`으로 기록한 ROS 1 `.bag`을 바로 읽는다.
ROS 2 sqlite3 `.db3` bag도 입력으로 줄 수 있지만, 그 경우에는 ROS 2 `rclpy` 환경이 필요하다.
기본값은 undistorted export다. bag에 `/cameraN/image_rgb/undistorted/compressed`
토픽이 있으면 그 토픽을 우선 사용하고, 없으면 `/cameraN/image_rgb/compressed`를
`CameraInfo`로 왜곡 보정해서 `samples/<CAM_CHANNEL>/`에 저장한다.
nuScenes devkit이 읽는 `v1.0-mini/*.json` 테이블도 함께 생성된다.

```bash
python3 scripts/bag_to_nuscenes.py --overwrite
```

raw Bayer와 RGB를 같이 보고 싶으면 raw/RGB stream을 모두 export한다. 이때 raw는
demosaic하거나 왜곡 보정하지 않고, RGGB 위치별로 색만 입힌 원본 Bayer mosaic PNG로
저장한다. RGB stream은 기본대로 undistorted 이미지를 우선 사용하거나 변환 중 왜곡 보정한다.

```bash
python3 scripts/bag_to_nuscenes.py --raw-and-rgb --overwrite
```

viewer에서는 bag에 들어있는 camera namespace 수만큼 채널을 자동으로 나열한다.
raw/RGB를 같이 export한 경우 raw 채널과 RGB 채널을 함께 선택해서 비교할 수 있다.
bag 안에 `/cameraN/image_raw`가 없으면 raw viewer용 export는 만들 수 없다.
raw/RGB를 모두 남기려면 bag 기록 시 `/cameraN/image_raw`, `/cameraN/image_rgb/compressed`,
`/cameraN/camera_info`를 같이 record한다.

기본 출력 위치:

```text
nuscenes_export/<bag_name>/
  samples/CAM_CAMERA0/*.jpg
  samples/CAM_CAMERA1/*.jpg
  samples/CAM_CAMERA2/*.jpg
  v1.0-mini/*.json
  conversion_report.json
```

nuScenes 표준 채널명으로 저장하고 싶으면 카메라 namespace를 명시적으로 매핑한다.

```bash
python3 scripts/bag_to_nuscenes.py \
  --camera-channel camera0=CAM_FRONT \
  --camera-channel camera1=CAM_FRONT_RIGHT \
  --overwrite
```

변환기는 bag의 `/cameraN/camera_info`를 우선 사용하고, 없으면
`calibration/flir_camera_info.yaml`을 fallback으로 사용한다. Extrinsic은
`calibration/flir_camera_extrinsics.yaml`의 `rotation_xyzw`/`translation_xyz_m`을
nuScenes `calibrated_sensor` 테이블에 넣는다.
원본 이미지를 그대로 내보내야 할 때만 `--no-undistort`를 붙인다.

변환된 이미지를 timestamp 순서로 localhost에서 확인하려면:

```bash
python3 scripts/serve_nuscenes_images.py
```

기본적으로 최신 `nuscenes_export/<bag_name>`을 찾아 `http://127.0.0.1:8000`에
띄운다. 특정 export를 보고 싶으면 dataset root를 인자로 넘긴다.

```bash
python3 scripts/serve_nuscenes_images.py nuscenes_export/flir_multicam_20260509_200435
```

뷰어는 현재 timestamp의 이미지만 크게 보여준다. 키보드 방향키로 이전/다음
timestamp 이미지로 이동하고, `사진 목록`을 펼쳐 원하는 timestamp를 직접 선택할 수 있다.

## 토픽 흐름

기본 흐름은 아래처럼 보면 된다.

1. `flir_spinnaker_camera`
2. `/image_rgb/compressed`
3. `flir_camera_calibration`
4. `calibration/flir_camera_info.yaml`
5. `flir_spinnaker_camera`의 `/camera_info`
6. `flir_camera_undistort_viewer`
7. `/image_rgb/undistorted/compressed`

## 주요 산출물

- `calibration/flir_camera_info.yaml`: 캘리브레이션 결과
- `calibration/captures/`: 캡처 샘플 이미지

`calibration/captures/`는 gitignore 처리되어 있다.

## 빠른 판단 기준

- 카메라 연결/토픽 자체가 궁금하면 `flir_spinnaker_camera`
- 보드 잡히는지 보고 캘리브레이션하려면 `flir_camera_calibration`
- 캘리브레이션 결과로 왜곡 보정된 영상을 다시 쓰려면 `flir_camera_undistort_viewer`
