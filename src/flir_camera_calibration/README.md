# flir_camera_calibration

체스보드 기반 캘리브레이션을 위한 보조 패키지다.

## 역할

- `/image_rgb/compressed`를 받아 체스보드 검출
- 검출된 보드를 preview로 보여주고 annotated compressed 토픽 퍼블리시
- 샘플을 모아 calibration YAML 저장

## 입력과 출력

입력:

- `/image_rgb/compressed`

출력:

- `/calibration/image_annotated/compressed`
- `calibration/flir_camera_info.yaml`
- `calibration/captures/` 내부 캡처 이미지

## 조작 키

- `space`: 현재 보드 샘플 캡처
- `c`: `cv::calibrateCamera()` 실행 후 YAML 저장
- `r`: 샘플 초기화
- `q` 또는 `Esc`: 종료

## 동작 방식

- preview는 입력 프레임이 들어오는대로 바로 갱신된다.
- live preview 검출은 성능 때문에 downscale + FAST_CHECK를 쓴다.
- 실제 캡처 시에는 원본 full-resolution에서 다시 정밀 검출한다.
- 기본 preview 검출 폭은 `640px`이다.

## 실행

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

체스보드 크기 override 예:

```bash
roslaunch flir_camera_calibration noetic_calibration.launch \
  input_topic:=/camera0/image_rgb/compressed \
  camera_serial:=25415248 \
  camera_name:=camera0 \
  frame_id:=camera0_optical_frame \
  board_cols:=6 \
  board_rows:=5 \
  square_size_m:=0.08
```

## 결과 활용

저장된 `calibration/flir_camera_info.yaml`은 `flir_spinnaker_camera`의 `camera_info_yaml_path` launch 인자로 연결할 수 있다.

```bash
roslaunch flir_spinnaker_camera noetic_flir_camera.launch \
  camera_info_yaml_path:=calibration/flir_camera_info.yaml
```

## 멀티캠 intrinsic calibration

멀티캠에서는 `src/flir_spinnaker_camera/config/multicam_cameras.yaml`의
`camera0`, `camera1` inventory를 기준으로 대상 카메라를 고른다.

```bash
roslaunch flir_camera_calibration noetic_multicam_calibration.launch camera_name:=camera0
roslaunch flir_camera_calibration noetic_multicam_calibration.launch camera_name:=camera1
roslaunch flir_camera_calibration noetic_multicam_calibration.launch camera_name:=camera2
```

저장 결과는 `calibration/flir_camera_info.yaml`의 `camera_info_by_serial` 아래에
카메라 serial별 entry로 upsert된다. 캡처 이미지는 기본적으로
`calibration/captures/<serial>/` 아래에 저장된다.

## 멀티캠 extrinsic calibration

여러 카메라가 같은 체커보드를 overlap region에서 볼 수 있게 둔 뒤 실행한다.

```bash
roslaunch flir_camera_calibration noetic_multicam_extrinsic_calibration.launch
```

노드는 각 `/cameraN/image_rgb/compressed`와 `/cameraN/camera_info`를 구독한다.
기본 모드에서는 보드를 현재 같이 보고 있는 카메라가 2대 이상이면 observation을
수동 수집하고, 이렇게 모은 pairwise graph를 `reference_camera` 기준 rig frame으로
합성한다.

- `space`: 현재 observation 수동 캡처
- `c`: `calibration/flir_camera_extrinsics.yaml` 저장
- `r`: observation 초기화
- `q` 또는 `Esc`: 종료

기본 기준은 `camera0`이며, 저장된 transform은 `flir_rig_frame` 기준
`cameraN_optical_frame` pose로 기록된다.
예를 들어 `camera0-camera1` overlap에서 `space`를 `min_observations` 이상 누르고,
이어서 `camera1-camera2` overlap에서도 같은 만큼 캡처하면 `camera2` pose는
`camera0 -> camera1 -> camera2` graph path로 합성된다. 연결되지 않은 카메라가 있거나
graph path의 가장 약한 edge observation 수가 `min_observations`보다 적으면 저장을
거부한다. 모든 카메라가 동시에 보드를 볼 때만 캡처하려면
`require_all_cameras_for_capture:=true`를 넘긴다.

저장된 `calibration/flir_camera_extrinsics.yaml`을 TF로 publish하려면:

```bash
roslaunch flir_spinnaker_camera noetic_extrinsics_tf.launch
```

## 메모

- `board_cols`, `board_rows`는 square 개수가 아니라 `inner corners` 기준이다.
- `sample_image_dir`가 비어 있지 않으면 캡처 원본도 저장된다.
- 기본 `calibration/captures/` 이미지는 gitignore 처리되어 있다.
