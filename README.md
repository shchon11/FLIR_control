# FLIR ROS Workspace

FLIR/Teledyne Spinnaker 카메라를 ROS 2 Humble에서 쓰기 위한 워크스페이스다.

이 저장소는 크게 세 가지를 맡는다.

- 카메라 스트리밍
- 체스보드 기반 캘리브레이션
- 캘리브레이션 결과를 적용한 왜곡 보정 스트림

## 패키지 맵

### `flir_spinnaker_camera`

메인 카메라 노드다.

- 실제 FLIR 카메라를 Spinnaker로 열고 프레임을 받음
- `/image_raw`, `/camera_info`, `/image_raw/metadata`, `/image_rgb/compressed` 퍼블리시
- `camera_info.yaml_path`를 통해 저장된 캘리브레이션 YAML을 `/camera_info`에 반영 가능

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
source scripts/setup_flir_env.bash
colcon build --symlink-install --packages-select flir_spinnaker_camera
ros2 launch flir_spinnaker_camera flir_camera.launch.py
```

### 2. 캘리브레이션 수행하기

```bash
source scripts/setup_flir_env.bash
colcon build --symlink-install --packages-select flir_camera_calibration
ros2 launch flir_camera_calibration calibration.launch.py
```

OpenCV 창에서:

- `space`: 현재 보드 샘플 캡처
- `c`: 캘리브레이션 실행 및 YAML 저장
- `r`: 샘플 초기화
- `q` 또는 `Esc`: 종료

### 3. 캘리브레이션 결과를 메인 카메라에 반영하기

`src/flir_spinnaker_camera/config/flir_camera.yaml`에 아래처럼 넣거나 launch 인자로 넘기면 된다.

```yaml
camera_info.yaml_path: "calibration/flir_camera_info.yaml"
```

### 4. 왜곡 보정된 스트림 퍼블리시하기

```bash
source scripts/setup_flir_env.bash
colcon build --symlink-install --packages-select flir_camera_undistort_viewer
ros2 launch flir_camera_undistort_viewer undistort_viewer.launch.py
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
source scripts/setup_flir_env.bash
colcon build --symlink-install
ros2 launch flir_spinnaker_camera multicam.launch.py
```

토픽은 아래처럼 분리된다.

- `/camera0/image_rgb/compressed`
- `/camera0/camera_info`
- `/camera1/image_rgb/compressed`
- `/camera1/camera_info`

카메라별 intrinsic calibration:

```bash
ros2 launch flir_camera_calibration multicam_calibration.launch.py camera_name:=camera0
ros2 launch flir_camera_calibration multicam_calibration.launch.py camera_name:=camera1
```

결과는 `calibration/flir_camera_info.yaml`의 `camera_info_by_serial` 아래에 serial별로 저장된다.
한 카메라를 다시 calibration해도 다른 serial entry는 유지된다.

멀티캠 undistort viewer:

```bash
ros2 launch flir_camera_undistort_viewer multicam_undistort_viewer.launch.py
```

출력:

- `/camera0/image_rgb/undistorted/compressed`
- `/camera1/image_rgb/undistorted/compressed`

멀티캠 runtime control:

```bash
python3 scripts/multicam_param_set.py camera.ExposureAuto Off
python3 scripts/multicam_param_set.py camera.GainAuto Off
```

이 스크립트는 inventory의 모든 camera namespace에 같은 `ros2 param set`을 적용한다.

멀티캠 extrinsic calibration:

```bash
ros2 launch flir_camera_calibration multicam_extrinsic_calibration.launch.py
```

overlap region에 체커보드를 두고, 모든 카메라에서 보드와 `CameraInfo`가 잡힌 상태에서
OpenCV 창의 `space`를 눌러 observation을 수동 캡처한다. `c`를 누르면
`calibration/flir_camera_extrinsics.yaml`에 `flir_rig_frame -> cameraN_optical_frame`
transform 결과가 저장된다.

### ROS 2 bag을 nuScenes 포맷으로 변환하기

`bags/` 아래의 최신 ROS 2 bag을 image-only nuScenes 레이아웃으로 변환한다.
기본값은 undistorted export다. bag에 `/cameraN/image_rgb/undistorted/compressed`
토픽이 있으면 그 토픽을 우선 사용하고, 없으면 `/cameraN/image_rgb/compressed`를
`CameraInfo`로 왜곡 보정해서 `samples/<CAM_CHANNEL>/`에 저장한다.
nuScenes devkit이 읽는 `v1.0-mini/*.json` 테이블도 함께 생성된다.

```bash
python3 scripts/bag_to_nuscenes.py --overwrite
```

기본 출력 위치:

```text
nuscenes_export/<bag_name>/
  samples/CAM_CAMERA0/*.jpg
  samples/CAM_CAMERA1/*.jpg
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

## 환경과 빌드

필수 환경:

- Ubuntu
- ROS 2 Humble
- Spinnaker SDK 4.x
- OpenCV

환경 스크립트:

```bash
source scripts/setup_flir_env.bash
```

이 스크립트는:

- ROS 2 Humble source
- `/opt/spinnaker`가 있으면 `SPINNAKER_ROOT` 설정
- `install/setup.bash` source
- `FLIR_ROS_WS` export

전체 빌드:

```bash
source scripts/setup_flir_env.bash
colcon build --symlink-install
```

## 빠른 판단 기준

- 카메라 연결/토픽 자체가 궁금하면 `flir_spinnaker_camera`
- 보드 잡히는지 보고 캘리브레이션하려면 `flir_camera_calibration`
- 캘리브레이션 결과로 왜곡 보정된 영상을 다시 쓰려면 `flir_camera_undistort_viewer`
