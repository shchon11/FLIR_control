# FLIR Multicam Extension Plan

이 문서는 현재 단일 FLIR 카메라 기준의 `calibration`, `viewer`, `control` 구조를
멀티 카메라 환경으로 확장할 때의 권장 구조를 정리한다.

핵심 방향은 다음과 같다.

- 모든 카메라는 같은 camera config 값을 사용한다.
- 카메라별로 달라지는 값은 시리얼 번호, namespace, frame id 같은 identity 값으로만 분리한다.
- `calibration/flir_camera_info.yaml`은 FLIR 카메라 시리얼 번호를 key로 하여 intrinsic calibration 결과를 저장한다.
- extrinsic calibration은 rig/world 기준 좌표계와 각 카메라 optical frame 사이의 transform으로 별도 관리한다.

## 현재 구조 요약

현재 패키지들은 단일 카메라를 기본 전제로 한다.

- `flir_spinnaker_camera`
  - Spinnaker 장치 1개를 열고 `image_raw`, `camera_info`, `image_raw/metadata`, `image_rgb/compressed`를 publish한다.
  - `camera_serial`이 비어 있으면 `camera_index`로 카메라를 선택한다.
  - `camera_info.yaml_path`에서 `camera_info.*` 값을 읽어 `/camera_info`에 반영한다.
- `flir_camera_calibration`
  - compressed image 1개를 구독한다.
  - 체스보드 샘플을 모아 `calibration/flir_camera_info.yaml`을 저장한다.
- `flir_camera_undistort_viewer`
  - compressed image 1개와 `camera_info` 1개를 구독한다.
  - 왜곡 보정 compressed image 1개를 publish한다.

멀티캠에서는 노드 로직을 한 노드가 여러 카메라를 직접 관리하는 방식보다, 카메라별 동일 노드를 namespace로 여러 번 띄우는 방식이 더 단순하고 ROS 2 관례에도 맞다.

## 목표 구조

권장 구성은 아래처럼 나눈다.

```text
config/
  flir_camera.yaml                  # 모든 카메라가 공유하는 동일한 장치/스트림 설정
  multicam_cameras.yaml             # 카메라 inventory: serial, name(camera0...), namespace, frame_id
  multicam_calibration.yaml         # calibration 실행 공통 설정
  multicam_viewer.yaml              # viewer 실행 공통 설정

calibration/
  flir_camera_info.yaml             # serial별 intrinsic camera_info 결과
  flir_camera_extrinsics.yaml       # rig/world 기준 serial별 extrinsic 결과
  captures/
    <serial>/                       # intrinsic calibration 캡처 이미지
  extrinsic_captures/
    <session_id>/                   # extrinsic calibration 캡처/검출 결과

launch/
  multicam.launch.py                # 카메라 N개 실행
  multicam_calibration.launch.py    # 특정 serial 또는 전체 serial 대상 calibration 실행
  multicam_undistort_viewer.launch.py
  multicam_rig.launch.py            # camera + viewer + static/dynamic TF publish 조합
```

실제 repo 구조에서는 기존 패키지별 `config/`, `launch/` 밑에 파일을 둘 수 있다. 여러 패키지를 함께 묶는 상위 실행 파일이 필요하면 별도 패키지 `flir_multicam_bringup`을 추가하는 것이 가장 깔끔하다.

## Camera Config 원칙

모든 카메라가 같은 설정을 가져야 하므로 `flir_camera.yaml`은 하나의 shared profile로 취급한다.

동일해야 하는 값:

- pixel format, color processing, buffer handling mode
- raw/RGB/compressed publish 여부
- compression format과 품질
- acquisition frame rate
- exposure, gain, white balance, gamma, ROI, trigger, bandwidth, stream buffer 설정
- publisher QoS
- timestamp 정책

카메라별로 달라도 되는 identity 값:

- `camera_serial`
- namespace
- node name
- `frame_id`
- calibration lookup key

즉 `camera_serial`은 카메라 설정값이라기보다 장치 선택용 identity로 본다. `flir_camera.yaml` 안에 시리얼별 설정을 넣지 않고, 멀티캠 launch가 shared config를 읽은 뒤 각 카메라 노드에 serial/namespace/frame id만 override한다.

토픽 이름은 물리적 위치인 `front_left`, `front_right`를 직접 넣지 않고 `camera0`,
`camera1` 같은 logical slot으로 단순화한다. 실제 장치 매핑은 serial이 책임지고,
물리적 배치 정보는 extrinsic calibration 또는 별도 metadata에서 해석한다.

예시 inventory:

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
      - name: camera2
        serial: "24100003"
        namespace: "camera2"
        frame_id: "camera2_optical_frame"
```

토픽은 namespace 아래로 자연스럽게 분리한다.

```text
/camera0/image_rgb/compressed
/camera0/camera_info
/camera0/image_rgb/undistorted/compressed

/camera1/image_rgb/compressed
/camera1/camera_info
/camera1/image_rgb/undistorted/compressed
```

## Intrinsic Calibration YAML

`calibration/flir_camera_info.yaml`은 단일 카메라 ROS parameter file 형태에서 serial-indexed registry 형태로 확장한다.
이 파일은 runtime CameraInfo publish에 직접 필요한 intrinsic 값을 책임진다.

권장 형식:

```yaml
version: 2
camera_info_by_serial:
  "24100001":
    camera_name: "camera0"
    frame_id: "camera0_optical_frame"
    camera_info:
      distortion_model: "plumb_bob"
      d: [-0.29, 0.07, -0.002, -0.001, -0.008]
      k: [856.0, 0.0, 958.8, 0.0, 848.9, 776.4, 0.0, 0.0, 1.0]
      r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      p: [856.0, 0.0, 958.8, 0.0, 0.0, 848.9, 776.4, 0.0, 0.0, 0.0, 1.0, 0.0]
      binning_x: 0
      binning_y: 0
      roi:
        x_offset: 0
        y_offset: 0
        height: 0
        width: 0
        do_rectify: false
    calibration:
      generated_at_utc: "2026-05-09T00:00:00Z"
      image_width: 1936
      image_height: 1464
      board_cols: 6
      board_rows: 5
      square_size_m: 0.08
      captured_frames: 21
      rms_reprojection_error: 1.69
      mean_reprojection_error: 1.69

  "24100002":
    camera_name: "camera1"
    frame_id: "camera1_optical_frame"
    camera_info:
      distortion_model: "plumb_bob"
      d: [0.0, 0.0, 0.0, 0.0, 0.0]
      k: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      p: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    calibration:
      generated_at_utc: ""
      image_width: 0
      image_height: 0
      board_cols: 6
      board_rows: 5
      square_size_m: 0.08
      captured_frames: 0
      rms_reprojection_error: 0.0
      mean_reprojection_error: 0.0
```

구현 시 변경점:

- calibration 노드는 `camera_serial` 파라미터를 받아 현재 결과를 해당 serial entry에 upsert한다.
- 기존처럼 파일 전체를 덮어쓰지 않고, 다른 serial의 calibration 결과를 유지한다.
- camera 노드는 `camera_info.yaml_path`와 `camera_serial`을 함께 사용해 자기 serial entry만 읽는다.
- viewer는 지금처럼 namespace 안의 `image_rgb/compressed`와 `camera_info`만 구독하면 되므로 큰 변경이 필요 없다.

기존 단일 카메라 YAML과의 호환이 필요하면 loader는 `version` 또는 `camera_info_by_serial` 존재 여부로 v1/v2를 구분한다. v1 파일은 기존 top-level `flir_camera.ros__parameters.camera_info.*`를 그대로 읽고, v2 파일은 serial entry를 읽는다.

## Intrinsic Calibration Flow

카메라별 intrinsic calibration은 한 번에 하나씩 수행하는 흐름을 기본으로 둔다.

1. shared camera config로 전체 카메라 또는 대상 카메라만 실행한다.
2. calibration launch에 `camera_serial` 또는 `camera_name`을 넘긴다.
3. launch가 inventory에서 namespace를 찾아 입력 토픽을 자동으로 정한다.
4. calibration 노드는 `calibration/captures/<serial>/`에 샘플을 저장한다.
5. `c` 키로 calibration을 수행하면 `calibration/flir_camera_info.yaml`의 해당 serial entry만 갱신한다.
6. camera 노드를 재시작하거나 reload 기능을 추가해 해당 serial의 `/camera_info`를 갱신한다.

예시 실행 형태:

```bash
ros2 launch flir_camera_calibration multicam_calibration.launch.py \
  camera_serial:=24100001
```

## Undistort Viewer 구조

viewer는 카메라별로 독립 노드를 띄운다.

```text
/camera0/flir_camera_undistort_viewer
  input_topic: image_rgb/compressed
  camera_info_topic: camera_info
  output_topic: image_rgb/undistorted/compressed

/camera1/flir_camera_undistort_viewer
  input_topic: image_rgb/compressed
  camera_info_topic: camera_info
  output_topic: image_rgb/undistorted/compressed
```

이 구조에서는 viewer가 serial-indexed YAML을 직접 읽지 않는다. camera 노드가 자기 serial에 맞는 `CameraInfo`를 publish하고, viewer는 ROS 토픽 계약만 따른다. 이렇게 해야 camera info source가 YAML에서 다른 calibration manager로 바뀌어도 viewer가 영향을 덜 받는다.

## Control 구조

control은 모든 카메라에 같은 설정을 적용하는 제어면과, 카메라별 상태를 관찰하는 상태면을 분리한다.

권장 방식:

- shared config file은 모든 카메라 노드의 초기 파라미터 source다.
- runtime control API는 같은 파라미터 변경을 모든 카메라 namespace에 broadcast한다.
- 특정 카메라만 다른 exposure/gain/ROI로 바꾸는 기능은 기본 정책상 제공하지 않는다.
- 예외가 필요하면 일시적 diagnostic override로만 다루고 shared config에 저장하지 않는다.

예시 control 노드 역할:

- inventory를 읽고 활성 카메라 목록을 관리한다.
- `set_shared_parameter(name, value)` 요청을 받으면 모든 카메라 노드에 같은 값을 적용한다.
- 적용 실패 카메라가 있으면 전체 operation을 실패로 기록하고 어떤 serial이 실패했는지 보고한다.
- 카메라별 metadata, heartbeat, frame rate, dropped frame 같은 상태는 serial별로 publish한다.

## Extrinsic Calibration

멀티 카메라에서는 각 카메라의 intrinsic만으로는 충분하지 않다. 공통 rig frame 또는 world frame 기준으로 각 카메라 optical frame의 위치/자세를 구해야 한다.

권장 좌표계:

```text
flir_rig_frame
  -> camera0_optical_frame
  -> camera1_optical_frame
  -> camera2_optical_frame
```

권장 저장 파일:

```yaml
version: 1
reference_frame: "flir_rig_frame"
extrinsics_by_serial:
  "24100001":
    camera_name: "camera0"
    parent_frame: "flir_rig_frame"
    child_frame: "camera0_optical_frame"
    translation_xyz_m: [0.0, 0.0, 0.0]
    rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
    calibration:
      method: "multi_camera_chessboard"
      generated_at_utc: "2026-05-09T00:00:00Z"
      rms_reprojection_error: 0.0
      observations: 0

  "24100002":
    camera_name: "camera1"
    parent_frame: "flir_rig_frame"
    child_frame: "camera1_optical_frame"
    translation_xyz_m: [0.12, 0.0, 0.0]
    rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
    calibration:
      method: "multi_camera_chessboard"
      generated_at_utc: "2026-05-09T00:00:00Z"
      rms_reprojection_error: 0.0
      observations: 0
```

`flir_camera_info.yaml`에 모든 calibration artifact를 한 파일로 모아야 한다면 같은 schema의
`extrinsics_by_serial` 섹션을 최상위에 함께 둘 수 있다. 다만 CameraInfo loader는
`camera_info_by_serial`만 읽고, extrinsic publisher는 `extrinsics_by_serial`만 읽도록 책임을
분리하는 편이 안전하다. 기본 권장은 CameraInfo와 TF transform의 의미가 다르므로
`flir_camera_info.yaml`과 `flir_camera_extrinsics.yaml`을 분리하는 것이다.

Extrinsic calibration pipeline:

1. 모든 카메라의 intrinsic calibration을 먼저 완료한다.
2. 동기화된 프레임 또는 시간 차이가 충분히 작은 프레임 묶음을 수집한다.
3. 여러 카메라에서 동시에 보이는 체스보드, ChArUco, AprilTag board 중 하나를 사용해 각 카메라 기준 target pose를 구한다.
4. 공통 target observation들을 이용해 카메라 간 transform을 계산한다.
5. 하나의 기준 카메라 또는 별도 `flir_rig_frame`을 기준으로 transform tree를 정한다.
6. 결과를 `calibration/flir_camera_extrinsics.yaml`에 serial별로 저장한다.
7. runtime에서는 static transform publisher 또는 별도 extrinsic publisher 노드가 이 YAML을 읽어 `/tf_static`에 publish한다.

체스보드만 사용할 수도 있지만, 멀티캠 extrinsic에는 ChArUco 또는 AprilTag board가 더 운영하기 쉽다. 보드 일부만 보여도 식별이 가능하고, 카메라 간 공통 observation 매칭이 명확하기 때문이다.

## Launch 설계

`multicam.launch.py`는 inventory를 읽어 카메라 수만큼 같은 노드를 생성한다.

각 카메라 노드에 주입하는 override:

- `camera_serial`
- `frame_id`
- `camera_info.yaml_path`
- namespace

카메라별로 바꾸지 않는 값:

- `params_file`
- exposure/gain/ROI/trigger/pixel format 등 shared camera config의 모든 장치 설정

개념 예:

```python
for camera in cameras:
    Node(
        package="flir_spinnaker_camera",
        executable="flir_spinnaker_camera_node",
        name="flir_camera",
        namespace=camera["namespace"],
        parameters=[
            shared_camera_params_file,
            {
                "camera_serial": camera["serial"],
                "frame_id": camera["frame_id"],
                "camera_info.yaml_path": calibration_yaml_path,
            },
        ],
    )
```

viewer launch도 같은 inventory를 순회하되, 각 namespace 내부의 relative topic을 사용한다.

```python
Node(
    package="flir_camera_undistort_viewer",
    executable="flir_camera_undistort_viewer_node",
    name="flir_camera_undistort_viewer",
    namespace=camera["namespace"],
    parameters=[viewer_params_file],
)
```

## 구현 순서

1. `multicam_cameras.yaml` inventory 형식을 추가한다.
2. `flir_camera_info.yaml` v2 serial-indexed schema를 추가하고 기존 v1 loader와 호환되게 만든다.
3. camera 노드가 `camera_serial`로 자기 intrinsic entry를 선택하게 한다.
4. calibration 노드에 `camera_serial` 파라미터와 serial entry upsert 저장을 추가한다.
5. `multicam.launch.py`로 카메라 N개를 shared config 기반으로 실행한다.
6. `multicam_undistort_viewer.launch.py`로 viewer N개를 namespace별로 실행한다.
7. extrinsic capture/calibration 도구를 추가한다.
8. `flir_camera_extrinsics.yaml` loader와 `/tf_static` publisher를 추가한다.
9. README에 single-camera와 multi-camera workflow를 분리해서 문서화한다.

## 검증 기준

Intrinsic 검증:

- 각 serial에 대해 `calibration/flir_camera_info.yaml` entry가 생성된다.
- 한 serial calibration 저장 시 다른 serial entry가 유지된다.
- 각 camera namespace의 `/camera_info`가 자기 serial의 K/D/R/P 값을 publish한다.
- viewer output이 namespace별로 정상 publish된다.

Shared config 검증:

- 모든 camera node가 같은 `flir_camera.yaml`을 사용한다.
- launch override에는 serial/namespace/frame id/calibration path만 존재한다.
- runtime control에서 exposure/gain/ROI 변경이 필요한 경우 모든 카메라에 동일하게 적용된다.

Extrinsic 검증:

- `flir_camera_extrinsics.yaml`의 모든 child frame이 실제 camera `frame_id`와 일치한다.
- `/tf_static`에 `flir_rig_frame -> <camera_optical_frame>` transform이 모두 publish된다.
- 여러 카메라에서 같은 calibration target을 관측했을 때 reprojection error가 기준 이하로 유지된다.
- 카메라 순서가 바뀌어도 serial key 기준으로 같은 calibration이 선택된다.

## 주요 설계 결정

- 멀티캠은 "멀티 카메라를 관리하는 거대한 단일 노드"가 아니라 "같은 단일 카메라 노드의 namespace별 복수 실행"으로 구성한다.
- camera config는 shared profile 하나만 유지하고, 카메라별 identity/inventory만 분리한다.
- intrinsic은 `flir_camera_info.yaml`에 serial별로 저장한다.
- extrinsic은 CameraInfo가 표현하지 못하는 rig transform이므로 별도 `flir_camera_extrinsics.yaml`로 저장하고 TF로 publish한다.
- viewer는 calibration 파일을 직접 알지 않고 `/camera_info`만 신뢰한다.
