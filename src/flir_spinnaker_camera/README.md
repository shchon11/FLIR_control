# flir_spinnaker_camera

FLIR/Spinnaker 카메라를 ROS 2 토픽으로 내보내는 메인 패키지다.

## 역할

- 카메라 오픈 및 acquisition
- raw 이미지 퍼블리시
- CameraInfo 퍼블리시
- per-frame metadata 퍼블리시
- RGB compressed 이미지 퍼블리시

## 퍼블리시 토픽

- `image_raw`
- `camera_info`
- `image_raw/metadata`
- `image_rgb/compressed`

## 핵심 포인트

- `/image_raw`는 raw/mono/Bayer 경로다.
- `/image_rgb/compressed`는 카메라 내부 압축이 아니라 host에서 RGB 변환 후 JPEG/PNG 압축한 결과다.
- `/camera_info`는 YAML에서 읽은 calibration 값을 반영할 수 있다.
- `header.stamp`는 기본적으로 host 수신 시각을 쓴다.
- 원본 장치 timestamp는 `/image_raw/metadata.camera_timestamp_ns`에 남는다.

## 자주 보는 설정

기본 설정 파일:

- `config/flir_camera.yaml`

특히 자주 보는 항목:

- `camera_serial`
- `camera_index`
- `frame_id`
- `publisher_qos_reliability`
- `publisher_qos_depth`
- `use_camera_timestamp_in_header`
- `pixel_format`
- `buffer_handling_mode`
- `publish_raw`
- `publish_camera_info`
- `publish_metadata`
- `publish_rgb_compressed`
- `rgb_compression_format`
- `rgb_jpeg_quality`
- `rgb_png_compression_level`
- `camera_info.yaml_path`

## calibration 적용

`flir_camera_calibration`에서 만든 YAML을 바로 적용할 수 있다.

```yaml
camera_info.yaml_path: "calibration/flir_camera_info.yaml"
```

이 값이 비어 있지 않으면 저장된 `camera_info.*` 값이 `/camera_info` 퍼블리시에 반영된다.

## 실행

```bash
source scripts/setup_flir_env.bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py
```

시리얼 지정 예:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py camera_serial:=12345678
```

캘리브레이션 YAML 적용 예:

```bash
ros2 launch flir_spinnaker_camera flir_camera.launch.py \
  camera_info_yaml_path:=calibration/flir_camera_info.yaml
```

## 멀티캠 실행

멀티캠 inventory는 `config/multicam_cameras.yaml`에서 관리한다. 토픽 구분은
`camera0`, `camera1` 같은 namespace로 단순화하고, 실제 장치 매핑은 serial로 한다.

```bash
ros2 launch flir_spinnaker_camera multicam.launch.py
```

예시 토픽:

- `/camera0/image_rgb/compressed`
- `/camera0/camera_info`
- `/camera1/image_rgb/compressed`
- `/camera1/camera_info`

모든 카메라 노드는 같은 `config/flir_camera.yaml`을 사용하며, launch에서는
`camera_serial`, `frame_id`, `camera_info.yaml_path`만 카메라별로 override한다.

모든 카메라에 같은 runtime parameter를 적용할 때는 repo root에서 helper를 쓸 수 있다.

```bash
python3 scripts/multicam_param_set.py camera.ExposureAuto Off
```

## 운영 메모

- `camera.ExposureAuto`가 `Continuous`나 `Once`면 수동 `camera.ExposureTime*`은 쓰기 불가가 될 수 있다.
- `camera.GainAuto`가 `Continuous`나 `Once`면 수동 `camera.Gain*`도 쓰기 불가가 될 수 있다.
- 큰 이미지 토픽에서 subscriber 영향이 보이면 `publisher_qos_reliability`, `publisher_qos_depth`를 먼저 확인하는 편이 좋다.
- PNG는 CPU 부담이 커서 실시간 스트림에는 JPEG가 대체로 낫다.
