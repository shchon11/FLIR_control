# flir_spinnaker_camera

FLIR/Spinnaker 카메라를 ROS 1 Noetic 토픽으로 내보내는 메인 패키지다.

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
- `camera_info_yaml_path`

## calibration 적용

`flir_camera_calibration`에서 만든 YAML을 바로 적용할 수 있다.

```yaml
camera_info_yaml_path:=calibration/flir_camera_info.yaml
```

이 값이 비어 있지 않으면 저장된 `camera_info.*` 값이 `/camera_info` 퍼블리시에 반영된다.

## 실행

```bash
export FLIR_ROS_DISTRO=noetic
source scripts/setup_flir_env.bash
catkin build flir_spinnaker_camera
source devel/setup.bash
roslaunch flir_spinnaker_camera noetic_flir_camera.launch
```

시리얼 지정 예:

```bash
roslaunch flir_spinnaker_camera noetic_flir_camera.launch camera_serial:=12345678
```

캘리브레이션 YAML 적용 예:

```bash
roslaunch flir_spinnaker_camera noetic_flir_camera.launch \
  camera_info_yaml_path:=calibration/flir_camera_info.yaml
```

## 멀티캠 실행

멀티캠 inventory는 `config/multicam_cameras.yaml`에서 관리한다. 토픽 구분은
`camera0`, `camera1` 같은 namespace로 단순화하고, 실제 장치 매핑은 serial로 한다.

```bash
roslaunch flir_spinnaker_camera noetic_multicam.launch
```

`Unable to contact my own server`가 나오면 현재 PC에 없는 `ROS_IP`가 잡힌 상태다.
repo root에서 `source scripts/setup_flir_env.bash`를 다시 실행한 뒤 launch한다.

Noetic에서 extrinsic TF만 따로 띄울 수도 있다.

```bash
roslaunch flir_spinnaker_camera noetic_extrinsics_tf.launch
```

예시 토픽:

- `/camera0/image_rgb/compressed`
- `/camera0/camera_info`
- `/camera1/image_rgb/compressed`
- `/camera1/camera_info`

모든 카메라 노드는 같은 `config/flir_camera.yaml`을 사용하며, launch에서는
`camera_serial`, `frame_id`, `camera_info_yaml_path`만 카메라별로 override한다.

연결된 Spinnaker 카메라를 launch 전에 감지해서 inventory YAML을 자동 갱신할 수도 있다.
기존 serial 설정은 유지하고, 새로 감지된 serial만 `cameraN` 항목으로 추가한다.

```bash
# Noetic에서는 inventory tool을 직접 실행한다.
rosrun flir_spinnaker_camera flir_multicam_inventory_tool \
  --output src/flir_spinnaker_camera/config/multicam_cameras.yaml \
  --dry-run
```

카메라별 ForceIP도 함께 채우려면 시작 IP를 준다. 아래 예시는 새 카메라들에
`192.168.1.206`, `192.168.1.207`, ... 순서로 `force_ip_address`를 채운다.

```bash
rosrun flir_spinnaker_camera flir_multicam_inventory_tool \
  --output src/flir_spinnaker_camera/config/multicam_cameras.yaml \
  --force-ip-base 192.168.100.206 \
  --force-ip-subnet-mask 255.255.255.0 \
  --force-ip-gateway 0.0.0.0 \
  --dry-run
```

쓰기 없이 결과만 확인하려면 inventory tool을 직접 dry-run으로 실행한다.

```bash
rosrun flir_spinnaker_camera flir_multicam_inventory_tool \
  --output src/flir_spinnaker_camera/config/multicam_cameras.yaml \
  --force-ip-base 192.168.100.206 \
  --new-hardware-trigger-role none \
  --new-ptp-action-role receiver \
  --first-camera-ptp-sender \
  --dry-run
```

자동 갱신은 YAML을 정규화해서 다시 쓰기 때문에 기존 주석은 보존하지 않는다.

## GigE ForceIP

카메라가 다른 subnet이나 `169.254.x.x` link-local 주소로 뜨면, Noetic 노드가
카메라 `Init()` 전에 Spinnaker ForceIP를 실행해서 serial별 IP를 임시 할당할 수
있다. 카메라가 Spinnaker 목록에 보이는 상태여야 한다.

멀티캠에서는 `config/multicam_cameras.yaml`에 카메라별 주소를 적는다.

```yaml
- name: camera0
  serial: "25415248"
  namespace: "camera0"
  frame_id: "camera0_optical_frame"
  force_ip_address: "192.168.100.207"
  force_ip_subnet_mask: "255.255.255.0"
  force_ip_gateway: "0.0.0.0"
```

`noetic_multicam.launch`는 `config/flir_camera.yaml`의
`network.force_ip.enable` 값을 기본으로 따른다. launch에서 켜려면
`force_ip_enable:=true`를 명시한다. 이미 원하는 IP이고 wrong-subnet 상태가
아니면 건너뛴다.

```bash
roslaunch flir_spinnaker_camera noetic_multicam.launch force_ip_enable:=true
```

단일 카메라 테스트:

```bash
roslaunch flir_spinnaker_camera noetic_flir_camera.launch \
  camera_serial:=25415248 \
  force_ip_address:=192.168.100.206
```

## BFS GPIO HW trigger

멀티캠 GPIO 케이블을 쓸 때는 `config/multicam_cameras.yaml`에서 카메라별 역할을 지정한다.

```yaml
- name: camera0
  serial: "25415248"
  namespace: "camera0"
  frame_id: "camera0_optical_frame"
  hardware_trigger_role: "master"
- name: camera1
  serial: "25415255"
  namespace: "camera1"
  frame_id: "camera1_optical_frame"
  hardware_trigger_role: "slave"
```

`master`는 `Line1`을 `Output`으로 두고 `ExposureActive`를 출력하며, `Line2` 3.3V를 켠다.
`slave`는 `FrameStart`를 `Line3` rising edge trigger로 설정하고 마지막에 `TriggerMode=On`으로 arm한다.
멀티캠 launch는 slave 노드를 먼저 띄우고 master 노드는 기본 1초 늦게 시작한다.

```bash
roslaunch flir_spinnaker_camera noetic_multicam.launch
```

Noetic launch에서는 `hardware_trigger_master_start_delay` 인자를 쓰지 않는다.

```bash
roslaunch flir_spinnaker_camera noetic_multicam.launch
```

## Runtime control

Noetic 카메라 노드는 카메라 namespace마다 `set_camera_control` service를 제공한다.
scope는 `camera`, `stream`, `tl_device` 중 하나이고, helper에서는 `camera.ExposureAuto`
같은 prefix로 지정할 수 있다.

```bash
python3 scripts/multicam_param_set.py camera.ExposureAuto Off
python3 scripts/multicam_param_set.py camera.ExposureTime 8000
python3 scripts/multicam_param_set.py --camera camera1 camera.GainAuto Off
python3 scripts/multicam_param_set.py stream.StreamBufferCountManual 64
```

직접 service를 호출할 수도 있다.

```bash
rosservice call /camera0/set_camera_control "scope: 'camera'
name: 'GainAuto'
value: 'Off'"
```

## PTP scheduled action trigger

HW trigger 케이블 없이 GigE 카메라들을 맞출 때는 PC NIC를 PTP grandmaster로 두고,
카메라는 PTP slave + `Action0` FrameStart trigger로 arm한다. PTP master 자체는
카메라 노드가 아니라 OS의 linuxptp가 담당한다.

예시:

```bash
sudo ptp4l -i enp3s0f1 -m -S
```

다른 터미널에서 카메라 launch를 실행한다.

```bash
roslaunch flir_spinnaker_camera noetic_multicam.launch
```

현재 장비에서는 카메라 NIC가 `enp3s0f1`이다.

```bash
sudo ptp4l -i enp3s0f1 -m -S
```

`ptp4l` 명령이 없으면 `linuxptp` 패키지를 설치해야 한다.

```bash
sudo apt install linuxptp
```

카메라 NIC가 hardware timestamping을 지원하지 않으면 launch 기본값처럼
`ptp4l_timestamping:=software`를 사용한다. `ptp4l`을 sudo 없이 launch에서
띄우려면 권한도 한 번 부여해야 한다.

```bash
sudo setcap cap_net_raw,cap_net_admin,cap_net_bind_service,cap_sys_time+ep $(command -v ptp4l)
```

launch는 기본적으로 `ptp4l`의 local management socket을 `/tmp/ptp4l-flir`에 만든다.
기본 `/var/run/ptp4l`은 일반 사용자 권한으로 bind할 수 없기 때문이다.

카메라 NIC의 host IP는 subnet의 network/broadcast 주소가 아니어야 한다. 예를 들어
`192.168.1.0/24` 대신 `192.168.1.10/24`처럼 잡는다.

```bash
scripts/setup_camera_nic.bash --interface enp5s0 --host-cidr 192.168.1.10/24
```

이 helper는 PC NIC에 잘못 붙은 `192.168.1.0/24` alias를 제거하고, PC host IP
`192.168.1.10/24`를 추가하며, `ptp4l` capability도 같이 맞춘다. 스위치 관리 주소가
`.0`인 것은 건드리지 않는다.

멀티캠 inventory에서 HW trigger 역할을 끄고 PTP action 역할을 지정한다.
`sender`는 정확히 한 노드에만 둔다. sender도 receiver 설정을 함께 적용하므로
해당 카메라도 같은 action command로 촬영된다.

```yaml
- name: camera0
  serial: "25415248"
  namespace: "camera0"
  frame_id: "camera0_optical_frame"
  hardware_trigger_role: "none"
  ptp_action_role: "sender"
- name: camera1
  serial: "25415255"
  namespace: "camera1"
  frame_id: "camera1_optical_frame"
  hardware_trigger_role: "none"
  ptp_action_role: "receiver"
```

공통 설정은 `config/flir_camera.yaml`의 `ptp.*`와 `ptp_action.*`에서 조정한다.
Noetic launch는 PTP/action 관련 launch arg가 비어 있으면 YAML 값을 그대로 쓴다.
PTP status가 `Slave`가 되면
sender가 카메라 timestamp를 latch하고 100 ms 뒤의 PTP time으로 scheduled action
command를 주기적으로 보낸다. `ptp.require_sync: true`면 `ptp4l`을 먼저 띄우지
않으면 launch가 실패한다. YAML 또는 launch override에서 `ptp_action.rate_hz` /
`ptp_action_rate_hz`를 `0.0`으로 두면 카메라의 resulting frame rate를 읽어서
가능한 rate를 자동으로 잡는다.

```bash
roslaunch flir_spinnaker_camera noetic_multicam.launch
```

단일 카메라는 Noetic 카메라 launch로 실행한다.

```bash
roslaunch flir_spinnaker_camera noetic_flir_camera.launch camera_serial:=25415248
```

## Extrinsic TF

멀티캠 launch는 기본으로 `calibration/flir_camera_extrinsics.yaml`을 읽어서
calibrated rig transform을 `/tf_static`에 publish한다.

```text
flir_rig_frame -> camera0_optical_frame
flir_rig_frame -> camera1_optical_frame
```

YAML의 `extrinsics_by_serial` 항목 중 `config/multicam_cameras.yaml` inventory에
있는 serial만 publish한다.

```bash
roslaunch flir_spinnaker_camera noetic_multicam.launch
```

필요하면 끄거나 다른 extrinsic YAML을 지정할 수 있다.

```bash
roslaunch flir_spinnaker_camera noetic_multicam.launch

roslaunch flir_spinnaker_camera noetic_extrinsics_tf.launch \
  extrinsics_yaml_path:=calibration/flir_camera_extrinsics.yaml
```

TF만 따로 띄울 때는 `noetic_extrinsics_tf.launch`를 쓴다.

```bash
roslaunch flir_spinnaker_camera noetic_extrinsics_tf.launch
```

Noetic 노드는 runtime parameter helper를 쓰지 않는다.

```bash
# 시작 시 적용할 camera control은 launch/config 쪽에 넣는다.
```

## 운영 메모

- `camera.ExposureAuto`가 `Continuous`나 `Once`면 수동 `camera.ExposureTime*`은 쓰기 불가가 될 수 있다.
- `camera.GainAuto`가 `Continuous`나 `Once`면 수동 `camera.Gain*`도 쓰기 불가가 될 수 있다.
- 큰 이미지 토픽에서 subscriber 영향이 보이면 `publisher_qos_reliability`, `publisher_qos_depth`를 먼저 확인하는 편이 좋다.
- PNG는 CPU 부담이 커서 실시간 스트림에는 JPEG가 대체로 낫다.
