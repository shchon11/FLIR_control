# GNSS PPS 동기화 전환 및 검증 명세

> GNSS 수신기가 PTP grandmaster가 되고, 같은 GNSS의 PPS가 카메라 GPIO를 직접
> 트리거하는 구성으로 넘어가기 위한 작업 명세 + 검증 절차.
>
> 짝 문서: DM_Clipgui `docs/gnss_pps_sync_gui.md` — GUI 대응 명세. 여기의
> T0~T6 단계 번호를 그대로 참조한다.
>
> 작성: 2026-09-21. 하드웨어 배선 완료 시점에 맞춰 작성했으며, 아직 실측으로
> 검증되지 않았다. 각 단계의 실측값을 채워 넣으면서 갱신할 것.

## 갱신 (2026-09-24) — 이 문서에서 바뀐 것

이 명세를 따라 작업하는 동안 구성이 바뀌었다. 아래는 지금 리포에 들어가 있는 것이고,
본문에서 이와 어긋나는 부분(특히 T4·TAI·`timestamp.utc_offset_ns`)은 더 이상 맞지 않는다.

- **카메라에는 PTP 를 주지 않는다.** 노출 정렬은 GPIO 하드웨어 트리거가 한다. 그래서
  `camera_timestamp_ns` 는 카메라마다 전원을 넣은 뒤부터 세는 카운터이고, 카메라끼리 직접
  비교할 수 없다 — `scripts/check_multicam_sync.py` 대신 `scripts/check_trigger_sync.py` 를 쓴다.
- **header.stamp 는 노출 시작 시각이다** (`timestamp.mode: camera_latched`). 카메라 카운터를
  주기적으로 래치해 PC 시계로 옮기고, 노출 종료 래치를 보정한다 (`timestamp.exposure_latch: end`).
  트리거 펄스가 PPS 격자 위에 있으면 `timestamp.trigger_grid_hz` 로 그 격자에 맞춘다.
- **T-1(노출 래치 판별)은 끝났다.** Blackfly S 는 노출이 끝날 때 타임스탬프를 찍는다
  (`config/flir_camera.yaml` 의 `camera.ExposureTime` 주석에 실측 기록).
- **Orin 이 grandmaster다.** PC 는 eno1 에서 slave, `phc2sys` 로 시스템 시계를 맞춘다
  (`scripts/ptp_setup.py`, DM_clipGUI 의 `ptp` 명령). 82599 카메라 포트에는 하드웨어
  타임스탬프를 쓰지 않는다.
- **라이다는 카메라 스위치에 물려 192.168.1.200 고정**이고, PPS/NMEA 는 아직 안 물렸다.

## 1. 배선 전제

```
┌──────────────┐
│ GNSS 수신기   │
│ (PTP GM)     │
└──┬────────┬──┘
   │ PTP    │ PPS (전기 신호)
   │        │
   │        └──────────────┬──────────┬──────────┐
   │                       ▼          ▼          ▼
   │                  ┌────────┐ ┌────────┐  ... 8대
   │                  │ BFS #1 │ │ BFS #2 │
   │                  │ GPIO in│ │ GPIO in│
   │                  └────────┘ └────────┘
   ▼
┌──────────┐
│   PC     │  ptp4l = slave (GM 아님)
│  (ircv)  │
└──────────┘
```

핵심: **트리거와 타임스탬프가 분리된다.**

| 역할 | 담당 | 정밀도 |
|------|------|--------|
| 8대 노출 시점 정렬 | PPS → GPIO | 케이블 전파지연 수준 (ns급) |
| 절대 시각 부여 | PTP → 카메라 내부 시계 | ptp4l offset 수준 (µs급) |

PPS가 정렬을 담당하므로 PTP scheduled action은 더 이상 필요 없다. PTP는 오직
`GevTimestamp`를 GNSS 시간축에 올리는 용도로만 남는다.

## 2. 현재 코드가 이 구성과 어긋나는 지점

전부 설정으로 해결되며, **C++ 수정은 §6 하나뿐이다.**

| # | 위치 | 현재 값 | 새 구성에서 | 이유 |
|---|------|---------|-------------|------|
| 1 | `multicam_cameras.yaml` 8대 전부 | `ptp_action_role: sender/receiver` | `none` | PPS가 트리거를 대신함 |
| 2 | `multicam_cameras.yaml` 8대 전부 | `hardware_trigger_role: none` | `slave` | 외부 PPS를 입력으로 받음 |
| 3 | `flir_camera.yaml:288` | `ptp.enable: false` | `true` | 타임스탬프용으로 PTP 유지 |
| 4 | `multicam.launch.py:701` | `ptp_master_interface: enp3s0f1` | `""` (빈 문자열) | PC가 GM이면 안 됨 |
| 5 | `flir_camera.yaml:20` | `use_camera_timestamp_in_header: false` | `true` (§6 수정 후) | header가 노출 시점을 보게 |

### 배타 검사는 문제되지 않는다

[`flir_spinnaker_camera_node.cpp:734-740`](../src/flir_spinnaker_camera/src/flir_spinnaker_camera_node.cpp#L734-L740)의
배타 검사는 `ptp_action.role` ↔ `hardware_trigger.role` 사이에만 걸려 있다.
`ptp.enable: true` + `hardware_trigger.role: "slave"` + `ptp_action.role: "none"`은
허용되는 조합이며, 이것이 새 구성이다.

### master 카메라가 없어진다

기존 GPIO 모델은 한 대가 `ExposureActive`를 Line1로 출력해 나머지를 때리는
구조였다. PPS가 8대에 병렬로 들어가면 **전원 `slave`, master 0대**다.
[`multicam.launch.py:506-525`](../src/flir_spinnaker_camera/launch/multicam.launch.py#L506-L525)는
master 개수를 검증하지 않고 시작 순서만 조정하므로, 전원 slave여도 정상 동작한다.

### 라이다 불변식이 깨진다

[`CLAUDE.md`](../CLAUDE.md)와 [`README.md:257-260`](../README.md#L257-L260)에
"`-s`도 `phc2sys`도 없어 시스템 시계를 건드리지 않는다"가 **의도된 설계**로
기록되어 있다. 라이다(`TIME_FROM_ROS_TIME`)와 카메라(호스트 도착 시각)가 같은
시스템 시계를 읽게 하려는 것이었다.

새 구성에서는 시스템 시계가 GNSS에 규율된다. 방향은 개선이지만(라이다도 GNSS
축으로 올라감) **두 문서의 해당 서술을 반드시 갱신해야 한다.** 그리고 초기
수렴 전 step이 운행 중에 들어오면 bag 타임스탬프가 뒤로 점프하므로, §5 T0을
통과한 뒤에 노드를 띄운다.

여유가 되면 Ouster도 PPS/PTP 직접 입력을 지원하므로 같이 물리는 것을 권장한다.

## 3. 미결 분기 — 배선 확인 후 확정

### 분기 A: PPS 주파수

생 PPS는 1 Hz다. GPIO `FrameStart`를 직접 때리면 **1 fps**가 된다.

| 경우 | 조치 |
|------|------|
| GNSS 박스가 PPS 위상동기 N Hz 펄스열 출력 | 그대로 사용. `camera.ExposureTime: 18000.0`(18 ms)이 1/N초보다 작은지만 확인 |
| 진짜 1 Hz만 나옴 & 1 fps로 충분 | 그대로 사용. 디베이어 CPU 예산 대폭 여유 |
| 진짜 1 Hz인데 N Hz 필요 | BFS Counter/Timer로 체배. 깔끔하지 않음 — 별도 검토 필요 |

현재 설정은 30 Hz 기준(`ptp_action.rate_hz: 30.0`)이고 CLAUDE.md의 디베이어
CPU 예산도 8×30Hz로 잡혀 있다. 1 Hz로 내려가면 이 제약은 무의미해진다.

### 분기 B: GNSS PTP가 들어오는 NIC

| 경우 | 조치 | 난이도 |
|------|------|--------|
| 카메라망(`enp3s0f1`/192.168.1.x)에 GM이 같이 물림 | PC와 카메라가 나란히 GM을 봄. `ptp4l -i enp3s0f1 -s` 하나면 끝 | 낮음 |
| 별도 NIC로 들어옴 | PC가 boundary clock 역할(ptp4l 2포트 설정) | 높음 |

[`docs/network_layout.md`](network_layout.md)는 "8대 카메라 + PTP grandmaster가
모두 `enp3s0f1`에 물려 있다"고 기록하고 있으나, 이는 PC가 GM이던 시절의 서술이다.
배선 확정 후 갱신할 것.

### 분기 C: GPIO 입력 라인

`hardware_trigger.slave.trigger_source`의 현재 기본값은 `Line3`이다.

| 라인 | 특성 | 판단 |
|------|------|------|
| `Line0` | 옵토 절연 입력 | 전기적으로 안전하나 옵토커플러 지연/지터가 µs급으로 붙음 |
| `Line3` | 비절연 입력 | 지연·지터 최소. PPS 정밀도를 살리려면 이쪽 |

PPS의 ns급 정밀도를 살리려면 `Line3`이 맞다. 다만 GNSS 박스와 카메라의 접지가
분리되어야 하는 상황이면 `Line0`을 쓰고 지터를 감수한다. **실측(T4)의 지터
폭으로 판단할 것** — 이 선택이 맞았는지는 T4가 알려준다.

## 4. 전환 작업 체크리스트

```yaml
# ① src/flir_spinnaker_camera/config/multicam_cameras.yaml — 8대 전부
  hardware_trigger_role: "slave"      # none 에서 변경
  ptp_action_role: "none"             # sender/receiver 에서 변경

# ② src/flir_spinnaker_camera/config/flir_camera.yaml
  ptp.enable: true                    # false 에서 변경
  ptp.mode: "SlaveOnly"               # 유지
  ptp.accepted_statuses: ["Slave"]    # 유지
  hardware_trigger.slave.trigger_source: "Line3"      # 분기 C
  hardware_trigger.slave.trigger_activation: "RisingEdge"
  use_camera_timestamp_in_header: true                # §6 수정 후에만
```

```bash
# ③ ptp4l을 slave로 별도 기동 (launch가 GM으로 띄우지 않도록 인터페이스 비움)
sudo ptp4l -i enp3s0f1 -s -m --uds_address /tmp/ptp4l-flir
# T0 통과 확인 후:
ros2 launch flir_spinnaker_camera multicam.launch.py ptp_master_interface:=""
```

`ptp4l_timestamping`은 launch 기본값이 `software`인데, 이는 **PC가 GM이던 시절의
선택**이다(주석: hardware 모드에서 카메라가 Listening에 머물렀음). 외부 GM
구성에서는 토폴로지가 다르므로 hardware 모드를 재시도할 가치가 있다. T1/T4에서
판단한다.

## 5. 검증 절차 — 단계별 판정

각 단계는 앞 단계 통과를 전제한다. 실패하면 그 자리에서 해결하고 진행한다.

**T-1만 예외다.** PPS 배선 없이 현재 PTP action 리그로 수행할 수 있고, 그 결과가
§6의 설계를 좌우하므로 배선 작업 전에 끝내 두는 편이 낫다.

### T-1 — 노출 래치 시점 판별 (배선 전에, 지금 가능)

**무엇을 정하는가:** 카메라가 타임스탬프를 노출 **시작**에 찍는지 **종료**에
찍는지. 이 한 번의 측정이 세 가지를 동시에 결정한다.

| 결과 | `timestamp.subtract_exposure` (§6) | Chunk data 작업 | autoexposure |
|------|-----------------------------------|-----------------|--------------|
| 시작 래치 | 불필요 | 불필요 | 타임스탬프에 영향 없음 — 자유롭게 사용 |
| 종료 래치 | **필요** | **필요** (아래 참조) | 보정 없이는 사용 불가 |

**원리 — 차등 노출.** 같은 트리거에 노출을 시작한 두 카메라의 노출 시간을 다르게
준다. 시작 래치면 두 타임스탬프가 같고, 종료 래치면 노출 차이만큼 벌어진다.
절대 기준이 필요 없어서 PPS 없이도 성립한다.

**절차:**

```bash
# 0) 먼저 트리거 주기를 늘린다. 노출을 60 ms까지 올릴 텐데 30 Hz(33 ms 주기)에서는
#    노출이 주기를 넘어 트리거를 건너뛴다 (flir_camera.yaml 의 ptp_action.rate_hz 주석).
#    flir_camera.yaml: ptp_action.rate_hz: 10.0   → 주기 100 ms

# 1) 리그 기동 후 전 카메라를 같은 노출로 맞춘다
ros2 node list | grep camera          # 노드 이름 확인
# 각 카메라 노드에 대해:
ros2 param set /<camera_ns>/<node> camera.ExposureTime 10000.0

# 2) 기준 스프레드 측정 (A)
python3 scripts/check_multicam_sync.py --duration 20

# 3) 카메라 '한 대만' 노출을 크게
ros2 param set /camera_front_right/<node> camera.ExposureTime 60000.0

# 4) 다시 측정 (B)
python3 scripts/check_multicam_sync.py --duration 20
```

**판정:**

| 결과 | 해석 |
|------|------|
| B ≈ A (스프레드 변화 없음) | **노출 시작 래치.** §6의 노출 보정과 chunk 작업이 전부 불필요해진다 |
| B − A ≈ 50 ms (= 60 − 10) | **노출 종료 래치 확정.** sensors.yaml 의 서술이 맞았던 것 |
| B − A 가 50 ms의 일부만 | 예상 밖. 실측값을 기록하고 재검토 |

**주의:**

- `camera.ExposureAuto`가 `Off`여야 `ExposureTime`이 쓰기 가능하다 (현재 `Off`).
  `Continuous`면 노드가 "not writable"로 거부한다.
- 노출을 바꾼 카메라가 PTP action **sender**면 트리거 주기 자체에 영향을 줄 수
  있으니, receiver 카메라를 골라 바꾼다 (현재 sender는 `camera_front_right`이므로
  다른 카메라를 고르거나 sender를 먼저 옮긴다).
- 측정이 끝나면 `ptp_action.rate_hz`와 노출을 원래대로 되돌린다.

**종료 래치로 판명되면 — chunk data가 필요하다.** 현재
[`flir_spinnaker_camera_node.cpp:3212-3213`](../src/flir_spinnaker_camera/src/flir_spinnaker_camera_node.cpp#L3212-L3213)은
노출 시간을 **퍼블리시 시점에 카메라 레지스터에서 읽는다**. 고정 노출이면 우연히
맞지만 autoexposure면 프레임과 값이 어긋나고(읽는 시점이 이미 수십 ms 뒤), 프레임당
10개 노드를 읽어 8대 × 30 Hz면 초당 2400회 GigE 왕복이 된다. 해법은 Chunk Data다:

```cpp
ChunkModeActive = true
ChunkSelector = "ExposureTime";  ChunkEnable = true
// 수신 시
const double exposure_us = image->GetChunkData().GetExposureTime();
```

그 프레임의 실제 노출이 프레임에 실려 온다. 레포 전체에 chunk 사용처가 한 곳도
없으므로 신규 작업이다. **시작 래치로 판명되면 이 작업은 통째로 불필요하다.**

### T0 — PC의 PTP 상태

```bash
pmc -u -b 0 -s /tmp/ptp4l-flir 'GET PORT_DATA_SET'    # portState
pmc -u -b 0 -s /tmp/ptp4l-flir 'GET TIME_STATUS_NP'   # master_offset, gmIdentity
```

| 결과 | 의미 | 조치 |
|------|------|------|
| `portState SLAVE`, `master_offset` < 10000 (ns) | 정상 | T1로 |
| `portState MASTER` | PC가 GM을 못 봄 — GNSS 미도달 | 케이블/VLAN/GM 전원 확인 |
| `portState LISTENING` 지속 | Announce 미수신 | 도메인 번호 불일치 의심. GM의 domainNumber 확인 후 `-d` 로 맞춤 |
| `master_offset` 수 초 단위 | 첫 수렴 전 | 수렴까지 대기. **여기서 노드 띄우지 말 것** |

### T1 — 카메라 PTP Slave 도달

노드 기동 로그에서 8대 전부 확인한다. `ptp.require_sync: true`이므로 실패하면
노드가 죽으므로, 기동이 되면 통과한 것이다.

```
PTP synchronized: GevIEEE1588Status='Slave', offset_from_master=... ns.
```

| 결과 | 조치 |
|------|------|
| 8대 전부 Slave | T2로 |
| 일부가 timeout으로 죽음 | `ptp.sync_timeout_ms`는 이미 60 s. 스위치가 PTP 멀티캐스트를 막는지 확인 |
| 전부 Listening | GM 도메인 불일치 또는 `ptp4l_timestamping` 문제. hardware 모드 시도 |

A70 thermal은 제외한다 — CLAUDE.md에 ptp4l 상대로 Slave 도달 실패가 기록되어
있고 free-run + 호스트 도착 시각으로 동작한다.

### T2 — 프레임 수신 및 레이트

```bash
ros2 topic hz /camera_front_right/image_raw
```

| 결과 | 의미 | 조치 |
|------|------|------|
| 예상 레이트와 일치 | 정상 | T3으로 |
| 1 Hz (N Hz를 기대했는데) | PPS가 체배 없이 직결됨 | 분기 A 재검토 |
| 0 Hz / 타임아웃 | 트리거 미수신 | 분기 C 라인 번호 오류, 극성(`trigger_activation`), 신호 레벨 확인 |
| 기대치의 절반 | 노출이 트리거 주기보다 김 | `camera.ExposureTime`(현재 18 ms)을 주기 아래로 |

### T3 — 상대 동기 (기존 스크립트)

```bash
python3 scripts/check_multicam_sync.py --duration 20
```

[`check_multicam_sync.py`](../scripts/check_multicam_sync.py)는 그대로 유효하다.
단 **판정선이 달라진다** — 기존 1 ms 기준은 PTP scheduled action 기준이었다.

| 스프레드 | PTP action 시절 판정 | PPS 구성 판정 |
|----------|---------------------|---------------|
| < 50 µs | — | 정상 |
| 50 µs ~ 1 ms | 정상이었음 | **비정상.** PPS가 일부 카메라에 안 들어가거나 옵토 지연(분기 C) |
| > 1 ms | 느슨함 | **PPS 미도달.** free-run 중인 카메라 있음 |

스크립트의 임계값 상수도 같이 낮출 것(현재 1 ms/10 ms 분기).

### T4 — 절대 검증 (신규, 이 구성에서 처음 가능)

**PPS 엣지는 정수 초 경계에 있다.** 따라서 `camera_timestamp_ns`를 10⁹로 나눈
나머지가 0 근처의 **고정값**이어야 한다. 이것이 상대 비교를 넘어선 ground truth
대조이며, T3보다 훨씬 강한 검증이다.

```python
# 각 카메라의 metadata를 모아서
residual_ns = camera_timestamp_ns % 1_000_000_000
# 이 값의 중앙값 = 트리거 전파 + 타임스탬프 래치 지연
# 이 값의 표준편차 = 지터
```

| 잔차 중앙값 | 해석 | 조치 |
|------------|------|------|
| 0 ~ 수십 µs | 타임스탬프가 **노출 시작**에 래치됨 | 이상적. 그대로 사용 |
| ≈ 18 ms (= `ExposureTime`) | 타임스탬프가 **노출 종료**에 래치됨 | `ExposureTime`을 바꿔 잔차가 따라 움직이는지 확인. 맞으면 §6에서 노출시간을 빼서 보정 |
| 카메라마다 다름 | 케이블 길이차 또는 라인 종류 혼용 | 분기 C 확인. 고정 오프셋이면 카메라별 보정 가능 |
| **±37초 근처** | **TAI/UTC 오프셋** | §7 참조 |

| 잔차 표준편차 | 해석 |
|--------------|------|
| < 10 µs | 정상 |
| ~ 수십 µs | 옵토커플러 경유 의심 → `Line0`을 쓰고 있다면 `Line3`으로 |
| > 1 ms | 트리거가 아니라 free-run 중. T2/T3 재확인 |

`ExposureTime`을 바꿔가며 잔차가 따라 움직이는지 보는 것이 노출 시작/종료
래치를 구분하는 확실한 방법이다. 이 구성에서만 가능한 진단이므로 반드시 수행한다.

**"노출 끝에 찍힌다"는 아직 검증되지 않았다.** DM_Clipgui의
`config/sensors.yaml`에 그런 설명이 달려 있으나(`camera.ExposureTime` help,
커밋 `94df95b`, 2026-09-19), 뒷받침하는 측정 기록이 양쪽 리포 어디에도 없다.
FLIR 공식 문서도 Blackfly S의 래치 시점을 명확히 하지 않는다. **가정하지 말고
T-1로 먼저 확정한다** — T-1은 PPS 배선 없이 지금 리그로 잴 수 있다.

### T5 — header.stamp 정합 (§6 적용 후)

```bash
ros2 topic echo /camera_front_right/image_raw --field header.stamp --once
date +%s
```

| 결과 | 조치 |
|------|------|
| 시스템 시각과 일치, 8대 간 일치 | 완료 |
| 37초 차이 | §7 |
| 8대가 서로 어긋남 | §6 수정이 반영 안 됨(고정 오프셋 방식이 남아 있음) |
| 값이 뒤로 점프 | 비단조 폴백이 작동. 로그에서 "Camera timestamp moved backwards" 확인 |

### T6 — 라이다 축 정합

```bash
ros2 topic echo /ouster/points --field header.stamp --once
```

카메라 header.stamp와 같은 축에 있는지 확인한다. 라이다는
`TIME_FROM_ROS_TIME`으로 시스템 시계를 읽으므로, 시스템 시계가 GNSS에 규율되면
자동으로 맞는다. 어긋나면 phc2sys 설정을 확인한다(§7).

## 6. 코드 변경 명세 — `ResolveHeaderStamp`

**대상:** [`flir_spinnaker_camera_node.cpp:3155-3189`](../src/flir_spinnaker_camera/src/flir_spinnaker_camera_node.cpp#L3155-L3189)

**현재 동작의 문제:**

```cpp
if (!camera_timestamp_alignment_initialized_) {
  camera_timestamp_alignment_initialized_ = true;
  header_stamp_offset_ns_ = fallback_ns - camera_timestamp;   // 첫 프레임에서 한 번만
}
```

첫 프레임에서 오프셋을 잡고 그 뒤로는 고정 오프셋을 더한다. 결과:

- 카메라 내부의 프레임 간격은 디바이스 정확도로 정확해진다 (장점)
- 절대 기준점이 "그 카메라 **첫 프레임의 호스트 도착 시각**"이라, 그 한 프레임의
  전송 지연이 영구히 박힌다
- **치명적:** 카메라마다 노드가 따로 뜨므로 8대가 각자 다른 첫 프레임으로 각자
  오프셋을 잡는다. PPS로 노출이 완벽히 정렬되어도 header.stamp는 어긋난다.
  하드웨어 동기화의 이득이 header 단계에서 전부 사라진다.

**변경 방향:** 고정 오프셋 누적을 버리고 직접 변환한다.

```
header.stamp = camera_timestamp_ns
             - utc_offset_ns      (§7, 파라미터로 노출)
             - exposure_offset_ns (T4에서 노출 종료 래치로 판명된 경우만)
```

**신규 파라미터:**

| 이름 | 기본값 | 용도 |
|------|--------|------|
| `timestamp.utc_offset_ns` | `0` | TAI→UTC 보정. T4 결과로 결정 |
| `timestamp.subtract_exposure` | `false` | 노출 종료 래치 보정. T4 결과로 결정 |

**유지할 것:** 비단조 감지 폴백
(`camera_timestamp_header_disabled_due_to_instability_`)은 그대로 둔다. 다만
현재는 한 번 걸리면 **영구히** 폴백하므로, 경고 로그가 한 번만 뜨고 조용히
호스트 시각으로 돌아가 있을 수 있다. 진단을 위해 주기적 재경고를 추가한다.

**적용 순서:** T4를 먼저 수행해 두 파라미터 값을 확정한 뒤 코드를 바꾼다.
T4 없이 켜면 37초 오차를 모르고 넘어갈 수 있다.

## 7. TAI/UTC 37초 함정

**이 구성에서 처음 나타나는 문제다.**

현재는 PC가 GM이고 software timestamping이므로 PTP 시간축 = PC의
`CLOCK_REALTIME` = UTC다. 오프셋이 0이라 문제가 보이지 않는다.

GNSS GM이 들어오면 PTP 시간축이 **TAI**가 되고, 카메라 타임스탬프가 UTC보다
37초 앞선다(2026년 기준).

**현재 코드가 이 문제를 가린다.** `ResolveHeaderStamp`의 첫-프레임 오프셋 방식은
37초를 우연히 흡수하므로 겉보기에 멀쩡하다. §6의 직접 변환으로 바꾸는 순간
드러난다. 그래서 T4를 §6보다 먼저 한다.

linuxptp 버전과 타임스탬핑 모드에 따라 시스템 시계가 UTC로 내려오는지 TAI로
내려오는지가 달라진다. **가정하지 말고 T4로 실측한다.**

| 측정 결과 | 조치 |
|-----------|------|
| 잔차가 0 근처 | 이미 UTC. `utc_offset_ns: 0` |
| 잔차가 37초 근처 | `timestamp.utc_offset_ns: 37000000000` 또는 아래 정석 구성 |

정석 구성은 hardware timestamping으로 PHC를 물리고 `phc2sys -w`가 UTC 오프셋을
적용해 시스템 시계로 내리는 것이다:

```bash
sudo ptp4l -i enp3s0f1 -s -H -m --uds_address /tmp/ptp4l-flir
sudo phc2sys -s enp3s0f1 -c CLOCK_REALTIME -w -m
```

`pmc -u -b 0 -s /tmp/ptp4l-flir 'GET TIME_STATUS_NP'`의 `currentUtcOffset`으로
GM이 광고하는 값을 확인할 수 있다.

## 8. 산출물

| 항목 | 상태 |
|------|------|
| `multicam_cameras.yaml` 역할 전환 (§4①) | 배선 확인 후 |
| `flir_camera.yaml` PTP 활성화 (§4②) | 배선 확인 후 |
| **T-1 노출 래치 판별 (§5)** | **배선 전 지금 가능 — 최우선** |
| Chunk data 노출 취득 (T-1이 종료 래치일 때만) | T-1 후 |
| T4 잔차 측정 스크립트 (`scripts/check_pps_phase.py`) | 신규 작성 필요 |
| `check_multicam_sync.py` 임계값 하향 (§5 T3) | T3 실측 후 |
| `ResolveHeaderStamp` 직접 변환 (§6) | T4 확정 후 |
| `CLAUDE.md` / `README.md` 라이다 불변식 서술 갱신 (§2) | 전환 확정 후 |
| `docs/network_layout.md` GM 위치 갱신 (분기 B) | 배선 확인 후 |
| **DM_Clipgui**: `~/FLIR_control` 경로 (GUI가 못 띄움) | **최우선** |
| **DM_Clipgui**: `_fix_sync_roles` 외부 트리거 대응 | **최우선** |
| **DM_Clipgui**: 동기 검증 패널 (T3/T4를 GUI로) | `check_pps_phase.py` 후 |

### 부수적으로 확인된 문서 드리프트

[`README.md:193`](../README.md#L193)은 `camera_center`가 PTP action sender라고
기술하지만, 실제 [`multicam_cameras.yaml`](../src/flir_spinnaker_camera/config/multicam_cameras.yaml)의
sender는 `camera_front_right`이고 `camera_center`라는 네임스페이스는 없다
(`camera0`가 192.168.1.1을 쓴다). 이 구성 전환에서 sender 개념 자체가 사라지므로
같이 정리한다.
