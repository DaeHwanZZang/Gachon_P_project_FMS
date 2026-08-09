# CLAUDE.md


## 프로젝트

**다중 AMR 로봇의 작업 할당 및 트래픽 제어를 위한 FMS(Fleet Management System) 개발**
대학 졸업과제. 산업현장 자율주행 로봇 군단을 조율하는 중앙 관제 시스템.

---

## 확정된 아키텍처 결정

이 결정들은 이미 팀에서 합의된 것이다. 변경 제안 전에 반드시 이유를 물을 것.

### 1. 2-파트 구조

```
[FMS Server]  <--MQTT-->  [Robot Client] x N
```

### 2. 프로그램 경계 — 셋은 완전히 독립된 프로세스다

```
[robot_client/main.py --id AMR-001]  ─┐
[robot_client/main.py --id AMR-002]  ─┼─MQTT─> [mosquitto] <─┬─ [FMS 서버]
              ... N개                 ─┘                      └─ [뷰어/모니터]
```

- **로봇**은 자기 위치·속도·각도를 스스로 계산해 `state` 로 보고한다.
  FMS 도 뷰어도 이 프로세스 안을 들여다보지 않는다
- **뷰어**(`tools/fleet_monitor.py`)는 **순수 관측자**다. 로봇을 시뮬레이션하지 않고,
  아무것도 발행하지 않는다. 화면의 모든 값은 로봇이 보고한 것이다.
  뷰어를 꺼도 로봇은 그대로 돌고, 로봇이 없으면 빈 맵만 보인다
- **FMS**는 order 를 내려보내고 state 를 수집한다
- 한쪽 코드가 다른 쪽을 import 하면 설계 위반이다.
  뷰어 프로세스 안에서 `robot.x += v*dt` 가 돌면 그건 뷰어가 아니라 시뮬레이터다

### 3. 로봇 시뮬레이션의 위치 — 가장 중요한 원칙

> **FMS는 로봇이 시뮬레이터인지 실물인지 알 수 없어야 한다.**

- 로봇 물리 시뮬레이션(운동학, 배터리 모델)은 **로봇 클라이언트 내부**에만 존재한다
- FMS 코드베이스는 `robot_client/sim/` 을 **절대 import 하지 않는다**
- FMS가 아는 로봇 상태는 **로봇이 MQTT로 보고한 값뿐**이다
- FMS 안에 `robot.x += speed * dt` 같은 코드가 있으면 설계 위반이다

### 4. 프로세스 모델 — 방식 A

**로봇 1대 = 프로세스 1개.** 한 프로세스에 N대를 넣지 않는다.

```bash
python robot_client/main.py --id AMR-001
python robot_client/main.py --id AMR-002
```

- 관리는 `docker compose up --scale robot=20`
- 장애 주입은 `docker stop` 으로 프로세스를 죽이는 방식
- **결과**: 전역 시계 공유 불가 → 가속 시뮬레이션 대신 **실시간 모드로 시작**
  (실험이 오래 걸리면 나중에 FMS의 clock tick 브로드캐스트를 추가 검토)

### 5. 작업 흐름

FMS가 스케줄(Order)을 할당 → 로봇이 수신 → 로봇이 job 처리 → 상태 보고

### 6. OS

Ubuntu Linux. 개발/실행 환경 통일.

---

## 기술 스택

| 구분 | 기술 | 비고 |
|---|---|---|
| FMS 백엔드 | Python + FastAPI | AI 모듈 통합, 알고리즘 라이브러리 |
| 로봇-FMS 통신 | MQTT (Mosquitto) | pub/sub, LWT 활용 |
| 브라우저 통신 | REST + WebSocket | FastAPI 내장 |
| DB | MySQL | 팀 기존 경험 |
| 프론트엔드 | React + Vite, SVG 맵 렌더링 | 로봇 20대 이하는 SVG로 충분 |
| 스키마 검증 | Pydantic v2 | 서버·로봇 공유 계약 |
| 컨테이너 | Docker Compose | 로봇 다중 인스턴스 관리 |

---

## 디렉터리 구조 (목표)

```
project/
├── CLAUDE.md
├── docker-compose.yml
├── requirements.txt
├── broker/
│   └── mosquitto.conf      # ✅ 로컬 MQTT 브로커 설정
├── common/
│   ├── __init__.py
│   ├── schemas.py          # ✅ FMS/로봇 공유 메시지 계약
│   └── map_model.py        # ✅ occupancy grid 맵 + 좌표 변환
├── maps/                   # 맵 이미지(PNG) + 메타데이터(JSON)
│   └── 641931de9eae7cecb34d5765/  # 실제 AMR 벤더 맵 (Schaeffler 현장). 개발/테스트 기준 맵
├── tools/                  # 개발 도구 (FMS/로봇 런타임에 포함되지 않음)
│   ├── make_sample_map.py     # 샘플 창고 맵 생성
│   ├── import_vendor_map.py   # ✅ 벤더 grid_cfg.grid -> MapMetadata JSON (이미지 변환 없음)
│   ├── map_viewer.py          # ✅ 맵/로봇 렌더링, matplotlib (정적 확인용, 레거시)
│   ├── qt_viewer.py           # ✅ 맵/로봇 렌더링, PySide6/QGraphicsView (라이브 뷰어 본체)
│   ├── fleet_monitor.py       # ✅ MQTT 구독 -> 뷰어 (순수 관측자, 렌더러 비의존)
│   ├── fleet_monitor_qt.py    # ✅ 위 FleetMonitor + qt_viewer 조합 (라이브 뷰어 진입점)
│   └── send_order.py          # ✅ 오더 발행 (임시 FMS 대역)
├── fms_server/
│   ├── main.py             # FastAPI 진입점
│   ├── mqtt_client.py      # 로봇 통신
│   ├── fleet.py            # 로봇 상태 레지스트리
│   ├── allocator.py        # 작업 할당
│   ├── traffic.py          # 시공간 예약, 데드락 처리
│   ├── planner.py          # A* 경로계획
│   ├── battery.py          # 충전 스케줄링
│   └── api/                # REST + WebSocket 라우터
├── robot_client/
│   ├── main.py             # ✅ CLI 진입점 (--id). 프로세스 1개 = 로봇 1대
│   ├── comm.py             # ✅ MQTT 송수신, LWT
│   ├── executor.py         # ✅ 오더 해석, 상태머신, released 처리
│   ├── clock.py            # ✅ 시계 추상화 (time.sleep 직접 호출 대신)
│   └── sim/                # ⚠️ FMS/뷰어에서 절대 import 금지
│       ├── kinematics.py   # ✅ 사다리꼴 속도 프로파일
│       └── battery.py      # ✅ 선형 방전/충전
├── frontend/
└── experiments/
    └── scenarios/*.yaml
```

---

## 맵 좌표계

맵은 **흑백 occupancy grid PNG + 메타데이터 JSON** 한 쌍이다 (`maps/`).
상세는 `common/map_model.py`.

| 그레이 값 | 셀 | 주행 |
|---|---|---|
| 흰색 (>= `free_thresh`, 기본 192) | `FREE` | 가능 |
| 회색 (그 사이) | `UNKNOWN` 미탐사 | **불가** |
| 검은색 (<= `occupied_thresh`, 기본 64) | `OCCUPIED` 장애물 | 불가 |

**회색(미탐사)은 검은색과 동일하게 막힌 것으로 취급한다.** 가 본 적 없는 곳으로
로봇을 보내지 않는다. 렌더링에서는 셋을 구분해 보여준다.

- world 좌표 = 미터. `origin` 은 이미지 **좌상단** 픽셀의 world 좌표
- pixel 좌표 = 이미지 관례(좌상단 원점, y 아래로 증가). **world y 도 똑같이 아래로
  증가한다 — 뒤집지 않는다.** (ROS map_server 는 origin=좌하단 + y 뒤집기를 쓰지만,
  실제 AMR 벤더 맵 포맷(`grid_cfg.grid` 계열: `ox/oy` + `origin_px/py` + `scale_m2px`)
  이 좌상단·뒤집지 않는 좌표계를 쓰길래 여기 맞췄다. 그래야 벤더 맵을 이미지 변환
  없이 그대로 쓸 수 있다 — `tools/import_vendor_map.py` 참고)
- 변환은 반드시 `MapModel.world_to_pixel` / `pixel_to_world` 를 쓸 것.
  직접 계산하면 floor 처리에서 어긋난다
- `pixel_to_world` 는 픽셀 **중심** 좌표를 돌려준다 (왕복 시 최대 resolution/2 오차)
- 로봇의 `theta` 는 world 프레임 `atan2(dy, dx)` 그대로다. world 도 y-down 이라
  Qt(`tools/qt_viewer.py`)의 "양수=시계방향" 회전 규약과 부호가 그대로 맞는다 —
  뷰어에서 따로 부호를 뒤집지 않는다

---

## 통신 프로토콜

VDA5050 규격을 참고해 단순화. 상세는 `common/schemas.py` 참조.

### MQTT 토픽

```
fms/v1/{robot_id}/connection   Robot -> FMS   LWT, retain=True
fms/v1/{robot_id}/state        Robot -> FMS   200ms 주기
fms/v1/{robot_id}/order        FMS -> Robot   작업 지시서
fms/v1/{robot_id}/instant      FMS -> Robot   긴급정지, 장애주입
```

### 핵심 메커니즘: `released` 필드

트래픽 제어는 **정지 명령이 아니라 통행권 부여**로 구현한다.

- FMS는 안전이 확보된 노드만 `released: true` 로 내려보낸다
- 로봇은 `released: false` 노드 직전에서 스스로 멈추고 `WAITING` 상태가 된다
- 통행권이 생기면 FMS가 `order_update_id` 를 올려 같은 오더를 재발행
- **메시지가 유실돼도 로봇은 안전한 쪽(정지)에 머무른다**
- 규칙: `released` 는 노드 목록 앞에서부터 연속으로 true (스키마에서 검증)

### 상태머신 (로봇)

```
IDLE ─(order)→ MOVING ─(도착)→ ACTING ─(완료)→ MOVING ...
                 │                            └→ IDLE
                 └─(released:false)→ WAITING
BATTERY_LOW → CHARGING → IDLE
ERROR (어느 상태에서든 진입)
```

### 로봇이 오더를 거절하는 경우

FMS 를 만들 때 이 규칙을 알고 있어야 한다. 로봇은 다음을 조용히 무시한다.

| 조건 | 이유 |
|---|---|
| 같은 `order_id` 인데 `order_update_id` 가 크지 않음 | 통행권을 되돌리지 않기 위해 |
| 이미 완료한 오더의 재전송 | QoS 1 중복·FMS 재시작 시 재실행 방지 |
| 다른 오더 실행 중 (IDLE 아님) | 오더는 한 번에 하나 |
| 첫 노드가 현재 위치에서 `--max-start-deviation`(기본 1m) 초과 | 로봇은 첫 노드까지 **직선**으로 간다. 멀면 선반을 관통한다 |

**FMS 는 로봇이 서 있는 자리에서 시작하는 오더를 내려보내야 한다.**

### 로봇 자체 안전장치

경로계획은 FMS 몫이지만, 로봇은 자기 맵으로 스스로를 지킨다 (실물이면 범퍼/라이다).

- 주행 중 주행 불가 셀에 들어가면 **직전 위치에 멈추고** `COLLISION` FATAL 오류 → `ERROR`
  (장애물 안에 서면 어떤 오더를 받아도 첫 tick 에 또 충돌해 빠져나올 수 없다)
- **FATAL 이면 오더를 버린다.** 안 버리면 `RESET_ERROR` 직후 같은 경로로 또 달린다.
  재배정은 FMS 몫
- `--map` 을 안 주면 충돌 감지가 꺼진다

---

## 현재 진행 상황

### 완료
- [x] 아키텍처 확정
- [x] 기술 스택 선정
- [x] MQTT 토픽 구조 설계
- [x] `common/schemas.py` — Order / State / Connection / InstantAction
- [x] `common/map_model.py` — occupancy grid 맵 (PNG + JSON 메타데이터), world↔pixel 변환, 셀 분류
- [x] **로봇 클라이언트** — 프로세스 1개 = 로봇 1대, MQTT 접속/LWT, 오더 상태머신,
      운동학·배터리 시뮬레이션, 충돌 감지, 긴급정지·장애주입
- [x] **뷰어/모니터** — MQTT 구독으로 로봇 N대 표시 (순수 관측자)
- [x] MQTT 브로커 로컬 구성 (`broker/mosquitto.conf`)
- [x] `tools/send_order.py` — 임시 FMS 대역 (오더/즉시명령 발행)

### 다음 작업 (우선순위 순)
1. **맵 노드/엣지 그래프** — 오더 노드용 토폴로지 JSON, networkx 변환
   (occupancy grid 는 완료. 그 위에 주행 노드 그래프를 얹는 단계)
   지금은 오더 좌표를 손으로 찍고 있어서, 이게 없으면 A* 를 붙일 수 없다
2. **FMS 서버 골격** — 로봇 레지스트리, 상태 수집, order 발행
   (`tools/send_order.py` 가 하는 일을 제대로 하는 것)
3. docker-compose — 브로커 + 로봇 N대 (`--scale robot=20`)
4. A* 경로계획
5. 작업 할당 (비용함수 기반)
6. **트래픽 제어 — 시공간 예약** (프로젝트 핵심)
7. 대시보드 (React)
8. 배터리/충전 스케줄링
9. 실험 및 성능 측정

### 중요: 개발 순서
로봇 클라이언트가 돌아가므로 나머지 파트는 이제 병렬로 진행할 수 있다.
FMS 를 만드는 쪽은 로봇을 띄워놓고 실제 state 를 받아가며 개발하면 된다.

---

## AI 접목 계획 (미확정)

규칙 기반 구현을 **먼저 완성**하고 비교군(baseline)으로 삼는다.
"AI를 썼다"가 아니라 "휴리스틱 대비 N% 개선"이 나와야 한다.

| 후보 | 난이도 | 비고 |
|---|---|---|
| 혼잡도 예측 (LSTM/GNN) → A* 가중치 반영 | 중 | **1순위 후보** |
| 이상 탐지 (Autoencoder) | 하 | 서브 모듈로 적합 |
| RL 기반 작업 할당 (PPO/DQN) | 상 | 시간 여유 있을 때만 |
| 배터리 잔여시간 예측 | 하 | 부가 기능 |

---

## 실험 설계 원칙

- 시나리오는 코드가 아니라 **YAML 설정 파일**로 분리
- **`seed` 고정 필수** — 같은 시드, 다른 알고리즘으로 비교해야 공정
- 로봇 속도에 ±5% 노이즈 — 완벽 동기화되면 충돌 상황이 재현되지 않음
- 장애 주입(fault injection)으로 FMS 대응력 검증

### 측정 지표
- 시간당 처리량 (tasks/hour), 로봇 수 2/5/10/20 비교
- 트래픽 제어 적용 전후 충돌·교착 발생 횟수
- 작업 할당 전략 비교: greedy vs 비용함수 vs (AI)
- 평균 작업 완료 시간, 로봇 가동률

---

## 코딩 규칙

- Python 3.11+, 타입 힌트 사용
- Pydantic v2 문법 (`model_validate_json`, `model_dump_json`)
  — v1 문법(`parse_raw`, `.dict()`)은 사용하지 않는다
- 스키마 변경 시 `common/schemas.py` 만 수정하고 양쪽에 반영
- 로봇 코드에서 `time.sleep()` 직접 호출 지양 (시계 추상화 유지)
- 주석과 커밋 메시지는 한국어

---

## 실행

Python 3.11+ 필요. 프로젝트 전용 가상환경을 쓴다 (테스트 프레임워크/빌드/린터는 아직 없음).

```bash
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python tools/make_sample_map.py       # maps/warehouse.png + .json 생성
.venv/bin/python tools/import_vendor_map.py maps/641931de9eae7cecb34d5765   # 실제 벤더 맵 임포트
```

**터미널을 나눠서 각각 띄운다.** 프로세스가 곧 프로그램 경계다.

```bash
# 1) 브로커
mosquitto -c broker/mosquitto.conf          # macOS: brew install mosquitto

# 2) 로봇 — 1대당 터미널 1개. 10대면 10번 실행
.venv/bin/python -m robot_client.main --id AMR-001 --map maps/warehouse.json --x 1.2 --y 6.0
.venv/bin/python -m robot_client.main --id AMR-002 --map maps/warehouse.json --x 1.2 --y 4.0

# 3) 뷰어 (관측 전용) — Qt 버전이 기본. matplotlib 버전은 정적 확인용으로만 남겨둠
.venv/bin/python tools/fleet_monitor_qt.py maps/warehouse.json

# 4) 오더 발행 — FMS 가 생기기 전까지 쓰는 임시 도구
.venv/bin/python tools/send_order.py AMR-001 --path 1.2,6 1.2,2 13.4,2 --order-id O1
.venv/bin/python tools/send_order.py AMR-001 --instant EMERGENCY_STOP
```

기타:

```bash
.venv/bin/python example_usage.py                        # 스키마 데모
.venv/bin/python tools/map_viewer.py maps/warehouse.json # 맵만 보는 정적 뷰어
```

- 뷰어에서 **클릭하면 world 좌표가 터미널에 찍힌다.** 오더 노드 위치를 잡을 때 쓴다
- 한글 라벨이 깨지면 한글 폰트를 설치할 것 (Ubuntu: `sudo apt install fonts-nanum`)
- 장애 주입은 로봇 프로세스를 죽이면 된다 (`kill -9`). 브로커가 LWT 로
  `CONNECTION_BROKEN` 을 대신 발행한다

## Architecture

MQTT topics (`fms/v1/{robot_id}/...`), built via helpers in `schemas.py` (`topic_order`, `topic_state`, `topic_connection`, `topic_instant`):

- `connection` — Robot -> FMS, retained + LWT. Broker auto-publishes `CONNECTION_BROKEN` if the robot process dies uncleanly.
- `state` — Robot -> FMS, periodic (200ms). The `State` message is described as "FMS's only truth about the robot" — don't invent side channels for robot status.
- `order` — FMS -> Robot, the task/route instruction.
- `instant` — FMS -> Robot, out-of-band commands (e-stop, cancel, fault injection) independent of any order.

All messages inherit `FmsMessage` (version, monotonic `header_id`, `timestamp`, `robot_id`) and use `extra="forbid"` — typo'd fields raise `ValidationError` instead of silently passing.

Key invariants enforced by validators (in `Order`, `TimeWindow`):
- `OrderNode.sequence_id` must be strictly ascending, no duplicates.
- `OrderNode.released` gates traffic control: once `released=False` appears, every following node must also be `released=False` (release is contiguous from the front — a robot never gets permission for a node past a blocked one).
- `TimeWindow.exit` must not precede `enter`.
- `order_update_id` is a per-`order_id` monotonic counter; robots must reject updates not greater than what they already hold — this logic lives on the (not-yet-written) robot client, `schemas.py` only carries the field.

When extending the protocol, add fields/enums to `schemas.py` rather than duplicating shapes in server/client code, and keep both sides' behavior consistent with the Korean docstrings already documenting intent (state machine in `RobotState`, action lifecycle in `ActionStatus`, error severity in `ErrorLevel`).
