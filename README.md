# FMS — Fleet Management System

다중 AMR(자율주행 로봇) 군단의 작업 할당과 트래픽 제어를 다루는 FMS 개발 프로젝트.
대학 졸업과제로, 산업현장 로봇 군단을 조율하는 중앙 관제 시스템을 목표로 한다.

## 아키텍처

```
[robot_client/main.py --id AMR-001]  ─┐
[robot_client/main.py --id AMR-002]  ─┼─ MQTT ─> [mosquitto] <─┬─ [FMS 서버]
              ... N개                 ─┘                        └─ [뷰어]
```

로봇, FMS, 뷰어는 **완전히 독립된 프로세스**이며 MQTT로만 통신한다.

- **로봇 클라이언트** (`robot_client/`) — 프로세스 1개 = 로봇 1대. 자기 위치·속도·각도를
  스스로 계산해 `state` 로 보고하고, 목적지가 담긴 `order` 를 받으면 자기 맵으로
  장애물을 피하는 경로를 스스로 계획(A\*)해서 움직인다.
- **뷰어** (`tools/qt_viewer.py`, `tools/fleet_monitor_qt.py`) — 순수 관측자. 로봇을
  시뮬레이션하지 않고 MQTT로 받은 값만 그린다. 꺼도 로봇은 그대로 돈다.
- **FMS 서버** (`fms_server/`, 개발 중) — 로봇 상태를 모으고 오더(목적지)를 내려보낸다.

**FMS는 로봇이 시뮬레이터인지 실물인지 알 수 없다.** 로봇의 운동학·배터리
시뮬레이션은 `robot_client/sim/` 안에만 있고, FMS·뷰어 코드는 이걸 절대 import
하지 않는다. FMS가 아는 로봇 상태는 오직 로봇이 MQTT로 보고한 값뿐이다.

상세 설계 결정과 프로토콜 규칙은 [`CLAUDE.md`](./CLAUDE.md) 참고.

## 기술 스택

| 구분 | 기술 |
|---|---|
| FMS 백엔드 | Python + FastAPI |
| 로봇 ↔ FMS 통신 | MQTT (Mosquitto) |
| 스키마 검증 | Pydantic v2 |
| 라이브 뷰어 | PySide6 (Qt) + QGraphicsView |
| 로봇 로컬 제어판 | Tkinter |
| DB | MySQL (예정) |
| 프론트엔드 | React + Vite (예정) |
| 컨테이너 | Docker Compose (예정) |

## 시작하기

Python 3.11+ 필요. macOS/Ubuntu 둘 다 지원.

```bash
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt

# MQTT 브로커 (별도 설치 필요: macOS는 brew install mosquitto)
mosquitto -c broker/mosquitto.conf

# 샘플 맵 생성 (창고 레이아웃)
.venv/bin/python tools/make_sample_map.py
```

### 로봇 띄우기 (터미널을 나눠서 각각 실행)

```bash
.venv/bin/python -m robot_client.main --id AMR-001 --map maps/warehouse.json --x 1.2 --y 6.0

# --gui 를 붙이면 로컬 제어판(위치 확인/이동 명령/강제 로컬라이징)도 뜬다
.venv/bin/python -m robot_client.main --id AMR-002 --map maps/warehouse.json --x 1.2 --y 4.0 --gui
```

### 뷰어로 관측

```bash
.venv/bin/python tools/fleet_monitor_qt.py maps/warehouse.json
```

드래그로 이동, 스크롤로 확대/축소. `g`=격자, `t`=계획경로, `r`=보기 초기화.

### 오더 보내기 (FMS 서버가 아직 없어서 쓰는 임시 도구)

```bash
.venv/bin/python tools/send_order.py AMR-001 --path 13.4,2 --order-id O1
.venv/bin/python tools/send_order.py AMR-001 --instant EMERGENCY_STOP
```

로봇은 목적지 좌표만 받고, 거기까지 장애물을 피해 가는 경로는 로봇 자신의 맵과
몸체 크기(`--robot-width`/`--robot-length`)를 기준으로 스스로 A\* 로 계획한다.

### 실제 현장 맵 쓰기

`grid_cfg.grid` 형식(벤더 SLAM 맵 포맷)을 이미지 변환 없이 그대로 임포트할 수 있다.

```bash
.venv/bin/python tools/import_vendor_map.py maps/<맵폴더>
```

## 디렉터리 구조

```
common/          FMS·로봇 공유 스키마, 맵 모델 (좌표 변환)
robot_client/    로봇 클라이언트 (프로세스 1개 = 로봇 1대)
  sim/           운동학·배터리 시뮬레이션 — FMS/뷰어에서 import 금지
fms_server/      FMS 서버 (개발 중)
tools/           개발 도구 (뷰어, 맵 생성/임포트, 오더 발행 등)
broker/          MQTT 브로커 설정
maps/            occupancy grid 맵 (PNG + JSON 메타데이터)
```

## 현재 상태

로봇 클라이언트(상태머신, 경로계획, 충돌 감지), MQTT 통신 계층, 라이브 뷰어까지
동작한다. FMS 서버는 아직 골격 단계 — `tools/send_order.py` 가 임시로 그 역할을
대신한다. 진행 상황과 다음 작업 순서는 [`CLAUDE.md`](./CLAUDE.md) 의 "현재 진행
상황" 절 참고.
