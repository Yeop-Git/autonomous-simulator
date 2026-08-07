# 중앙 집중형 V2X 자율주행 시뮬레이터

**완전 V2X 환경**을 가정한 다차량 자율주행 시뮬레이터다. 중앙 서버가 모든 차량·보행자·
장애물·긴급차의 위치와 속도를 오차 없이 안다고 두고, 그 위에서 **전역 상황 인지, 경로
계획, 충돌 회피, 협조적 기동**을 구현하고 정량 평가한다. 카메라·LiDAR 인지는 의도적으로
범위 밖이다 — 인지가 완벽할 때 남는 문제, 즉 *누가 언제 어디로 가야 하는가* 만 다루기
위해서다.

Unity는 그리며 움직이고, Python 서버는 결정한다.

![IntegratedCity 촬영 예정 자리표시자](docs/images/hero.png)

| | |
|---|---|
| **구현 범위** | 씬 6종(허브 1 + 주행 5), 서버 모듈 18개, 플래너 3종, 제어기 4종 |
| **통신** | WebSocket `localhost:8765`, JSON Schema Draft-07 양방향 검증, 명목 25 Hz |
| **검증** | 자동 테스트 259건 (257 pass / 2 skip, 2026-08-07 재검증, [§12](#12-검증-체계)) |
| **정량 결과** | 플래너 비교 · LKA 제어기 비교 · 씬별 부하 계측 ([§11](#11-실험-결과)) |
| **서버 계산 부하** | 헤드리스 `step()` p50 0.06–0.74 ms, 최악 p95 1.40 ms — 네트워크·Unity 적용 시간 제외 |

---

## 목차

**I. 설계**
[1. 문제 설정](#1-문제-설정과-범위) ·
[2. 시스템 구성](#2-시스템-구성) ·
[3. 인터페이스 명세](#3-인터페이스-명세) ·
[4. 월드 모델](#4-월드-모델)

**II. 알고리즘과 파라미터**
[5. 경로 계획](#5-경로-계획) ·
[6. 충돌 예측](#6-충돌-예측) ·
[7. 행동 결정](#7-행동-결정) ·
[8. 제어](#8-제어) ·
[9. 파라미터 일람](#9-파라미터-일람)

**III. 시나리오와 결과**
[10. 시나리오 명세](#10-시나리오-명세) ·
[11. 실험 결과](#11-실험-결과) ·
[12. 검증 체계](#12-검증-체계) ·
[13. 한계와 향후 과제](#13-한계와-향후-과제)

**IV. 부록**
[14. 실행 방법](#14-실행-방법) ·
[15. 저장소 구성](#15-저장소-구성) ·
[16. 참고문헌](#16-참고문헌)

**표기.** 좌표는 Unity 월드 공간, 단위는 m. `heading`은 yaw(도), 0 = +Z, 시계 방향
증가. 속도 m/s, 가속도 m/s², 시간 s. 본문의 `>` 인용 줄은 해당 절의 구현 파일이다.

---

# I. 설계

## 1. 문제 설정과 범위

### 1.1 가정

| 가정 | 근거 |
|---|---|
| 중앙 서버가 전 객체의 상태를 오차 없이, 지연 없이 안다 | V2X 이상 조건. 인지 오차를 제거해 **계획·조정 문제만** 분리 |
| 차량은 서버의 속도·경로 명령을 그대로 추종한다 | 차량 동역학은 Unity가 담당, 서버는 정책 계층 |
| 매 전송·제어 tick(명목 25 Hz)마다 전체 스냅샷이 도착한다 | 부분 관측·통신 두절은 별도 연구 주제 |

관측 잡음이 필요한 실험을 위해 `server/noise.py`에 잡음 주입기를 두었으나,
기본 파이프라인에서는 비활성이다.

### 1.2 범위 밖

카메라·LiDAR 인지, 센서 융합, 차량 동역학(타이어·서스펜션), 실차 이식.

### 1.3 연구 질문

1. 중앙 관제가 합류 갭 생성에 제공하는 협조 기동은 무엇인가 (→ [§7.2 합류 예약](#72-합류-시간-슬롯-예약))
2. 구조화된 도로망과 비정형 장애물에서 각각 어떤 탐색 알고리즘이 적합한가 (→ [§11.1](#111-실험-1--경로-탐색-알고리즘-비교))
3. 횡방향 제어 법칙은 속도·곡률에 따라 어떻게 열화하는가 (→ [§11.2](#112-실험-2--lka-횡방향-제어기-비교))
4. 서버 계산 커널은 현재 씬에서 명목 송신 간격 안에 드는가 (→ [§11.3](#113-실험-3--씬별-제어-루프-부하))

---

## 2. 시스템 구성

### 2.1 계층

```
┌─ Unity (C#) ──────────────┐        ┌─ Python 중앙 서버 ─────────────────────┐
│  씬 · 도로 형상 · 차량 물리 │  state │  world_model    전역 상황 인지          │
│  VehicleController        │───────▶│  collision_predictor  위험 예측         │
│  LKAController            │  25 Hz │  planners/      A* · RRT · RRT*        │
│  V2XClient                │◀───────│  merge · traffic · left_turn · behavior │
│  시나리오 디렉터 · UI       │command │  controllers/   ACC · lateral          │
└───────────────────────────┘        └────────────────────────────────────────┘
```

한 틱의 처리 순서 (`CentralController.step`):

```
1. 잡음 주입(선택)              5. 합류 예약 갱신
2. tick 역행 검사 → 상태 폐기     6. 전역 충돌 예측 (전 이동체 쌍)
3. 월드 스냅샷 갱신             ── 이하 차량별 ──
4. 정차 객체 판정               7. 선행차 탐색 → ACC 목표 속도
                              8. 정책 결정 (신호·좌회전·회피·차선변경)
                              9. 행동 FSM 라벨 확정 → 정책 계층의 라벨 덮어쓰기
                             10. CommandMessage 생성 · 스키마 검증
```

### 2.2 역할 분담

| | Unity | Python |
|---|---|---|
| 도로·차선 형상 | **소유** (사람이 편집) | export된 JSON을 읽기만 함 |
| 차량 운동 | **소유** (물리·조향) | 목표 속도·경로만 지시 |
| 경로 계획 | — | **소유** |
| 충돌 예측·행동 결정 | — | **소유** |
| 신호 표시 | 그림 | **집행** (동일 계획을 양쪽이 보유, [§12](#12-검증-체계)에서 대조) |
| 횡방향 제어 | 라이브 데모(C#) | 헤드리스 실험(Python), 동일 법칙 |

> `unity/Assets/Scripts/`, `server/central_control.py`

---

## 3. 인터페이스 명세

메시지는 두 종류뿐이며 **JSON Schema가 유일한 진실**이다. 서버는 수신 `StateMessage`와
송신 `CommandMessage`를 **양쪽 모두** Draft-07로 검증한다. 검증기는 기동 시 한 번만
컴파일한다 — 매 스냅샷마다 컴파일하면 WebSocket 루프가 Unity보다 수 틱 뒤처진다.

> `shared/protocol/`, `server/main.py`

### 3.1 StateMessage (Unity → Python)

| 필드 | 타입 | 필수 | 의미 |
|---|---|:---:|---|
| `time` | number | ● | 씬 시작 이후 시뮬레이션 시각 (s) |
| `tick` | integer | | 단조 증가 프레임 카운터 |
| `scenario` | enum | | `highway` \| `urban` \| `lka_test` \| `emergency_avoidance` \| `integrated_city` |
| `planner_mode` | enum | | `rrt` \| `rrt_star` — 회피 씬에서 실행 중 전환 |
| `vehicles[]` | object | ● | 아래 |
| `objects[]` | object | ● | `pedestrian` \| `bicycle` \| `emergency_vehicle` \| `static_obstacle` \| `unexpected_obstacle` + `position` · `velocity` · `radius`(기본 0.4) |
| `events[]` | object | ● | `PedestrianSuddenCrossing` \| `VehicleBreakdown` \| `FallingObject` \| `EmergencyVehicle` \| `ConstructionZone` |

`vehicles[]` 항목:

| 필드 | 필수 | 비고 |
|---|:---:|---|
| `id` · `position` · `velocity` · `heading` · `current_lane` | ● | `position`/`velocity`는 `[x, y, z]` 고정 길이 3 |
| `acceleration` · `target_lane` | | `target_lane`은 `null` 허용 |
| `maneuver` | | `straight` \| `left` \| `right` |
| `has_goal` · `goal` | | Unity의 `JsonUtility`가 고정 형태만 낼 수 있어, 목표 유무를 **별도 불리언**으로 표현 |
| `behavior_state` | | 18종 enum (Unity가 보고하는 현재 상태) |

### 3.2 CommandMessage (Python → Unity)

| 필드 | 필수 | 의미 |
|---|:---:|---|
| `time` | ● | 응답 대상 state의 `time`을 그대로 되돌려 지연을 측정 |
| `tick` | | 응답 대상 state의 `tick` |
| `commands[].vehicle_id` | ● | |
| `commands[].target_speed` | ● | m/s, `0` = 정지 |
| `commands[].behavior` | ● | 15종 enum (`LaneKeeping` … `ControlledStopping`) |
| `commands[].target_lane` | | 차선 변경 목표 (`null` 허용) |
| `commands[].path[]` | | 추종할 웨이포인트 열 (월드 좌표) |
| `commands[].left_turn_phase` | | 좌회전 9단계 중 현재 단계 |
| `commands[].turn_signal` | | `none` \| `left` \| `right` \| `hazard` |
| `commands[].planner` | | `astar` \| `rrt` \| `rrt_star` — 이 경로를 만든 플래너 |
| `commands[].plan_status` · `planning_time_ms` · `minimum_clearance` | | 회피 계획 계측값 |
| `commands[].lka_enabled` | | 기본 `true` |

### 3.3 동기 규약 — 본 프로젝트의 1순위 리스크

양측이 조용히 어긋나는 것이 최대 위험이므로, 검출을 **명시적**으로 만든다.

| 상황 | 서버의 대응 |
|---|---|
| 중복 tick | `WARNING duplicate tick` |
| 역순 tick | `WARNING out-of-order tick` |
| 건너뛴 tick (기대 stride 2 초과) | `WARNING gap: jumped a -> b` |
| **tick 역행** (Unity가 씬을 재로드해 시계 리셋) | 차량별 누적 상태를 **전부 폐기**. 그렇지 않으면 재시작 전의 회피 상태가 남아 오판을 만든다 |
| Unity 측 전송 겹침 | `_isSending` 직렬화, 명령이 N틱 이상 밀리면 경고 |
| 좌회전 단계 전이 · 녹색인데 대기 | 진단 로그 1회씩 출력 (`[LeftTurn]`, `[LeftTurnBlocked]`) |

신호 계획과 차선 그래프는 **양쪽에 사본이 존재**하므로 드리프트 검출 테스트를 따로 둔다
([§12](#12-검증-체계)).

---

## 4. 월드 모델

정적인 **차선 그래프**와 매 전송·제어 tick에 갱신되는 **동적 스냅샷**을 분리한다.

| | 차선 그래프 `LaneNetwork` | 동적 스냅샷 `WorldModel` |
|---|---|---|
| 출처 | Unity 씬 → `V2X > Export Lane Network` | 매 전송·제어 tick(명목 25 Hz) `StateMessage` |
| 수명 | 서버 실행 내내 불변 | 매 전송·제어 tick에 교체 |
| 내용 | 차선 id, 중심선 폴리라인, 폭, 제한속도, 좌/우 인접, 후속 차선 | 차량·객체의 위치·속도·차선·행동 |

차선 하나는 중심선 폴리라인과 위상 정보로 표현되며, 기본 폭 3.5 m, 기본 제한속도
13.9 m/s(≈50 km/h)다. 서버는 이 그래프 위에서만 계획하므로 **도로 형상의 진실은
언제나 Unity 씬**이고, 씬을 고친 뒤 export를 잊으면 서버는 존재하지 않는 도로 위에서
계획하게 된다 — 그래서 [§12](#12-검증-체계)의 대조 테스트가 필요하다.

> `server/world_model.py`, `shared/protocol/lane_network.schema.json`

---

# II. 알고리즘과 파라미터

## 5. 경로 계획

모든 플래너는 동일 인터페이스를 구현하여 실험 러너가 교체할 수 있다.

```python
plan(start: Vec3, goal: Vec3, world: World) -> list[Vec3]    # 빈 리스트 = 경로 없음
```

`World`는 `neighbors` · `lane_centerline` · `nearest_lane` · `all_lane_ids` ·
`is_blocked` 다섯 개만 요구하는 프로토콜이라, 플래너는 Unity 없이 단위 테스트된다.

> `server/planners/base.py`

### 5.1 전역 계획 — A\*

차선 그래프 위의 A\*. 간선 비용은 차선 길이, 휴리스틱은 차선 끝에서 목표까지의 직선거리
(허용 가능 heuristic). 어려운 부분은 탐색이 아니라 **찾은 차선 열을 하나의 웨이포인트
경로로 잇는 접합 처리**다.

| 문제 | 해법 |
|---|---|
| 온램프가 본선 **중간**(arc 115 m)으로 접합할 때, 후속 차선을 시작점부터 이으면 경로가 뒤로 튄다 | 후속 차선을 **직전 차선 끝의 투영 지점**부터 이어 붙임 |
| 교차로 정지선에서 직진·좌회전 커넥터가 **같은 점에서 출발**해, 최근접 차선이 동률이면 잘못 고른 뒤 "경로 없음"으로 정지 | 1.0 m 이내를 동률로 보고 최대 4개 후보를 **순서대로 재시도** |

> `server/planners/astar.py`

### 5.2 국부 계획 — RRT / RRT\*

전역 경로는 A\*가 유지하되, 돌발 장애물 앞에서는 샘플링 플래너가 **일시적으로 경로를
넘겨받는다**. 자유공간 전체가 아니라 **차선 코리도로 제한**된 공간을 탐색한다.

- **코리도** = 현재·목표·원래 차선의 좌우 인접 그룹과 그 후속 차선들의 합집합.
  차선 *이름*이 아니라 위상으로 정하므로, 도심 격자와 간선처럼 명명 규칙이 다른
  두 도로 계열을 가로질러도 성립한다.
- **점유 공간** = 타 차량의 3 s 예측(0.5 s 간격)을 반경 **2.45 m**(차량 여유 1.45 m
  + 1.0 m)로, 장애물은 **자기 반경 + 1.45 m**로 부풀린 것.
- **RRT\***는 첫 연결에서 멈추지 않고 예산을 모두 써서 `choose-parent` + `rewire`로
  경로를 개선한다 (운용 재배선 반경 8 m).

경로 후처리는 세 단계다:

| 단계 | 내용 |
|---|---|
| 단축 (shortcut) | 0.5 m 해상도로 충돌을 재검사하며 중간 정점을 탐욕적으로 제거 |
| 재샘플링 | 2 m 등간격으로 재배치 (Unity 추종용) |
| 검증 | 인접 구간 최대 꺾임각이 **78°를 넘으면 경로 폐기** — 조향 불가능한 해를 내보내지 않는다 |

**주의 — 운용 설정과 클래스 기본값은 다르다.** `RRTConfig` 데이터클래스의 기본값
(step 4.0 m, 목표 샘플 0.1, 2000 반복, 목표 반경 4.0 m, 간선 해상도 1.0 m, 여유 30 m,
재배선 12 m)은 단위 테스트용이며, 실제 씬을 도는 값은 `local_avoidance.py`가 매번
덮어쓴다. 아래가 운용 값이다.

| | step | 목표 샘플 | 최대 반복 | 목표 반경 | 간선 해상도 | 경계 여유 | 재배선 | 시간 예산 |
|---|---|---|---|---|---|---|---|---|
| RRT | 3.0 m | 0.22 | 1800 | 3.5 m | 0.5 m | 12 m | — | **45 ms** |
| RRT\* | 3.0 m | 0.22 | 1200 | 3.5 m | 0.5 m | 12 m | 8 m | **140 ms** |

시드는 `(tick + 차량 id 해시) % 100000`이라 재현 가능하다. 실제 계획 시간과 최소
여유거리는 명령(`planning_time_ms`, `minimum_clearance`)에 실어 보고한다.

> `server/planners/rrt.py` · `rrt_star.py` · `_rrt_common.py` · `avoidance_world.py`,
> `server/path_postprocess.py`

---

## 6. 충돌 예측

샘플링이 아니라 **해석적으로** 푼다. 두 물체의 상대 위치·속도로 이차방정식을 세워
안전거리를 처음 침범하는 시각을 직접 구하므로, 빠르게 횡단하는 차량이 샘플 사이로
빠져나가는 **터널링이 원리적으로 없다**.

| 항목 | 값 |
|---|---|
| 예측 지평선 | 4.0 s |
| 안전거리 | 2.5 m (중심 간) |
| 산출값 | 최근접 접근 시각·거리, 안전거리 침범까지의 시간(TTC) |

예측기 자체는 **원시 충돌만** 낸다. 교착을 막는 두 필터는 그 위 제어층에 있다.

1. `_breach_is_standing` — **이미 안전거리 안이면서 서로 접근 중이 아닌** 위반을 제외.
   정지한 두 차량이 서로를 영원히 급제동시키는 상태를 방지한다.
2. `_traffic_conflict_is_managed` — **신호로 관리되는 충돌**을 제외. 녹색 접근로가
   적색 접근로 때문에 급제동하지 않는다.

> `server/collision_predictor.py` (원시 충돌 검출),
> `server/central_control.py` (제어층 두 필터)

---

## 7. 행동 결정

### 7.1 행동 FSM

`next_behavior`는 우선순위 기반 **순수 함수**다. 상태를 기억하지 않고 매 틱 전체
상황으로 다시 판단하므로, 전이 표가 꼬여 빠져나오지 못하는 상태가 존재할 수 없다.

| 순위 | 조건 | 라벨 |
|---:|---|---|
| 1 | 최소 TTC ≤ 1.5 s (선행차 추돌 임박 포함) | `EmergencyBraking` |
| 2 | 목표 보유 + 도착 | `Arrived` |
| 3 | 목표 보유 + **경로 없음** | `Stopping` |
| 4 | 최소 TTC ≤ 3.0 s (선행차 외 충돌) | `Stopping` |
| 5 | 선행차 범퍼 간격 ≤ 30 m | `Following` |
| 6 | 그 외 | `LaneKeeping` |

**`WaitingAtIntersection`은 이 순수 FSM의 산출이 아니다.** 신호·좌회전·회피 정책이
`central_control.py`에서 라벨을 **사후에 덮어쓰며**, 신호 대기 덮어쓰기는 `Arrived`만
보호하므로 `EmergencyBraking`도 덮일 수 있다. 즉 위 표는 FSM 내부 우선순위이지 최종
명령 라벨의 우선순위가 아니다.

틱을 넘어 유지되는 약속은 의도적으로 최소화했다: 이미 시작한 횡방향 이동, 늦은 취소,
회피 기동의 단계 — 이 셋뿐이다.

> `server/behavior.py` (순수 FSM), `server/central_control.py` (라벨 덮어쓰기)

### 7.2 합류 시간 슬롯 예약

**중앙 관제의 대표 기술.** 램프 차량이 스스로 눈치 보는 대신, 서버가 본선 전 차량의
합류점 도착 시각(ETA)을 계산해 슬롯을 예약한다.

```
1. 본선 차량 ETA 정렬 (이미 합류점을 지난 차량 제외)
2. 램프 ETA가 이미 2.0 s 이상 갭 안이면 → 그대로 통과
3. 아니면 가장 가까운 충분한 갭으로 램프 차량을 재타이밍 (속도 지시)
4. 도달 가능한 갭이 없으면 → 본선 후행 차량에 감속을 지시해 갭을 연다
                              ← 램프 차량 혼자서는 절대 할 수 없는 수
```

합류 차선 여부는 **이름이 아니라 위상과 기하로** 판정한다.

| 유형 | 판정 |
|---|---|
| 온램프 | 후속 차선의 **중간**(끝-시작 접합에서 1.0 m 초과 이격)으로 접합 |
| 갓길 테이퍼 | 공통 접합점 15 m 전에서 8 m 이내로 나란히(헤딩 차 30° 이내) 달리는 더 느린 차선 |

관측 범위는 접합점 상류 250 m, 램프 차량이 5 m/s 미만이면 합류가 아니라 서행으로 본다.

> `server/merge.py`, `server/central_control.py`

### 7.3 신호 교차로

고정 주기 60 s 신호를 **서버가 집행**하고 Unity가 그린다. 동일 계획이 양쪽에 있으므로
드리프트 검출 테스트로 묶여 있다.

| 구간 (s) | 상태 |
|---|---|
| 0 – 10 | 동서 직진 녹색 |
| 10 – 13 | 동서 황색 |
| **13 – 21** | **보행 페이즈** (전 방향 적색) |
| 21 – 31 | 남북 직진 녹색 |
| 31 – 34 | 남북 황색 |
| 34 – 36 | 전 방향 적색 (버퍼) |
| **36 – 42** | **북측 보호 좌회전 화살표** |
| 42 – 44 | 좌회전 황색 |
| 44 – 47 | 전 방향 적색 (버퍼) |
| **47 – 55** | **보행 페이즈** |
| 55 – 60 | 전 방향 적색 (버퍼) |

각 신호등은 `green_time` / `yellow_time` / `red_time` / `offset`으로 정의되며, 위
구간은 여덟 접근로의 정의값에서 계산된 것이다. 보행 페이즈만 `PEDESTRIAN_PHASES`에
직접 상수로 박혀 있다. 같은 매니저에 합성 테스트용 레거시 신호등(`urban_north`,
`urban_east`, 주기 46 s)도 등록되어 있으나, 출하 씬의 차선 id와는 대응되지 않아
실제 주행에는 관여하지 않는다.

- 정지는 정지선 **5.5 m 앞**에서 **1.8 m/s²**의 편안한 감속으로 미리 시작한다.
- 황색은 "편하게 설 수 있으면 선다" — 무조건 정지가 아니다.
- 접근 판정 범위는 정지선 상류 55 m.
- 한국식 우회전: 적신호에서 **완전 정지 후** 보행자·교차 교통에 양보하며 진행.

> `server/traffic.py`, `server/central_control.py`,
> `unity/Assets/Scripts/UI/TrafficLightSystem.cs`

### 7.4 보호 좌회전 정책

전이 기반 FSM이 아니라 **매 틱 전체 스냅샷을 다시 읽는 우선순위 정책**이다.

```
원차선 ──(갭 수용)──▶ 좌회전 차선 ──▶ 정지선 ──▶ 화살표 대기 ──▶ 교차 ──▶ 출구 정렬
       ↘ (마감까지 갭 없음) ────────▶ 직진으로 안전 취소 (AbortedStraight)
```

| 조건 | 값 |
|---|---|
| 차선 변경 갭 수용 시간 간격 | 1.25 s |
| 차선 변경 완료 목표 지점 | 정지선 14 m 전 |
| 차선 변경 경로 최소 길이 / 정지 여유 | 12 m / 2 m |
| 교차 충돌 확인 범위 | 35 m |
| 보행자 침범 판정 | 주행 통로 반폭 3 m |

녹색이어도 앞차가 서 있거나, 보행자가 **실제 주행 통로 안에** 있거나, 출구가 막혔으면
대기한다. 보행자는 원형 반경이 아니라 **회전 시 실제로 지나갈 경로에 투영**해 판정하므로,
인도 위의 사람 때문에 정지하지 않는다.

> `server/left_turn.py`, `server/central_control.py`,
> [`docs/left_turn_behavior.md`](docs/left_turn_behavior.md)

### 7.5 국부 회피 상태기

```
HazardDetected ─▶ EscapePlanning ─▶ LateralEvading ─▶ (긴급차면 Yielding)
                                          │
                                          ▼
                              RejoinPlanning ─▶ LaneRejoining ─▶ 원차선 복귀
      계획 실패 ─▶ ControlledStopping (0.75 s 주기로 재시도)
```

| 상황 | 대응 |
|---|---|
| 탈출 경로 끝에 도달했는데 장애물이 아직 옆에 있음 | 경로를 **연장** |
| 복귀가 막힘 | 정지하지 않고 **회피 차선에서 계속 주행하며 1.5 s 뒤 재시도** |
| 어느 단계에서든 경로가 빔 | 목표 속도 0 — **경로 없이 달리는 상태를 만들지 않는다** |
| 긴급차 접근 (반경 60 m) | 최우측 갓길로 대피, 3 m/s로 서행하며 통과 확인 후 복귀 |

> `server/local_avoidance.py`, `server/emergency.py`,
> [`docs/emergency_avoidance_plan.md`](docs/emergency_avoidance_plan.md)

---

## 8. 제어

### 8.1 선행차 탐색과 ACC

ACC가 따라갈 **선행차**를 찾는 일이 알고리즘의 실질이다. 세 경로로 찾는다.

1. **같은 차선** — 중심선 arc 비교
2. **하류 차선** — 후속 차선의 진입 arc를 빼서 거리 계산 (중간 합류 접합 대응)
3. **형제 차선** — 같은 접합점으로 들어오는 다른 차선. 먼저 도착하는 쪽이 선행차 (지퍼 병합)

도로를 막은 장애물은 정지한 차량과 정확히 같은 방식으로 선행차가 된다.

ACC는 **일정 시간 간격(constant time-gap) 정책**에 선형 피드백을 얹고 가감속 한계로
클램프한 형태다. 완전한 IDM 비선형성 없이도 갭 유지와 추돌 방지에 충분하다.

$$
v_{\text{safe}} = \sqrt{v_{\text{lead}}^2 + 2\,a_{\max}\,(g - g_0)},\qquad
v_{\text{cmd}} = \min\bigl(v_{\text{free}},\; v_{\text{safe}},\; v_{\text{lead}} + k_g (g - g_{\text{des}})\bigr)
$$

여기서 $g_{\text{des}} = g_0 + \tau v_{\text{ego}}$. 안전속도 식은 **양측이 동일하게
제동**한다는 가정에서 유도된 표준 안전거리 모델이며, 선행차가 우리보다 급하게 설 수
있는 경우(예: 벽 충돌)까지는 보호하지 않는다 — 그 경우는 $v_{\text{lead}}^2$ 항을
빼면 된다. 현실적인 교통류를 위해 등제동 형태를 유지한다.

| 기호 | 파라미터 | 값 |
|---|---|---|
| $\tau$ | 희망 차두 시간 | 2.0 s |
| $g_0$ | 정지 시 최소 범퍼 간격 | 6.0 m |
| $a_{\max}$ | 편안한 감속 | 4.0 m/s² |
| — | 급제동 (범퍼 간격이 $g_0$ 미만일 때) | 8.0 m/s² |
| — | 최대 가속 | 2.0 m/s² |
| $k_g$ | 갭 오차 이득 | 0.4 |

> `server/behavior.py`, `server/controllers/acc.py`

### 8.2 횡방향 제어

라이브 데모는 Unity의 C# `LKAController`가, 정량 실험은 동일 법칙의 Python 구현이
헤드리스로 돌린다. 세 구현 모두 같은 시그니처라 실험 러너가 교체한다.

```python
steer(x, z, heading_deg, speed, centerline, wheel_base) -> steering_rad
```

| 제어기 | 법칙 | 기본 이득 |
|---|---|---|
| Pure Pursuit | 전방주시점 기반 기하 추종, $\delta = \arctan\dfrac{2L\sin\alpha}{\ell_d}$ | $\ell_d = 4.0 + 0.4v$, 최대 조향 0.6 rad |
| Stanley | 전륜 기준 헤딩오차 + 횡오차, $\delta = \theta_e + \arctan\dfrac{k e}{k_s + v}$ | $k = 1.5$, $k_s = 1.0$ |
| PID | 횡오차 되먹임 | $k_p = 0.12$, $k_i = 0$, $k_d = 0.4$ |

공통으로 **Frenet 오차**를 계산해 로그로 남긴다: 부호 있는 횡오차(양수 = 경로 진행
방향 기준 좌측), 헤딩 오차(도), Menger 곡률(1/m).

**이득 정정은 사람의 일이다.** 위 값은 미조정 기본값이며, 이 저장소가 제공하는 것은
비교 그래프이지 최적 이득이 아니다.

> `server/controllers/lateral.py`, `unity/Assets/Scripts/Vehicle/LKAController.cs`

### 8.3 로깅 스키마 (고정)

분석 노트북이 깨지지 않도록 CSV 컬럼을 **동결**했다.

```
time, vehicle_id, scenario, position_x, position_z, speed, lane_id,
behavior_state, lateral_error, heading_error, target_speed,
collision_risk, ttc, event_type
```

지표: 평균 속도, 최소 TTC, 최대·RMS 횡오차, 차선 이탈 횟수, 급제동 횟수(감속
4 m/s² 초과), 도착 수, 위험 이벤트 수.

> `server/logging_csv.py`, `server/metrics.py`

---

## 9. 파라미터 일람

실제 주행 경로에서 쓰이는 값. 괄호 안은 정의 위치이며, 클래스 기본값이 호출부에서
덮어써지는 경우는 **운용 값**을 적었다.

<details>
<summary><b>펼쳐 보기</b></summary>

| 계층 | 파라미터 | 값 |
|---|---|---|
| **통신** | 포트 / 기대 tick stride | 8765 / 2 (`main.py`) |
| **월드** | 기본 차선 폭 / 기본 제한속도 | 3.5 m / 13.9 m/s (`world_model.py`) |
| **A\*** | 동률 판정 밴드 / 최대 후보 차선 | 1.0 m / 4 (`planners/astar.py`) |
| **RRT (운용)** | step / 목표 샘플 확률 / 최대 반복 / 목표 반경 | 3.0 m / 0.22 / 1800 / 3.5 m (`local_avoidance.py`) |
| | 간선 검사 해상도 / 경계 여유 | 0.5 m / 12 m |
| **RRT\* (운용)** | 최대 반복 / 재배선 반경 | 1200 / 8.0 m (`local_avoidance.py`) |
| | *(참고)* 클래스 기본값 | 4.0 m / 0.1 / 2000 / 4.0 m / 1.0 m / 30 m / 재배선 12 m (`_rrt_common.py`, `rrt_star.py`) |
| **회피 공간** | 차량 점유 반경 / 장애물 가산 / 예측 지평 / 간격 | 2.45 m / +1.45 m / 3.0 s / 0.5 s (`avoidance_world.py`) |
| **후처리** | 단축 해상도 / 재샘플 간격 / 최대 꺾임각 | 0.5 m / 2.0 m / 78° (`path_postprocess.py`) |
| **회피 예산** | RRT / RRT\* | 45 ms / 140 ms (`local_avoidance.py`) |
| **회피 재시도** | 복귀 차단 후 대기 / 통제 정지 재계획 주기 | 1.5 s / 0.75 s (`local_avoidance.py`) |
| **충돌 예측** | 지평선 / 스텝 / 안전거리 | 4.0 s / 0.2 s / 2.5 m (`collision_predictor.py`) |
| **ACC** | 차두시간 / 정지간격 / 가속 / 감속 / 급제동 / 갭이득 | 2.0 s / 6.0 m / 2.0 / 4.0 / 8.0 m/s² / 0.4 (`controllers/acc.py`) |
| | 정지 판정 선행차 속도 / 정지 간격 허용오차 | 0.2 m/s / 0.5 m |
| **행동 FSM** | 긴급 TTC / 주의 TTC / 추종 진입 간격 | 1.5 s / 3.0 s / 30 m (`behavior.py`) |
| **차선 변경** | 수용 시간 간격 / 정지 간격 / 차량 길이 | 1.5 s / 6.0 m / 4.5 m (`lane_change.py`) |
| **합류** | 필요 차두 / 최소 합류 속도 / 상류 관측 / 접합 판정 | 2.0 s / 5.0 m/s / 250 m / 1.0 m (`merge.py`, `central_control.py`) |
| | 평행 판정 (탐침 / 간격 / 각도) | 15 m / 8 m / 30° |
| **신호** | 주기 / 정지선 여유 / 접근 감속 / 접근 범위 | 60 s / 5.5 m / 1.8 m/s² / 55 m |
| **좌회전** | 갭 수용 / 준비 거리 / 충돌 범위 / 보행 통로 | 1.25 s / 14 m / 35 m / 3.0 m (`central_control.py`) |
| **긴급차** | 양보 반경 / 양보 속도 | 60 m / 3.0 m/s (`emergency.py`) |
| **주행 판정** | 도착 반경 / 경로 이탈 / 목표 이동 / 정지 판정 | 3.0 m / 3.0 m / 1.0 m / 0.3 m/s (`central_control.py`) |
| **지표** | 급제동 임계 | 4.0 m/s² (`metrics.py`) |

</details>

---

# III. 시나리오와 결과

## 10. 시나리오 명세

Unity 실행 시 **Main 허브**가 먼저 뜨고, 거기서 씬을 선택해 진입한다. 각 씬에서는
`Esc` 또는 우측 상단 버튼으로 허브에 복귀한다.

> 이 절의 씬 이미지는 캡처 구도와 파일명을 고정하기 위한 **자리표시자**다. 알고리즘
> 차트는 실제 CSV 산출물이지만, 씬 이미지는 실험 증거로 사용하지 않는다. 실제 캡처는
> [`docs/images/README.md`](docs/images/README.md)의 동일 파일명으로 교체한다.

### 10.0 Main — 허브

![Main 허브](docs/images/scene-main-hub.png)

주행 씬이 아니라 **메뉴**다. 씬을 고르면 목적·기술 명세·조작법이 우측 패널에 표시되고,
`Enter` 또는 실행 버튼으로 진입한다. 선택과 실행을 두 단계로 나눈 것은 의도적이다 —
설명이 놓일 자리가 생기고, 잘못 클릭해도 메뉴를 벗어나지 않는다.

- 숫자키 `1`~`5` 선택 · `Enter` 실행
- Build Settings에 없는 씬을 고르면 멈추는 대신 **무엇이 빠졌는지 화면에 알린다**
- 도로가 없는 씬이라 차선 export도 없다

허브와 다섯 주행 씬은 1920×1080 기준의 같은 UI 체계를 쓴다. 배경은
`#F5F5F7`, 패널은 흰색, 본문은 `#1D1D1F`, 조작 강조색은 `#0066CC`로
제한했다. 조작 버튼은 화면 하단, 허브 복귀는 우측 상단에 고정한다.
한글과 영문을 함께 읽는 화면이므로 본문 16 px, 상태 18 px, 패널 제목
24 px를 최소 크기로 삼았다. 폰트는 Pretendard 1.3.8 Regular/SemiBold를 포함하며,
재배포 조건은 [`unity/Assets/Fonts/Pretendard-LICENSE.txt`](unity/Assets/Fonts/Pretendard-LICENSE.txt)에
보존했다.

---

### 10.1 LKA_Test — 차선 유지 시험로

![LKA_Test](docs/images/scene-lka-test.png)

**형상** 반경 90 m, ±55° 구간의 단일 곡선 트랙(폭 7 m), 제한속도 27.8 m/s.
차량 1대, 다른 교통도 이벤트도 없다.

**목적** 다른 요인을 모두 제거하고 **횡방향 제어기만 홀로** 남겨 성능을 읽는다.

| 검증 대상 | 절 |
|---|---|
| Pure Pursuit / Stanley 횡방향 제어 (씬 기본값 Stanley) | [8.2](#82-횡방향-제어) |
| Frenet 오차(횡·헤딩·곡률) 계산 | [8.2](#82-횡방향-제어) |
| ACC 자유주행 구간 | [8.1](#81-선행차-탐색과-acc) |
| 고정 CSV 로깅 → 속도별 RMS 오차 | [8.3](#83-로깅-스키마-고정) |

**조작** `1` `2` `3` 카메라 전환 · `Esc` 허브

---

### 10.2 Highway — 고속도로 합류와 차선 변경

![Highway 전경](docs/images/scene-highway.png)

**형상** 3차선 본선(길이 300 m, 차선 간격 3.5 m, 제한속도 27.8 m/s) + 온램프
(제한속도 18 m/s). 램프는 본선 **중간 지점 z = 115 m**로 접합한다 — 단순한 끝-시작
접합보다 훨씬 까다로운 구성이며, [§5.1](#51-전역-계획--a)의 접합 처리와
[§8.1](#81-선행차-탐색과-acc)의 하류 차선 탐색이 여기서 필요해진다.

![합류 순간](docs/images/scene-highway-merge.png)

| 검증 대상 | 절 |
|---|---|
| V2X 합류 시간 슬롯 예약 (재타이밍 → 본선 양보) | [7.2](#72-합류-시간-슬롯-예약) |
| 합류 차선 판정을 위상·기하로 도출 | [7.2](#72-합류-시간-슬롯-예약) |
| 중간 합류 접합을 반영한 선행차 탐색 + ACC | [8.1](#81-선행차-탐색과-acc) |
| 차선 변경 갭 수용 (lead/lag 시간 간격) | [8.1](#81-선행차-탐색과-acc) |
| A\* 전역 경로와 중간 합류 접합 처리 | [5.1](#51-전역-계획--a) |

**조작** `Q`/`E` 차선 변경 · `1` `2` `3` 카메라 · `Esc` 허브

---

### 10.3 Urban — 신호 교차로와 보호 좌회전

![Urban 전경](docs/images/scene-urban.png)

**형상** 4방향 8접근로 신호 교차로. 접근로 정지선은 중심에서 16 m,
차선 중심은 ±1.8 / ±5.4 m.

**목적** 직진·좌회전·우회전을 UI로 고르면 서버가 신호·갭·보행자를 모두 확인하고
명령한다.

![보호 좌회전](docs/images/scene-urban-left-turn.png)

| 검증 대상 | 절 |
|---|---|
| 60 s 고정 주기 신호 (보행 페이즈 포함) — 서버가 집행 | [7.3](#73-신호-교차로) |
| 보호 좌회전 정책 + 늦은 갭 실패 시 직진 안전 취소 | [7.4](#74-보호-좌회전-정책) |
| 한국식 우회전 (적신호 완전 정지 후 양보 진행) | [7.3](#73-신호-교차로) |
| 보행자 횡단 예측 + 주행 통로 침범 판정 | [7.4](#74-보호-좌회전-정책) |
| 신호로 관리되는 충돌 필터링 | [6](#6-충돌-예측) |

**조작** 직진 / 좌회전 / 우회전 토글 · `1` `2` `3` 카메라 · `Esc` 허브

---

### 10.4 EmergencyAvoidance — 돌발 장애물과 긴급차 회피

![낙하물 회피](docs/images/scene-emergency-avoidance.png)

**형상** 직선 4차선 실험로(주행 3 + 갓길 1), 길이 320 m 이상.

**목적** 국부 샘플링 플래너를 수동 전환해 RRT와 RRT\*를 비교하는 전용 실험로.
실행 중 RRT ↔ RRT\*를 전환하며 비교할 수 있다.

![긴급차 양보](docs/images/scene-emergency-vehicle.png)

| 검증 대상 | 절 |
|---|---|
| 코리도 제한 RRT / RRT\* (실행 중 전환) | [5.2](#52-국부-계획--rrt--rrt) |
| 회피 상태기 (감지 → 계획 → 이탈 → 복귀 → 합류) | [7.5](#75-국부-회피-상태기) |
| 경로 후처리 (단축 · 재샘플링 · 78° 검증) | [5.2](#52-국부-계획--rrt--rrt) |
| 긴급차 우선: 갓길 대피 후 통과 확인 뒤 복귀 | [7.5](#75-국부-회피-상태기) |
| 계획 시간 / 최소 여유거리 실시간 계측 | [8.3](#83-로깅-스키마-고정) |

**조작** `4` 낙하물 · `5` 긴급차 · `6` RRT↔RRT\* · `0` 리셋 · `Esc` 허브

---

### 10.5 IntegratedCity — 통합 시나리오

![IntegratedCity 주행](docs/images/scene-integrated-city.png)

**형상** 교차로 → 대로 → 순환로로 이어지는 통합 코스. 도심 격자(`urban_*`)와
간선(`city_*`)이라는 **명명 규칙과 성격이 다른 두 도로 계열이 한 씬에 공존**한다.

**목적** 도심 신호, 외곽 순환, 갓길 테이퍼, 장애물·긴급차 이벤트를 선택적으로 통합한다.
보호 좌회전·사용자 우회전·Highway 온램프 합류를 모두 한 주행에서 재현하는 씬은 아니다.
위상 기반 판정([§7.2](#72-합류-시간-슬롯-예약))과 이름 무관 코리도
([§5.2](#52-국부-계획--rrt--rrt))가 필요한 이유를 보여준다.

| 검증 대상 | 절 |
|---|---|
| 도심 신호 + 갓길↔대로 테이퍼 합류 | [7.2](#72-합류-시간-슬롯-예약) |
| 이름이 아닌 위상으로 합류 지점을 찾아 예약 적용 | [7.2](#72-합류-시간-슬롯-예약) |
| 서로 다른 도로 계열을 가로지르는 회피 코리도 | [5.2](#52-국부-계획--rrt--rrt) |
| 다차량 동시 주행 중 전역 상황 인지·충돌 예측 | [6](#6-충돌-예측) |

**조작** 시나리오 디렉터가 이벤트를 자동 진행 · `1` `2` `3` 카메라 · `Esc` 허브

---

## 11. 실험 결과

세 실험 모두 **Unity 없이 헤드리스로** 재현된다.

```bash
python experiments/run_algorithm_compare.py   # 실험 1
python experiments/run_lka_test.py            # 실험 2
python experiments/run_scene_stats.py         # 실험 3
python experiments/make_charts.py             # -> experiments/results/charts/
python -m pytest server/tests -q --junitxml=experiments/results/pytest.xml
python experiments/validate_results.py         # CSV/JUnit/차트 무결성 + manifest
```

### 11.1 실험 1 — 경로 탐색 알고리즘 비교

세 시나리오로 구조화 도로망 탐색과 자유공간 샘플링의 적용 범위를 비교한다. 5개 시드에서
{1, 5, 20}개의 계획 질의를 순차 실행한다. 묶음은 동일 고정 난수열의 앞부분을 공유하므로
서로 독립 표본이 아니며, 동시 다차량 상호작용 실험도 아니다.

| 시나리오 | 세계 | 시험 대상 |
|---|---|---|
| `road_open` | 장애물 없는 2×2 일방통행 격자 | 구조화 도로망 기준 사례 |
| `road_detour` | 단일 직선 차선, 중간이 장애물로 막힘 | 차선 그래프에 **대체 간선이 없음** |
| `obstacle_field` | 비정형 장애물 6개가 흩어진 자유 코리도 | 그래프 밖 회피 |

**결과** (최대 차량 수, 시드 평균)

| 시나리오 | 플래너 | 성공률 | 계산시간 (ms) | 경로장 (m) | 경로장 표준편차 (m) | 노드 |
|---|---|---:|---:|---:|---:|---:|
| road_open | **A\*** | **100 %** | **0.36** | 233.8 | 1.96 | 8 |
| road_open | RRT | 100 % | 0.05 | 164.3 | 1.56 | 1 |
| road_open | RRT\* | 100 % | 456.9 | 168.5 | 3.15 | 1501 |
| road_detour | A\* | **0 %** | 0.07 | — | — | 1 |
| road_detour | **RRT** | **100 %** | **0.50** | 112.8 | 6.10 | 60 |
| road_detour | **RRT\*** | **100 %** | 553.3 | **96.7** | **0.21** | 1499 |
| obstacle_field | A\* | **0 %** | 0.07 | — | — | 1 |
| obstacle_field | **RRT** | **100 %** | **0.99** | 112.2 | 9.14 | 64 |
| obstacle_field | **RRT\*** | **100 %** | 739.2 | **97.0** | **0.96** | 1456 |

> 원자료: `experiments/results/algo_compare_summary.csv`, 20질의 행. 시간 값은 기기
> 의존적이다. **노드** 열은 플래너 계열마다 세는 대상이 다르다 — A\*는 전개한 차선
> 그래프 노드 수, RRT/RRT\*는 트리에 추가된 샘플 수다. RRT가 `road_open`에서 1인 것은
> 출발–목표 직선이 막히지 않아 트리를 자라게 할 필요가 없었다는 뜻이다. 성공은
> “비어 있지 않은 경로”가 아니라 **시작·목표 각 4 m 이내 연결 + 모든 경로 간선 충돌 없음**으로 판정한다.

![A* vs RRT vs RRT*](experiments/results/charts/algo_compare.png)

**고찰**

1. **구조화된 도로망에서는 A\*가 적합하다.** 이번 실행에서 0.36 ms에 *도로 규칙을 지키는* 233.8 m 경로를
   낸다. RRT가 낸 164 m는 격자를 가로지르고 통행 방향을 무시하는 대각선이라 **주행
   불가능**하다 — 짧은 경로장이 곧 좋은 경로가 아님을 보여주는 사례다.
2. **A\*는 그래프 밖 장애물을 다루지 못한다.** 막힌 차선은 후속 간선을 남기지 않으므로
   0.1 ms 안에 빈 경로를 반환하고 실패한다(성공률 0 %). 샘플링 플래너가 필요한 이유가
   이것이다.
3. **RRT는 빠르고, RRT\*는 낫지만 계산비용이 수백 배 크다.** 관측한 100개 질의에서 둘 다 해를 찾았다. RRT는
   ~1 ms에 거칠지만 실현 가능한 경로(112–113 m)를, RRT\*는 예산을 모두 써서 ~97 m
   (**약 14 % 단축**)로 다듬는다. 풀링된 경로장 표준편차도 RRT\*가 작지만(0.2–1.0 m
   대 6.1–9.1 m), 서로 다른 질의와 시드가 섞인 값이므로 시드 강건성으로 해석하지 않는다.
4. **운용 함의.** 이 구현에서 RRT\*는 질의당 약 0.5–0.8 s를 사용하므로 25 Hz 온라인
   루프에 직접 넣을 수 없다. 다만 여기서 20개 질의는 순차 호출이고 묶음 간 고정 난수열을 공유하므로, 이 결과만으로
   동시 다차량 스케줄러의 확장성을 주장하지 않는다. 현재 정책은 전역 도로 라우팅 A\*,
   온라인 장애물 우회 RRT, 경로 품질 비교 RRT\*다.

![계획 부하 대 차량 수](experiments/results/charts/algo_time_vs_vehicles.png)

### 11.2 실험 2 — LKA 횡방향 제어기 비교

곡선 트랙(반경 140 m, 길이 240 m)에서 40 / 60 / 80 / 100 km/h를 스윕했다. 각
제어기·속도 조합은 중심선 위 동일 초기조건의 **단일 결정론적 합성 실행**이며, Unity의
반경 90 m `LKA_Test` 씬 실측이 아니다. 미조정 기본 이득 기준 RMS 횡오차(m):

| 속도 (km/h) | Pure Pursuit | Stanley | PID |
|---:|---:|---:|---:|
| 40 | 0.089 | 0.041 | **0.016** |
| 60 | 0.113 | 0.055 | **0.014** |
| 80 | 0.135 | **0.049** | 0.239 |
| 100 | 0.153 | **0.058** | 0.303 |

![LKA 횡오차 대 속도](experiments/results/charts/lka_lateral_error.png)

**고찰**

1. **이 시험 조건에서는 Stanley의 속도 민감도가 가장 작다** — 전 속도에서
   0.04–0.06 m로 평탄했다. 잡음·초기 횡오차·조향 지연에 대한 강건성은 검증하지 않았다.
2. **Pure Pursuit는 이 설정에서 속도에 따라 열화한다** (0.089 → 0.153 m). 전방주시거리는
   `4.0 + 0.4v`로 속도 가변이므로, 원인을 고정 lookahead로 단정하지 않고 현재 선형 법칙과
   미조정 이득의 결합 효과로만 해석한다.
3. **PID는 저속에서 오차가 가장 작았으나 고속에서 크게 증가했다** (0.016 → 0.303 m). 피드포워드 없는
   순수 오차 되먹임의 한계다.
4. 이 곡률·초기조건에서는 어떤 제어기도 차선을 이탈하지 않았다(departures = 0).
   이는 이 표본의 관측일 뿐 안전성을 입증하지 않으며, 이득 정정과 Unity 물리 검증이 필요하다.

### 11.3 실험 3 — 씬별 제어 루프 부하

실험 1·2가 각각 플래너와 제어기를 격리해 재는 반면, 이 실험은 라우팅·신호·합류 예약·
좌회전 정책·충돌 예측·ACC를 포함한 **Python 중앙 제어기 계산 커널**을 잰다. 각 씬에
[§12](#12-검증-체계) 회귀 스위트와 동일한 교통을 넣고 한 번씩 헤드리스로 구동한다.
WebSocket, JSON 직렬화·스키마 검증, Unity 메인 스레드 적용 및 렌더링은 포함하지 않는다.

| 시나리오 | 차량 | 구간 | `step()` p50 / p95 (ms) | 동일차선 최소간격 (m) | 최소 TTC (s) | 급제동 에피소드 | 10 s 내 최대 행동전환 |
|---|---:|---:|---:|---:|---:|---:|---:|
| LKA_Test | 1 | 40 s | 0.06 / 0.21 | — | ∞ | 1 | 0 |
| Highway (램프 합류) | 4 | 40 s | 0.59 / 0.75 | 10.99 | ∞ | 24 | 1 |
| Urban (8접근로 + 보행자) | 8 | 120 s | 0.42 / 1.10 | 15.05 | 3.87 | 22 | 2 |
| EmergencyAvoidance (낙하물 + 긴급차) | 2 | 50 s | 0.38 / 0.57 | 30.06 | 1.18 | 15 | 8 |
| IntegratedCity (갓길 합류 + 장애물) | 3 | 50 s | 0.74 / 1.40 | 29.13 | 3.12 | 16 | 6 |

> 원자료: `experiments/results/scene_stats.csv`. 2026-08-07, Windows
> 10.0.26200, CPython 3.14.4, 단일 실행. p50/p95는 실행 간 신뢰구간이 아니라 한 실행의
> 틱 분포다. 실행 조건과 해석 경계는 `experiments/results/README.md`에 고정한다.
> `validate_results.py`는 이 표의 씬 식별·차량 수·틱 수·지표 범위와
> 파일 해시를 검사한다. 씬별 raw tick 로그가 없으므로 간격·TTC·급제동을
> 독립 재집계하는 검증은 아니며, 현재 manifest는 무결성 출처로 한정한다.

**고찰**

1. **서버 계산 커널은 현재 규모에서 명목 간격보다 짧다.** Unity 고정 스텝 0.02 s,
   2틱마다 송신하므로 명목 간격은 40 ms다. 가장 큰 p95 1.40 ms는 이 간격의 3.5 %다.
   그러나 실제 폐루프 여유는 왕복 지연을 함께 계측하기 전에는 확정할 수 없다. 편집기
   로그에서 응답 지연 시 전송 건너뛰기 경고가 실제로 관찰됐다.
2. **씬 간 차이는 설명적 관찰에 그친다.** Urban 8대보다 IntegratedCity 3대의 p95가
   높지만 차량 수·네트워크·정책 수·실행 길이를 통제하지 않았으므로 정책 다양성이
   원인이라고 결론내리지 않는다. 확장성은 차량 수와 활성 정책 수를 독립 스윕해야 한다.
3. **샘플링 플래너 틱에서 스파이크가 관찰된다.** 이번 EmergencyAvoidance 실행의
   최댓값은 13.05 ms였고 이전 실행에서는 16–28 ms였다. 낙하물 투입 시점의 RRT 호출과
   일치하지만, 단일 실행이므로 45 ms 제한의 충분성을 증명하는 값으로 쓰지 않는다.
   반대로 RRT\*의 500 ms대 질의(실험 1)는 이 예산에 **들어갈 수 없다** — 실행 중
   RRT\*로 전환하는 것이 비교 실험용이지 운용 설정이 아닌 이유다.
4. **이 표본에서 동일 차선 중심 간 최소거리는 11.0 m였다.** 이는 차체 footprint 기반
   충돌 카운터가 아니므로 “추돌 없음”의 증거로 일반화하지 않는다. 최소 TTC 1.18 s는
   제어 차량이 포함된 보고 conflict 중 최솟값이며, 검열된 지표다.
5. **급제동 수치는 헤드리스 시뮬레이터의 한계로 읽어야 한다.** 다차량 씬 네 곳에서
   관측된 최대 감속은 정확히 **6.0 m/s²** (교통이 없는 LKA_Test만 4.19 m/s²로
   포화하지 않는다) — 6.0은 `headless_sim`의 1차 속도 추종기 상한이지
   제어기가 요청한 값이 아니다. 즉 목표 속도가 계단식으로 떨어질 때 시뮬레이터가
   포화한 것이며, 승차감 평가는 차량 동역학을 가진 Unity 쪽에서 해야 한다. 여기서
   행동 전환 횟수는 10 s당 최대 8회로 회귀 기준 20회보다 낮았다. 이는 해당 실행이
   회귀 임계값을 통과했다는 뜻이지, 모든 초기조건에서 한계 진동이 없다는 증명은 아니다.

전체 해석은 [`docs/experiment_results.md`](docs/experiment_results.md)와
[`docs/phase7_report.md`](docs/phase7_report.md)에 있다.

---

## 12. 검증 체계

```bash
python -m pytest server/tests -q
```

**259건** 수집, **257 pass / 2 skip**. 2026-08-07 Windows 샌드박스에서 위 명령으로
재검증했으며 JUnit 스위트 시간은 240.2 s(외부 wall time 242 s)였다. 스킵 2건은 도로가 없는 Main 허브에 차선 export
검사를 적용하지 않는 의도적 스킵이다. 캐시 디렉터리 생성 경고 1건은 있었으나 테스트
실패나 오류는 없었다.

| 파일 | 건수 | 대상 |
|---|---:|---|
| `test_scene_networks.py` | 67 | 차선 그래프 정적 감사, 양측 계획 대조 |
| `test_phase6_urban.py` · `test_left_turn_policy.py` | 다수 | 신호·좌회전·보행자 |
| `test_phase5_highway.py` | 다수 | 합류 예약·차선 변경 |
| `test_rrt.py` · `test_astar.py` | 13 · — | 플래너 |
| `test_collision_predictor.py` · `test_behavior_acc.py` · `test_lateral.py` | — | 예측·제어 |
| `test_local_avoidance.py` · `test_emergency.py` | — | 회피 상태기 |
| `test_regression_drive.py` | — | 전 씬 실주행 회귀 |
| `test_review_fixes.py` | 10 | 과거 감사에서 나온 개별 결함 고정 |
| `test_protocol.py` · `test_world_model.py` · `test_logging_csv.py` · `test_noise.py` · `test_headless_sim.py` | — | 인프라 |

### 12.1 드리프트 검출 — 1순위 리스크 대응

일반 단위·통합 테스트와 별개로, [§3.3](#33-동기-규약--본-프로젝트의-1순위-리스크)의
위험을 직접 검증하는 검사를 둔다.

| 검사 | 무엇을 막는가 |
|---|---|
| 씬 ↔ 차선 export 대조 | 씬을 고치고 재export를 잊어, 서버가 **존재하지 않는 도로** 위에서 계획하는 것 |
| Unity ↔ Python 신호 계획 대조 | 화면은 녹색인데 서버는 적색으로 붙잡는 상태 |
| 두 스키마의 `scenario` enum 동기 | Unity가 보고할 수 없는 시나리오가 생기는 것 |
| 차선 그래프 정적 감사 | 끊긴 참조, 비대칭 인접, 주행 불가능한 접합, 경로 점프 |
| 상충 접근로 동시 녹색 없음 (주기 전수 스윕) | 신호 계획 자체의 모순 |

### 12.2 회귀 주행 — 실주행 불변식

`test_regression_drive.py`는 각 씬에 실제 교통과 이벤트를 넣고 구동한 뒤, 일곱 차례의
감사에서 매번 새로운 형태로 깨졌던 **세 가지 속성**을 검사한다.

| 불변식 | 임계 | 무엇의 징후인가 |
|---|---|---|
| 추돌 없음 | 동일 차선 두 차량이 4.0 m 이내로 접근하지 않음 | 인접 차선은 제외 — 갓길 테이퍼는 중심선이 1.98 m로 붙는 것이 정상 |
| 끼임 없음 | 이동 명령을 받고도 2 s 이상 정지하지 않음 | 낡은 경로, 잃어버린 루트, 빠져나갈 수 없는 기동 단계 |
| 진동 없음 | 10 s 창 안에서 행동이 20회 넘게 바뀌지 않음 | 제동이 멈추는 순간 해제되는 충돌, 창-정지를 오가는 4 s 예측 |

---

## 13. 한계와 향후 과제

### 13.1 연구 질문에 대한 답

| 질문 ([§1.3](#13-연구-질문)) | 현재까지의 답 |
|---|---|
| 중앙 관제가 합류에 제공하는 협조 기동은 무엇인가 | 본 구현의 중앙 서버는 본선 차량에게 감속을 지시해 갭을 연다([§7.2](#72-합류-시간-슬롯-예약)). 램프 차량 단독 제어보다 넓은 행동공간이지만, 분산 협조 V2X와의 우열은 baseline이 없어 주장하지 않는다 |
| 어떤 탐색 알고리즘이 어디에 맞는가 | 구조화된 도로망은 A\*, 그래프 밖 장애물은 RRT, 품질 벤치마킹만 RRT\*. A\*는 이 막힌 차선 표본에서 성공률 0 %, RRT\*는 질의당 약 0.5–0.8 s로 현재 온라인 주기에 부적합 ([§11.1](#111-실험-1--경로-탐색-알고리즘-비교)) |
| 횡방향 제어는 어떻게 열화하는가 | 미조정 이득 기준 Stanley만 40–100 km/h에서 평탄(0.04–0.06 m). Pure Pursuit는 단조 열화, PID는 80 km/h부터 오차가 크게 증가 ([§11.2](#112-실험-2--lka-횡방향-제어기-비교)) |
| 서버 계산 커널은 명목 송신 간격 안에 드는가 | 현재 5개 헤드리스 사례에서는 든다. 최악 p95 1.40 ms로 40 ms 간격의 3.5 %. 단, 왕복 지연·직렬화·Unity 적용을 포함한 폐루프 검증은 아니다 ([§11.3](#113-실험-3--씬별-제어-루프-부하)) |

### 13.2 알려진 한계

| 한계 | 내용 |
|---|---|
| **규모** | 실측한 최대 동시 차량은 8대다. 충돌 예측이 전 쌍 O(n²)이므로 수백 대 규모에서는 공간 분할이 필요하다 — 현재 구조로는 검증되지 않았다 |
| **성능 통계** | 씬별 부하는 단일 실행이며 워밍업·반복 실행·신뢰구간·CPU 모델을 기록하지 못했다. 현재 값은 회귀용 기준선이지 일반화 가능한 성능 보증이 아니다 |
| **종단간 지연 미측정** | 서버 `step()`만 계측했다. WebSocket, JSON/스키마, Unity 적용·렌더링을 포함한 state→command 왕복 지연과 전송 누락률은 별도 계측이 필요하다 |
| **승차감 미측정** | 헤드리스 시뮬레이터의 속도 추종기가 6.0 m/s²에서 포화하므로, 급제동·저크 지표는 이 저장소의 숫자로 판단할 수 없다. 차량 동역학을 가진 Unity 측 로그가 필요하다 |
| **이득 미정정** | Pure Pursuit / Stanley / PID 모두 기본값이다. 비교는 유효하나 절대 성능은 정정 후에야 의미가 있다 |
| **완전 V2X 가정** | 통신 지연·패킷 손실·부분 관측이 없다. `server/noise.py`에 잡음 주입기만 준비되어 있고 기본 파이프라인에서는 비활성이다 |
| **씬 의존 상수** | 신호 계획과 일부 정지선 좌표가 `central_control.py`에 하드코딩되어 있다. 새 교차로를 추가하려면 서버 코드를 고쳐야 한다 |
| **인지 부재** | 설계상 의도된 범위 밖이지만, 결과를 인지 오차가 있는 시스템으로 일반화할 수는 없다 |

### 13.3 향후 과제

1. **잡음·지연 하에서의 재평가.** `noise.py`를 파이프라인에 넣고 위 네 실험을 다시 돌려,
   완전 V2X 가정이 결론을 얼마나 떠받치고 있었는지 정량화한다.
2. **다차량 규모 실험.** 20–100대 구간에서 `step()` 비용 곡선을 측정해 O(n²) 충돌
   예측이 언제 병목이 되는지 확인한다.
3. **Unity 측 승차감 로그.** 동일 시나리오를 Unity에서 구동해 실제 감속·저크 분포를
   얻고, 헤드리스 지표와 대조한다.
4. **신호 계획의 데이터화.** 하드코딩된 신호 정의를 차선 export와 같은 계층의
   설정 파일로 옮겨, 교차로 추가가 서버 코드 수정을 요구하지 않게 한다.

---

# IV. 부록

## 14. 실행 방법

### 14.1 Python 서버

```bash
cd server
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python main.py --network scenarios/Urban_lanes.json
```

`--network`로 해당 씬의 차선 export를 지정한다. 생략하면 합성 네트워크로 뜨므로,
Unity 씬과 함께 돌릴 때는 **반드시 지정**해야 한다.

| 씬 | `--network` |
|---|---|
| LKA_Test | `scenarios/LKA_Test_lanes.json` |
| Highway | `scenarios/Highway_lanes.json` |
| Urban | `scenarios/Urban_lanes.json` |
| EmergencyAvoidance | `scenarios/EmergencyAvoidance_lanes.json` |
| IntegratedCity | `scenarios/IntegratedCity_lanes.json` |

### 14.2 Unity

1. `unity/` 폴더를 Unity 6로 연다.
2. `Main` 씬을 열고 Play — 허브에서 씬을 고른다.
3. 씬을 처음 만들거나 다시 만들려면 메뉴 **`V2X > Build All Demo Scenes`**
   (허브만 다시 만들려면 `V2X > Build Main Hub`).
4. 도로를 편집했다면 **`V2X > Export Lane Network...`** 로 반드시 다시 내보낸다.
   빠뜨리면 [§12.1](#121-드리프트-검출--1순위-리스크-대응)의 대조 테스트가 잡아낸다.

서버가 꺼져 있어도 허브와 씬은 뜨지만 차량은 움직이지 않는다 — 명령을 주는 쪽이
서버이기 때문이다.

### 14.3 사람이 해야 하는 작업

다음은 자동화하지 않는다: Unity 에디터 씬 구성, 도로·차선 기하 배치, 제어기 이득
정정(Pure Pursuit 전방주시거리, Stanley 이득, PID), Unity–Python 타임스텝 동기 디버깅.

---

## 15. 저장소 구성

```
autonomous-simulator/
├─ unity/Assets/Scripts/
│  ├─ Communication/   V2XClient + 메시지 클래스 (와이어 포맷 미러)
│  ├─ Vehicle/         VehicleController, LKAController
│  ├─ Road/            Lane, RoadNetworkManager
│  ├─ Sim/             SimulationManager + 씬별 시나리오 디렉터
│  ├─ UI/              허브, 카메라, 신호 표시, 씬별 조작 패널
│  └─ Editor/          씬 빌더, 차선 export, 서버 자동 실행
├─ server/
│  ├─ main.py             WebSocket 서버 + 스키마 검증 + 동기 검사
│  ├─ central_control.py  매 틱 정책의 중심 (신호·좌회전·합류·회피 조율)
│  ├─ world_model.py      차선 그래프 + 동적 스냅샷
│  ├─ planners/           A*, RRT, RRT*, 회피 탐색공간
│  ├─ controllers/        ACC(종방향), lateral(횡방향)
│  ├─ scenarios/          씬별 차선 export (Unity가 내보낸 것)
│  ├─ tools/              fake Unity 클라이언트, Unity 씬 리더
│  └─ tests/              259건
├─ experiments/           실험 러너 + 결과 CSV/차트 + 무결성 manifest
├─ shared/protocol/       JSON Schema — 와이어 포맷의 유일한 진실
└─ docs/                  설계안, 프로토콜, 실험 결과, 일일 워크로그
```

### 15.1 문서

| 문서 | 내용 |
|---|---|
| [`docs/project_plan.md`](docs/project_plan.md) | 전체 설계안 (연구 질문부터 평가 지표까지) |
| [`docs/api_protocol.md`](docs/api_protocol.md) | 메시지 포맷과 동기 규약 |
| [`docs/left_turn_behavior.md`](docs/left_turn_behavior.md) | 보호 좌회전 정책 상세 |
| [`docs/emergency_avoidance_plan.md`](docs/emergency_avoidance_plan.md) | 회피 씬 설계 |
| [`docs/experiment_results.md`](docs/experiment_results.md) | A\*/RRT/RRT\*, LKA 비교 결과 |
| [`docs/phase7_report.md`](docs/phase7_report.md) | Phase 7 보고서 |
| [`docs/unity_setup.md`](docs/unity_setup.md) | Unity 프로젝트 세팅 |
| [`docs/worklog/`](docs/worklog/) | 일일 작업 로그 (그날 고친 결함과 다음 착수 지점) |
| [`CLAUDE.md`](CLAUDE.md) | 코딩 에이전트가 먼저 읽어야 할 작업 맥락 |

### 15.2 Git

- `.gitignore`가 Unity `Library/` · `Temp/` · 빌드 산출물과 Python `__pycache__/` ·
  `.venv/`를 제외한다.
- Unity 바이너리 에셋은 **Git LFS**로 관리한다(`.gitattributes`). 첫 커밋 전에
  `git lfs install`을 한 번 실행한다.

---

## 16. 참고문헌

아래 원전은 알고리즘의 정의와 설계 배경을 위한 근거다. 본 저장소의 성능
수치와 안전성 주장은 이 문헌에서 가져오지 않고 §11의 내부 실험으로만 한정한다.

1. P. E. Hart, N. J. Nilsson, B. Raphael, “A Formal Basis for the Heuristic
   Determination of Minimum Cost Paths,” *IEEE Transactions on Systems Science
   and Cybernetics*, 4(2), 100–107, 1968. [doi:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136)
2. S. M. LaValle, “Rapidly-Exploring Random Trees: A New Tool for Path Planning,”
   Technical Report, 1998. [author PDF](https://lavalle.pl/papers/Lav98c.pdf)
3. S. Karaman, E. Frazzoli, “Sampling-based Algorithms for Optimal Motion
   Planning,” *The International Journal of Robotics Research*, 30(7), 846–894,
   2011. [doi:10.1177/0278364911406761](https://doi.org/10.1177/0278364911406761)
4. R. C. Coulter, “Implementation of the Pure Pursuit Path Tracking Algorithm,”
   CMU-RI-TR-92-01, 1992. [Carnegie Mellon Robotics Institute](https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm)
5. G. M. Hoffmann, C. J. Tomlin, M. Montemerlo, S. Thrun, “Autonomous Automobile
   Trajectory Tracking for Off-Road Driving: Controller Design, Experimental
   Validation and Racing,” *American Control Conference*, 2296–2301, 2007.
   [doi:10.1109/ACC.2007.4282788](https://doi.org/10.1109/ACC.2007.4282788)
