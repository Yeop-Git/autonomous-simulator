# 중앙 집중형 V2X 자율주행 시뮬레이터

**완전한 V2X 세계**를 가정한 자율주행 시뮬레이터입니다. 중앙 서버가 모든 차량·보행자·
장애물·긴급차의 위치와 속도를 오차 없이 알고 있다고 두고, 그 위에서 **다차량 경로 계획과
충돌 회피**를 연구합니다. 카메라/LiDAR 인지는 의도적으로 범위 밖입니다 — 인지가 완벽할 때
남는 문제, 즉 *누가 언제 어디로 가야 하는가* 에만 집중하기 위해서입니다.

Unity는 그리고 움직이고, Python 서버는 결정합니다.

![IntegratedCity 전경](docs/images/hero.png)

```
상태 수집  →  위험 예측  →  (재)계획  →  행동 결정  →  차량 제어  →  Unity
   Unity        Python 중앙 서버                                    Unity
```

---

## 목차

- [무엇을 만들었나](#무엇을-만들었나)
- [기반 기술과 알고리즘](#기반-기술과-알고리즘)
- [씬 안내](#씬-안내)
- [실행 방법](#실행-방법)
- [테스트와 실험](#테스트와-실험)
- [저장소 구조](#저장소-구조)
- [문서](#문서)

---

## 무엇을 만들었나

| | |
|---|---|
| **중앙 관제** | 서버가 전 차량의 상태를 한 번에 보고 매 틱 명령을 내림 |
| **경로 탐색** | A\*(전역, 차선 그래프) + RRT/RRT\*(국부, 자유공간) 비교 |
| **고속도로** | 온램프 합류 시간 슬롯 예약, 차선 변경 갭 수용 |
| **시내** | 신호 교차로, 보호 좌회전, 한국식 우회전, 보행자 양보 |
| **돌발 상황** | 낙하물 회피, 긴급차 양보, 회피 후 원차선 복귀 |
| **LKA/ADAS** | Pure Pursuit / Stanley 횡방향, ACC 종방향 |
| **정량 평가** | 고정 CSV 스키마 → 계산시간·경로길이·lateral error 비교 |

씬은 6종(허브 1 + 주행 5), 서버 테스트는 **250여 건**이 상시 그린입니다.

---

## 기반 기술과 알고리즘

씬 설명에 들어가기 전에, 모든 씬이 공유하는 토대부터 봅니다.

### 1. 중앙 집중 월드 모델

정적인 **차선 그래프**(`LaneNetwork`)와 매 틱 갱신되는 **동적 스냅샷**(`WorldModel`)을
분리합니다. 차선 그래프는 Unity 씬에서 사람이 배치한 도로를 그대로 내보낸 것이고,
동적 스냅샷은 Unity가 매 물리 틱마다 보내주는 전 객체의 위치·속도입니다.

- 차선 하나 = 중심선 폴리라인 + 폭/제한속도 + 좌/우 인접 + 후속 차선 목록
- 서버는 이 그래프 위에서만 계획하므로, **도로 형상의 진실은 언제나 Unity 씬**입니다

> `server/world_model.py`

### 2. 통신 계약 — 이 프로젝트의 1순위 리스크

메시지는 딱 두 종류이고, **JSON Schema가 유일한 진실**입니다.

| 방향 | 메시지 | 내용 |
|---|---|---|
| Unity → Python | `state_message.schema.json` | 매 틱 전체 월드 스냅샷 |
| Python → Unity | `command_message.schema.json` | 차량별 목표 속도·경로·행동 |

양쪽 모두 `time`과 `tick`을 실어 보내 **서로가 어긋났는지 검출**합니다.

- 서버: 중복 tick / 역순 tick / 건너뛴 tick을 경고로 출력
- Unity: 전송이 겹치지 않도록 직렬화(`_isSending`), 명령이 N틱 이상 밀리면 경고
- tick이 뒤로 가면(= Unity가 씬을 다시 로드해 시계를 리셋) 서버는 **차량별 기억을 전부
  버립니다** — 안 그러면 출발선에 돌아온 차에게 "너는 아직 회피 중"이라고 우깁니다
- 신호 계획과 차선 그래프는 양쪽에 사본이 존재하므로, **드리프트를 잡는 테스트**를
  따로 둡니다(아래 [테스트](#테스트와-실험) 참고)

> `shared/protocol/`, `server/main.py`, `unity/Assets/Scripts/Communication/`

### 3. 전역 경로 계획 — A\*

차선 그래프 위의 A\*입니다. 간선 비용은 차선 길이, 휴리스틱은 차선 끝에서 목표까지의
직선거리입니다. 찾은 차선 열을 하나의 웨이포인트 경로로 **이어 붙일 때**가 까다롭습니다.

- 후속 차선을 그 차선의 **시작점**이 아니라 *직전 차선이 끝난 지점의 투영*에서 이어 붙임
  → 온램프처럼 본선 중간(arc 115 m)으로 합류하는 접합에서 경로가 뒤로 튀지 않음
- 교차로 정지선에서는 직진·좌회전 커넥터가 **같은 점에서 출발**하므로, 최근접 차선이
  동률이면 순서대로 재시도 → "경로 없음"으로 교차로에 서 버리는 일 방지

> `server/planners/astar.py`

### 4. 국부 경로 계획 — RRT / RRT\*

전역 경로는 A\*가 유지하되, 돌발 장애물 앞에서는 샘플링 플래너가 **일시적으로 경로를
넘겨받습니다**. 자유공간 전체가 아니라 **차선 코리도로 제한**된 공간에서 탐색합니다.

- 코리도 = 현재/목표/원래 차선의 **좌우 인접 그룹 + 그 후속 차선** (차선 이름과 무관)
- 장애물·타 차량은 3초 예측을 반경으로 부풀려 점유 공간으로 취급
- 경로 후처리: 충돌을 보존하는 **단축(shortcut)** → 2 m 간격 **재샘플링** → 최대 조향각
  78° 초과 시 폐기
- 시간 예산: RRT 45 ms / RRT\* 140 ms — 계획 시간과 최소 여유거리를 명령에 실어 보고

> `server/planners/rrt.py`, `rrt_star.py`, `avoidance_world.py`, `path_postprocess.py`

### 5. 충돌 예측

샘플링이 아니라 **해석적으로** 풉니다. 상대 위치·속도로 이차방정식을 세워 안전거리를
처음 침범하는 시각을 직접 구하므로, 빠른 횡단 차량이 샘플 사이로 빠져나가는
터널링이 없습니다.

- 최근접 접근 시각/거리 + 안전거리 침범까지의 시간(TTC), 지평선 4 s, 안전거리 2.5 m
- **이미 안전거리 안이면서 서로 접근 중이 아닌** 위반은 제어층에서 제외 — 정지한 두 차가
  서로를 영원히 급제동시키는 교착을 막습니다

> `server/collision_predictor.py`

### 6. 선행차 탐색과 ACC

ACC가 따라갈 **선행차**를 찾는 일이 생각보다 어렵습니다. 세 경로로 찾습니다.

1. 같은 차선에서 앞선 차 (arc 비교)
2. **하류 차선** — 후속 차선의 진입 arc를 빼서 거리 계산 (중간 합류 접합 대응)
3. **형제 차선** — 같은 접합점으로 들어오는 다른 차선. 먼저 도착하는 쪽이 선행차
   (지퍼 병합)

찾은 선행차에 대해 ACC는 *일정 시간 간격* 정책으로 목표 속도를 냅니다(희망 차두 2 s,
정지 시 최소 간격 6 m, 편안한 감속 4 m/s², 급제동 8 m/s²).

> `server/behavior.py`, `server/controllers/acc.py`

### 7. 행동 결정 FSM

우선순위 기반의 순수 함수입니다: **긴급제동 > 도착 > 경로없음 정지 > 주의정지 >
추종 > 순항**. 상태를 기억하지 않고 매 틱 전체 상황으로 다시 판단하므로, 전이 표가
꼬여 빠져나오지 못하는 상태가 없습니다.

> `server/behavior.py`

### 8. 횡방향 제어 — Pure Pursuit / Stanley

라이브 데모는 Unity의 C# `LKAController`가, 정량 실험은 동일한 법칙의 Python 구현이
헤드리스로 돌립니다. 두 구현이 같은 인터페이스라 실험 러너가 갈아 끼웁니다.

- Pure Pursuit: 전방주시거리 기반, 저속·완만한 곡선에 강함
- Stanley: 전륜 기준 횡오차 + 헤딩오차, 곡률이 큰 구간에서 유리
- 공통으로 **Frenet 오차**(lateral / heading / curvature)를 로그로 남김

> `server/controllers/lateral.py`, `unity/Assets/Scripts/Vehicle/LKAController.cs`

### 9. 합류 시간 슬롯 예약 — 중앙 관제의 대표 기술

램프 차량이 스스로 눈치 보는 대신, 서버가 본선 전 차량의 ETA를 계산해 **슬롯을 예약**합니다.

1. 본선 차량들의 합류점 도착 시각(ETA)을 정렬 — 이미 지나간 차는 제외
2. 램프 차량의 ETA가 이미 충분한 갭 안이면 그대로 통과
3. 아니면 가장 가까운 큰 갭으로 **램프 차량을 재타이밍**(속도 지시)
4. 도달 가능한 갭이 없으면 **본선 차량에게 감속을 지시해 갭을 열게** 함 — 램프 차량
   혼자서는 절대 할 수 없는 수

합류 차선인지 여부는 이름이 아니라 **위상과 기하**로 판정합니다: ① 후속 차선의 중간으로
접합하거나(온램프), ② 접합점 15 m 전에서 나란히 달리는 더 느린 차선(갓길 테이퍼).

> `server/merge.py`, `server/central_control.py`

### 10. 신호 교차로

고정 주기(60 s) 신호를 **서버가 집행**하고 Unity가 그립니다. 같은 계획이 양쪽에 있으므로
드리프트 검출 테스트로 묶어 두었습니다.

- 동서 녹색 10 s → 보행 페이즈 8 s(전적색) → 남북 녹색 10 s → 보호 좌회전 6 s → 보행 8 s
- 정지는 정지선 5.5 m 앞에서, 1.8 m/s²의 편안한 감속으로 미리 시작
- 황색은 "편하게 설 수 있으면 선다" — 무조건 정지가 아님
- 녹색 접근로가 적색 접근로 때문에 급제동하지 않도록 **신호로 관리되는 충돌**은 필터링

> `server/traffic.py`, `unity/Assets/Scripts/UI/TrafficLightSystem.cs`

### 11. 보호 좌회전 정책

전이 기반 FSM이 아니라 **매 틱 전체 스냅샷을 다시 읽는 우선순위 정책**입니다. 틱 사이로
넘어가는 약속은 두 개뿐입니다 — 이미 시작한 횡방향 이동, 그리고 늦은 취소.

```
원차선 → (갭 수용) 좌회전 차선 → 정지선 → 화살표 대기 → 교차로 진입
       ↘ (마감까지 갭 없음) 직진으로 안전 취소
```

- 녹색이어도 앞차가 서 있거나, 보행자가 **주행 통로 안에** 있거나, 출구가 막혔으면 대기
  (보행자는 원형 반경이 아니라 실제로 지나갈 경로에 투영해 판정 — 인도의 사람은 무시)
- 차선 변경은 정지선 약 14 m 전에 끝나도록 거리를 잡고, 정지 대기열이 있으면 그 뒤에
  설 만큼만 전진

> `server/left_turn.py`, `server/central_control.py`

### 12. 국부 회피 상태기

```
위험 감지 → 탈출 계획 → 횡방향 이탈 → (긴급차면 양보 대기) → 복귀 계획 → 원차선 합류
                  ↘ 계획 실패 → 통제 정지(0.75 s 주기 재시도)
```

- 탈출 경로 끝에 도달했는데 장애물이 아직 옆이면 **경로를 연장**
- 복귀가 막히면 정지하지 않고 **회피 차선에서 계속 주행하며 1.5 s 뒤 재시도**
- 어느 단계에서든 경로가 비면 속도 명령을 0으로 — 경로 없이 달리는 상태를 만들지 않음

> `server/local_avoidance.py`

### 13. 로깅·지표·실험

CSV 컬럼이 **고정**되어 있어 분석 노트북이 깨지지 않습니다.

```
time, vehicle_id, scenario, position_x, position_z, speed, lane_id,
behavior_state, lateral_error, heading_error, target_speed,
collision_risk, ttc, event_type
```

지표: 평균 속도, 최소 TTC, 차선 이탈 횟수, 급제동 횟수, 도착 수, 위험 이벤트 수.

> `server/logging_csv.py`, `server/metrics.py`, `experiments/`

---

## 씬 안내

Unity를 실행하면 **Main 허브**가 먼저 뜨고, 거기서 원하는 씬으로 들어갑니다.
각 씬에서는 `Esc` 또는 좌상단 버튼으로 허브에 돌아옵니다.

### Main — 허브

![Main 허브](docs/images/scene-main-hub.png)

주행 씬이 아니라 **메뉴**입니다. 씬 목록에서 하나를 고르면 그 씬의 목적·들어간 기술·
조작법이 오른쪽 패널에 뜨고, `Enter` 또는 실행 버튼으로 진입합니다.

- 숫자키 `1`~`5` 선택 · `Enter` 실행
- Build Settings에 없는 씬을 고르면 멈추는 대신 무엇이 빠졌는지 화면에 알려줌

> 도로가 없는 씬이라 차선 export도 없습니다.

---

### LKA_Test — 차선 유지 시험로

![LKA_Test](docs/images/scene-lka-test.png)

일정 곡률의 단일 곡선 트랙. 다른 차량도 이벤트도 없이 **횡방향 제어기만 홀로** 남겨
성능을 읽습니다.

| 들어간 기술 | 어디서 |
|---|---|
| 횡방향 제어 Pure Pursuit / Stanley (기본 Stanley) | [8](#8-횡방향-제어--pure-pursuit--stanley) |
| Frenet 오차(lateral / heading / curvature) 계산 | [8](#8-횡방향-제어--pure-pursuit--stanley) |
| ACC 자유주행 구간 | [6](#6-선행차-탐색과-acc) |
| 고정 CSV 로깅 → 속도별 RMS 오차 비교 | [13](#13-로깅지표실험) |

**조작** `1` `2` `3` 카메라 전환 · `Esc` 허브로

---

### Highway — 고속도로 합류와 차선 변경

![Highway 전경](docs/images/scene-highway.png)

3차선 본선(제한속도 27.8 m/s)과 온램프(18 m/s). 램프는 본선 **중간 지점(z=115)** 으로
합류하므로, 단순한 끝-시작 접합보다 훨씬 까다롭습니다.

![합류 순간](docs/images/scene-highway-merge.png)

| 들어간 기술 | 어디서 |
|---|---|
| V2X 합류 시간 슬롯 예약 (재타이밍 → 본선 양보) | [9](#9-합류-시간-슬롯-예약--중앙-관제의-대표-기술) |
| 합류 차선 판정을 위상·기하로 도출 | [9](#9-합류-시간-슬롯-예약--중앙-관제의-대표-기술) |
| 중간 합류 접합을 반영한 선행차 탐색 + ACC | [6](#6-선행차-탐색과-acc) |
| 차선 변경 갭 수용 (lead/lag 시간 간격) | [6](#6-선행차-탐색과-acc) |
| A\* 전역 경로 + 낙하물 이벤트 재계획 | [3](#3-전역-경로-계획--a) |

**조작** `Q`/`E` 차선 변경 · `1` `2` `3` 카메라 · `Esc` 허브로

---

### Urban — 신호 교차로와 보호 좌회전

![Urban 전경](docs/images/scene-urban.png)

4방향 8접근로 신호 교차로. 직진·좌회전·우회전을 UI로 고르면 서버가 신호·갭·보행자를
모두 확인하고 명령합니다.

![보호 좌회전](docs/images/scene-urban-left-turn.png)

| 들어간 기술 | 어디서 |
|---|---|
| 고정 주기 신호 (보행 페이즈 포함) — 서버가 집행 | [10](#10-신호-교차로) |
| 보호 좌회전 정책 + 늦은 갭 실패 시 직진 안전 취소 | [11](#11-보호-좌회전-정책) |
| 한국식 우회전 (적신호 완전 정지 후 양보하며 진행) | [10](#10-신호-교차로) |
| 보행자 횡단 예측 + 주행 통로 침범 판정 | [11](#11-보호-좌회전-정책) |
| 신호로 관리되는 충돌 필터링 | [5](#5-충돌-예측) |

**조작** 직진 / 좌회전 / 우회전 토글 · `1` `2` `3` 카메라 · `Esc` 허브로

---

### EmergencyAvoidance — 돌발 장애물과 긴급차 회피

![낙하물 회피](docs/images/scene-emergency-avoidance.png)

직선 4차선 실험로(주행 3 + 갓길 1). 주행 중 낙하물이 떨어지고, 뒤에서 긴급차가
접근합니다. 전역 A\* 대신 **국부 샘플링 플래너가 일시적으로 경로를 넘겨받는** 유일한
씬이며, 실행 중 RRT ↔ RRT\*를 바꿔 가며 비교할 수 있습니다.

![긴급차 양보](docs/images/scene-emergency-vehicle.png)

| 들어간 기술 | 어디서 |
|---|---|
| 코리도 제한 RRT / RRT\* (실행 중 전환) | [4](#4-국부-경로-계획--rrt--rrt) |
| 회피 상태기 (감지 → 계획 → 이탈 → 복귀 → 합류) | [12](#12-국부-회피-상태기) |
| 경로 후처리 (단축 · 재샘플링 · 최대 조향각 검증) | [4](#4-국부-경로-계획--rrt--rrt) |
| 긴급차 우선: 최우측 갓길 대피 후 통과 확인 뒤 복귀 | [12](#12-국부-회피-상태기) |
| 계획 시간 / 최소 여유거리 실시간 계측 | [13](#13-로깅지표실험) |

**조작** `4` 낙하물 · `5` 긴급차 · `6` RRT↔RRT\* · `0` 리셋 · `Esc` 허브로

---

### IntegratedCity — 통합 10분 시나리오

![IntegratedCity 주행](docs/images/scene-integrated-city.png)

교차로 → 대로 → 순환로로 이어지는 통합 코스. 앞선 씬들의 요소가 **한 주행 안에 모두**
등장합니다. 도심 격자(`urban_*`)와 간선(`city_*`)이라는 성격이 다른 두 도로 계열이
한 씬에 공존하는 것이 특징입니다.

| 들어간 기술 | 어디서 |
|---|---|
| 앞의 모든 요소 + 갓길↔대로 테이퍼 합류 | [9](#9-합류-시간-슬롯-예약--중앙-관제의-대표-기술) |
| 이름이 아닌 위상으로 합류 지점을 찾아 예약 적용 | [9](#9-합류-시간-슬롯-예약--중앙-관제의-대표-기술) |
| 서로 다른 도로 계열을 가로지르는 회피 코리도 | [4](#4-국부-경로-계획--rrt--rrt) |
| 다차량 동시 주행 중 전역 상황 인지·충돌 예측 | [5](#5-충돌-예측) |

**조작** 시나리오 디렉터가 이벤트를 자동 진행 · `1` `2` `3` 카메라 · `Esc` 허브로

---

## 실행 방법

### 1. Python 서버

```bash
cd server
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python main.py --network scenarios/Urban_lanes.json
```

`--network`로 그 씬의 차선 export를 지정합니다. 생략하면 합성 네트워크로 뜨므로,
Unity 씬과 함께 돌릴 때는 반드시 지정하세요.

| 씬 | `--network` |
|---|---|
| LKA_Test | `scenarios/LKA_Test_lanes.json` |
| Highway | `scenarios/Highway_lanes.json` |
| Urban | `scenarios/Urban_lanes.json` |
| EmergencyAvoidance | `scenarios/EmergencyAvoidance_lanes.json` |
| IntegratedCity | `scenarios/IntegratedCity_lanes.json` |

### 2. Unity

1. `unity/` 폴더를 Unity 6로 엽니다.
2. `Main` 씬을 열고 Play — 허브에서 원하는 씬을 고릅니다.
3. 씬을 처음 만들거나 다시 만들려면 메뉴 **`V2X > Build All Demo Scenes`** (허브 하나만
   다시 만들려면 `V2X > Build Main Hub`).
4. 도로를 편집했다면 **`V2X > Export Lane Network...`** 로 반드시 다시 내보내세요.
   빠뜨리면 테스트가 잡아줍니다.

서버가 꺼져 있어도 허브와 씬은 뜨지만, 차량은 움직이지 않습니다(명령을 주는 쪽이
서버이므로).

---

## 테스트와 실험

```bash
python -m pytest server/tests -q     # 250여 건
```

일반적인 단위/통합 테스트 외에, 이 프로젝트가 스스로 꼽은 **1순위 리스크(양측 동기
이탈)** 를 겨냥한 검사가 따로 있습니다.

| 검사 | 무엇을 막나 |
|---|---|
| 씬 ↔ 차선 export 대조 | 씬을 고치고 재export를 잊어 서버가 없는 도로 위에서 계획하는 것 |
| Unity ↔ Python 신호 계획 대조 | 화면은 녹색인데 서버는 적색으로 붙잡는 상태 |
| 두 스키마의 scenario enum 동기 | Unity가 보고할 수 없는 시나리오가 생기는 것 |
| 차선 그래프 정적 감사 | 끊긴 참조, 비대칭 인접, 주행 불가능한 접합, 경로 점프 |
| 상충 접근로 동시 녹색 없음 (주기 전수 스윕) | 신호 계획 자체의 모순 |

실험 재현:

```bash
python experiments/run_algorithm_compare.py   # A* vs RRT vs RRT*
python experiments/run_lka_test.py            # Pure Pursuit vs Stanley
python experiments/make_charts.py             # -> experiments/results/charts/
```

| | |
|---|---|
| ![알고리즘 비교](experiments/results/charts/algo_compare.png) | ![LKA 횡오차](experiments/results/charts/lka_lateral_error.png) |

결과 해석은 [`docs/experiment_results.md`](docs/experiment_results.md)에 있습니다.

---

## 저장소 구조

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
│  ├─ main.py            WebSocket 서버 + 스키마 검증 + 동기 검사
│  ├─ central_control.py 매 틱 정책의 중심 (신호·좌회전·합류·회피 조율)
│  ├─ planners/          A*, RRT, RRT*, 회피 탐색공간
│  ├─ controllers/       ACC(종방향), lateral(횡방향)
│  ├─ scenarios/         씬별 차선 export (Unity가 내보낸 것)
│  ├─ tools/             fake Unity 클라이언트, Unity 씬 리더
│  └─ tests/             250여 건
├─ experiments/        실험 러너 + 분석 노트북 + 결과 CSV/차트
├─ shared/protocol/    JSON Schema — 와이어 포맷의 유일한 진실
└─ docs/               설계안, 프로토콜, 실험 결과, 일일 워크로그
```

---

## 문서

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

## Git 참고

- `.gitignore`가 Unity `Library/` · `Temp/` · 빌드 산출물과 Python `__pycache__/` ·
  `.venv/`를 제외합니다.
- Unity 바이너리 에셋은 **Git LFS**로 관리합니다(`.gitattributes`). 첫 커밋 전에
  `git lfs install`을 한 번 실행하세요.
