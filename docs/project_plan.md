# 중앙 관제형 V2V/V2X 기반 자율주행 시뮬레이션 프로젝트 설계안

> Unity 시뮬레이션 + Python 중앙 관제 서버 + 다중 차량 경로계획 + 고속도로/시내 비교 + LKA/ADAS 테스트

---

## 1. 프로젝트 개요

이 프로젝트는 **극도로 발달한 V2V/V2X 환경**을 가정한다. 중앙 관제 시스템은 모든 자동차, 보행자, 자전거, 장애물, 긴급차량 등 움직이는 오브젝트의 위치, 속도, 가속도, 진행 방향을 실시간으로 알고 있다. 돌발상황이 발생했을 때도 중앙 시스템이 즉시 인지한다고 가정한다.

따라서 일반 자율주행에서 큰 비중을 차지하는 카메라·라이다 기반 인지는 초기 범위에서 제외하고, 다음에 집중한다.

1. **중앙 관제 기반 전역 상황 인식**
2. **다중 차량 경로 탐색 및 충돌 회피**
3. **고속도로 환경과 시내 환경의 주행 전략 비교**
4. **A\*, RRT, RRT\* 등 경로 탐색 알고리즘 비교**
5. **LKA/ADAS 기능의 성능 테스트**
6. **보행자, 돌발 장애물, 긴급차량 등 이벤트 기반 확장**

Unity는 시각화와 차량 이동 시뮬레이션을 담당하고, Python 또는 C++ 기반 외부 모듈은 경로 탐색, 충돌 예측, 교통 제어, 차량 제어 명령 생성을 담당한다.

---

## 2. 프로젝트 핵심 컨셉

### 2.1 일반 자율주행과의 차이

일반적인 자율주행 시스템은 다음 흐름을 가진다.

```text
센서 입력 → 객체 인식 → 객체 추적 → 예측 → 판단 → 경로 계획 → 제어
```

이 프로젝트에서는 중앙 V2X 시스템이 이미 모든 객체 정보를 알고 있다고 가정하므로 흐름을 단순화한다.

```text
중앙 상태 수집 → 위험 예측 → 경로 계획/재계획 → 행동 결정 → 차량 제어 → Unity 반영
```

즉, 핵심 질문은 다음이다.

> 차량이 무엇을 볼 수 있는가가 아니라, 모든 정보를 알고 있을 때 어떻게 가장 안전하고 효율적으로 판단할 것인가?

---

## 3. 주요 연구 질문

### 3.1 중앙 관제형 자율주행

- 모든 차량의 위치와 속도를 알고 있다면, 개별 차량이 독립적으로 판단하는 방식보다 중앙 관제 방식이 얼마나 효율적인가?
- 다중 차량이 동시에 움직일 때 교차로, 합류부, 차선 변경 구간에서 충돌을 어떻게 방지할 수 있는가?
- 돌발 장애물이 생겼을 때 전체 차량 흐름을 어떻게 재계획할 것인가?

### 3.2 경로 탐색 알고리즘 비교

- 도로 그래프 기반 환경에서는 A\*가 가장 효율적인가?
- 비정형 장애물 회피나 차선 변경, 주차장, 도로 차단 상황에서는 RRT/RRT\*가 더 유리한가?
- RRT\*는 더 좋은 경로를 찾지만, 실시간 주행에 사용할 만큼 빠른가?

### 3.3 고속도로 vs 시내 비교

- 고속도로에서는 고속 주행, 차간 거리, 차선 유지, 차선 변경, 합류 제어가 핵심인가?
- 시내에서는 교차로, 신호등, 횡단보도, 보행자, 정차 차량, 돌발상황 대응이 더 중요한가?
- 같은 중앙 관제 시스템을 사용할 때, 두 환경에서 필요한 정책과 평가 지표는 어떻게 달라지는가?

### 3.4 LKA/ADAS 테스트

- Lane Keeping Assist는 차선 중앙 유지에 어느 정도 효과가 있는가?
- 곡선로, 고속 주행, 외란, 센서 노이즈가 있을 때 LKA의 안정성은 어떻게 달라지는가?
- 중앙 V2X 기반 완전 경로계획과 LKA 같은 저수준 ADAS 제어를 어떻게 결합할 수 있는가?

---

## 4. 전체 시스템 구조

```text
┌────────────────────────────────────────────────────┐
│                    Unity Simulation                 │
│                                                    │
│  - 도로, 차선, 교차로, 고속도로, 보행자 시각화       │
│  - 차량 이동 및 물리/키네마틱 시뮬레이션              │
│  - 차선 중심선, waypoint, 충돌 예상 지점 표시        │
│  - LKA 작동 상태, 경로 재계획, 위험도 UI 표시         │
└────────────────────────┬───────────────────────────┘
                         │ WebSocket / TCP / gRPC / REST
                         ▼
┌────────────────────────────────────────────────────┐
│                Central V2X Control Server           │
│                                                    │
│  - 모든 오브젝트 상태 수집                          │
│  - 월드 모델 관리                                   │
│  - 교통 시스템 관리                                 │
│  - 다중 차량 충돌 예측                              │
│  - 고속도로/시내 시나리오 정책 선택                  │
│  - 차량별 행동 결정                                 │
└────────────────────────┬───────────────────────────┘
                         │
                         ▼
┌────────────────────────────────────────────────────┐
│                Path Planning Module                 │
│                                                    │
│  - A* : 도로 그래프 기반 전역 경로 탐색              │
│  - RRT : 빠른 비정형 장애물 회피                    │
│  - RRT* : 더 좋은 경로 품질 비교                    │
│  - 추후 Hybrid A*, D* Lite, MPC 확장 가능            │
└────────────────────────┬───────────────────────────┘
                         │
                         ▼
┌────────────────────────────────────────────────────┐
│              Vehicle Control / ADAS Module          │
│                                                    │
│  - Longitudinal Control: 속도, 정지, 차간거리         │
│  - Lateral Control: 차선 중심 추종, 조향             │
│  - LKA / LDW / Lane Centering 테스트                 │
│  - Pure Pursuit, Stanley, PID, MPC 확장 가능          │
└────────────────────────────────────────────────────┘
```

---

## 5. 환경 모드 설계

이 프로젝트는 하나의 시스템으로 두 가지 주요 환경을 지원한다.

1. **고속도로 환경**
2. **일반 시내 환경**

두 환경은 같은 차량 모델과 중앙 관제 서버를 사용하지만, 도로 구조, 주요 위험 요소, 행동 정책, 평가 지표가 다르다.

---

## 6. 고속도로 환경 설계

### 6.1 고속도로 환경의 특징

고속도로는 시내보다 구조가 단순하지만, 차량 속도가 높기 때문에 작은 판단 지연도 큰 위험으로 이어질 수 있다.

주요 특징은 다음과 같다.

```text
- 고속 주행
- 차선 유지 중요
- 차간 거리 유지 중요
- 차선 변경 위험도 높음
- 진입 램프/출구 램프/합류 구간 존재
- 정지보다는 감속과 회피가 중요
- 보행자는 기본적으로 없음
```

### 6.2 고속도로 주요 시나리오

| 시나리오 | 설명 | 핵심 기능 |
|---|---|---|
| 기본 차선 주행 | 차량이 차선 중심을 따라 고속 주행 | LKA, 차선 중심 추종 |
| 선행 차량 추종 | 앞차 속도에 맞춰 감속/가속 | ACC 유사 제어 |
| 차선 변경 | 느린 차량을 추월하거나 목적지 차선으로 이동 | 안전 간격 판단, 경로 생성 |
| 합류 구간 | 진입 램프 차량이 본선에 합류 | 중앙 관제 기반 속도 조정 |
| 급정거 차량 발생 | 앞차가 갑자기 멈춤 | 긴급 감속, 주변 차량 연쇄 제어 |
| 낙하물 발생 | 차선 위 장애물이 갑자기 생성 | 회피, 재계획, 차선 폐쇄 처리 |
| 긴급차량 통과 | 구급차가 빠르게 이동 | 우선순위 부여, 차선 양보 |

### 6.3 고속도로에서 중요한 알고리즘

고속도로에서는 전역 경로 탐색보다 **차선 유지, 속도 제어, 차선 변경 판단**이 중요하다.

추천 구성은 다음과 같다.

```text
전역 경로: A* on lane graph
국소 경로: 차선 중심선 spline 추종
차선 변경: gap acceptance + collision prediction
돌발 장애물 회피: RRT/RRT* 또는 차선 그래프 재탐색
차량 제어: LKA + ACC 유사 제어
```

### 6.4 고속도로 평가 지표

| 지표 | 의미 |
|---|---|
| 평균 주행 속도 | 전체 교통 흐름 효율 |
| 평균 차간 거리 | 안전성 |
| TTC(Time To Collision) 최소값 | 충돌 위험도 |
| 차선 이탈 횟수 | LKA 성능 |
| 차선 변경 성공률 | 판단 안정성 |
| 급제동 횟수 | 주행 부드러움 |
| 합류 성공률 | 중앙 관제 성능 |
| 경로 재계획 시간 | 돌발상황 대응성 |

---

## 7. 일반 시내 환경 설계

### 7.1 시내 환경의 특징

시내는 고속도로보다 속도는 낮지만, 상호작용 대상이 훨씬 많고 상황이 복잡하다.

주요 특징은 다음과 같다.

```text
- 교차로가 많음
- 신호등, 정지선, 횡단보도 존재
- 보행자와 자전거 존재
- 불법 정차 차량 또는 도로 공사 가능
- 저속이지만 판단 경우의 수가 많음
- 경로 재탐색 빈도가 높음
```

### 7.2 시내 주요 시나리오

| 시나리오 | 설명 | 핵심 기능 |
|---|---|---|
| 신호등 교차로 통과 | 신호에 따라 정지/진행 | 신호 시스템, 정지선 제어 |
| 무신호 교차로 통과 | 중앙 관제가 진입 순서를 예약 | 교차로 reservation |
| 보행자 횡단 | 보행자가 횡단보도로 이동 | 보행자 예측, 정지 판단 |
| 갑작스런 무단횡단 | 보행자가 도로에 갑자기 진입 | 긴급 정지, 주변 차량 연쇄 제어 |
| 정차 차량 회피 | 도로 가장자리에 멈춘 차량 회피 | 차선 변경, 국소 경로계획 |
| 도로 공사 구간 | 일부 차선 차단 | 재계획, 우회 경로 |
| 자전거 추월 | 느린 이동체를 안전하게 추월 | 측면 간격 유지 |
| 긴급차량 접근 | 긴급차량이 교차로로 진입 | 우선순위 재조정 |

### 7.3 시내에서 중요한 알고리즘

시내에서는 전역 경로와 교통 규칙 처리가 중요하다.

추천 구성은 다음과 같다.

```text
전역 경로: A* on road graph
교차로 제어: traffic light 또는 reservation-based intersection
보행자 대응: predicted trajectory + stop/yield decision
장애물 회피: A* 재탐색 또는 RRT/RRT*
차량 제어: 저속 waypoint 추종 + 정지선 정밀 제어
```

### 7.4 시내 평가 지표

| 지표 | 의미 |
|---|---|
| 목적지 도달 시간 | 경로 효율 |
| 교차로 대기 시간 | 교통 시스템 효율 |
| 충돌/근접 충돌 횟수 | 안전성 |
| 보행자 위험 상황 수 | 보행자 대응 성능 |
| 신호 위반 횟수 | 교통 규칙 준수 |
| 재계획 횟수 | 환경 복잡도 대응 |
| 차량별 평균 지연 시간 | 다중 차량 공정성 |
| 전체 throughput | 단위 시간당 통과 차량 수 |

---

## 8. 고속도로와 시내 비교 실험 설계

### 8.1 비교 목적

고속도로와 시내는 같은 자율주행 시스템을 사용하더라도 병목이 다르다.

```text
고속도로: 속도, 차간 거리, 차선 유지, 합류 안정성
시내: 교차로, 보행자, 정지/출발, 경로 재계획
```

따라서 동일한 중앙 관제 시스템이 두 환경에서 어떻게 다른 정책을 사용해야 하는지 비교한다.

### 8.2 비교 항목

| 항목 | 고속도로 | 시내 |
|---|---|---|
| 주요 위험 | 고속 충돌, 급정거, 합류 실패 | 보행자, 교차로 충돌, 신호 위반 |
| 핵심 제어 | LKA, ACC, 차선 변경 | 정지/출발, 양보, 교차로 예약 |
| 주요 경로 탐색 | 차선 그래프 기반 A* | 도로 그래프 기반 A* |
| RRT/RRT* 활용 | 장애물 회피, 차선 변경 궤적 | 도로 공사, 정차 차량 회피 |
| 중앙 관제 역할 | 속도 조정, 차선 변경 승인 | 교차로 통과 순서, 보행자 보호 |
| 평가 지표 | TTC, 차선 이탈, 평균 속도 | 대기 시간, 보행자 안전, throughput |

### 8.3 추천 비교 실험

#### 실험 A: 동일 차량 수에서 환경별 교통 흐름 비교

```text
조건:
- 차량 20대
- 고속도로 맵 1개
- 시내 맵 1개
- 모든 차량은 랜덤 목적지로 이동

측정:
- 평균 도착 시간
- 평균 속도
- 충돌 위험 이벤트 수
- 재계획 횟수
```

#### 실험 B: 돌발상황 발생 시 대응 비교

```text
조건:
- 고속도로: 낙하물이 1개 차선을 막음
- 시내: 보행자가 갑자기 횡단

측정:
- 최초 위험 감지부터 차량 제어 명령까지 걸린 시간
- 충돌 회피 성공 여부
- 주변 차량의 연쇄 감속 정도
- 전체 교통 회복 시간
```

#### 실험 C: 중앙 관제 유무 비교

```text
조건:
- Local-only: 각 차량이 자신 기준으로만 판단
- Centralized V2X: 중앙 서버가 모든 차량을 조정

측정:
- 충돌/근접 충돌 횟수
- 평균 대기 시간
- 전체 통과량
- 교차로/합류부 병목 해소 정도
```

---

## 9. LKA/ADAS 모듈 설계

### 9.1 LKA의 역할

LKA는 Lane Keeping Assist, 즉 차선 유지 보조 기능이다. 이 프로젝트에서는 LKA를 완전 자율주행의 대체물이 아니라, **저수준 횡방향 제어 모듈**로 취급한다.

중앙 관제 서버가 “어느 차선을 따라가라” 또는 “이 차선으로 변경하라”를 결정하면, LKA/차선 추종 모듈은 실제 차량이 차선 중심을 안정적으로 따라가도록 조향을 계산한다.

```text
중앙 관제 판단: 어느 경로/차선으로 갈 것인가?
LKA/ADAS 제어: 선택된 차선 중심을 얼마나 잘 따라갈 것인가?
```

### 9.2 ADAS 기능 구분

프로젝트에서는 ADAS를 다음 세 단계로 나누어 실험할 수 있다.

| 기능 | 설명 | 구현 우선순위 |
|---|---|---|
| LDW | Lane Departure Warning. 차선을 벗어나면 경고만 표시 | 낮음 |
| LKA | 차선을 벗어나려 할 때 조향 보정 | 높음 |
| Lane Centering | 계속 차선 중앙을 유지하도록 조향 | 높음 |
| ACC 유사 기능 | 앞차와 안전거리를 유지하며 속도 조절 | 중간 |
| AEB 유사 기능 | 충돌 위험 시 긴급 제동 | 중간 |

초기 구현에서는 **Lane Centering + LKA + 간단한 ACC**까지만 구현하는 것을 추천한다.

---

## 10. LKA 구현 모델

### 10.1 차선 모델

Unity 도로의 각 차선은 중심선으로 표현한다.

```text
Lane
- lane_id
- centerline waypoints
- width
- speed_limit
- left_lane_id
- right_lane_id
- next_lane_ids
```

차량은 현재 위치에서 가장 가까운 차선 중심선 점을 찾고, 다음 값을 계산한다.

```text
lateral_error: 차선 중심선에서 차량이 좌우로 벗어난 거리
heading_error: 차선 방향과 차량 진행 방향의 각도 차이
curvature: 앞으로 따라갈 차선의 곡률
```

### 10.2 제어 방식 후보

#### 1. Pure Pursuit

차량 앞쪽의 lookahead point를 따라가도록 조향한다.

장점:

```text
- 구현 쉬움
- waypoint 기반 주행과 잘 맞음
- Unity 시뮬레이션에서 안정적
```

단점:

```text
- 고속 주행에서 lookahead distance 튜닝 필요
- 곡선로에서 오차가 커질 수 있음
```

#### 2. Stanley Controller

횡방향 오차와 heading error를 이용해 조향을 계산한다.

장점:

```text
- 차선 중심 유지 실험에 적합
- LKA 느낌을 내기 좋음
```

단점:

```text
- 속도와 gain 튜닝이 필요
```

#### 3. PID 기반 보정

차선 중앙에서 벗어난 정도를 기준으로 조향을 보정한다.

장점:

```text
- 가장 단순함
- 실험용으로 빠르게 구현 가능
```

단점:

```text
- 복잡한 곡선로에서는 성능 한계
```

#### 4. MPC 확장

차량 동역학과 미래 경로를 함께 고려해 조향과 속도를 최적화한다.

장점:

```text
- 가장 정교함
- 고속도로 LKA/ACC 통합 실험에 적합
```

단점:

```text
- 구현 난이도 높음
- 초기 MVP에는 과함
```

### 10.3 초기 추천

초기에는 다음 순서가 좋다.

```text
1. Pure Pursuit로 waypoint 추종 구현
2. PID로 차선 중앙 보정 추가
3. Stanley Controller로 LKA 성능 개선
4. 시간이 남으면 MPC 비교 실험 추가
```

---

## 11. LKA 테스트 시나리오

### 11.1 기본 차선 유지 테스트

```text
환경:
- 직선 고속도로
- 차량 1대
- 일정 속도 유지

측정:
- lateral error 평균/RMS/최대값
- 차선 이탈 횟수
- 조향 변화량
```

### 11.2 곡선로 차선 유지 테스트

```text
환경:
- 완만한 곡선 고속도로
- 여러 속도 조건: 40, 60, 80, 100 km/h

측정:
- 속도별 lateral error
- 곡률별 차선 이탈 가능성
- 조향 안정성
```

### 11.3 외란 테스트

```text
환경:
- 차량에 순간적인 횡방향 힘 또는 위치 오차 부여
- 바람, 노면 미끄러짐, 조향 지연을 단순 모델로 표현

측정:
- 차선 중앙 복귀 시간
- 최대 lateral error
- overshoot 여부
```

### 11.4 센서 노이즈 테스트

V2X 환경에서는 모든 위치 정보를 정확히 안다고 가정하지만, 현실성을 위해 노이즈 모드를 선택적으로 넣을 수 있다.

```text
Full V2X Mode:
- 위치/속도 정보 정확

Noisy V2X Mode:
- 위치에 작은 Gaussian noise 추가
- 속도 추정 오차 추가

Local ADAS Mode:
- 차량 주변 일부 정보만 사용
```

측정:

```text
- 노이즈 크기에 따른 LKA 안정성
- 차선 이탈 횟수 변화
- 조향 명령의 진동 정도
```

---

## 12. 다중 차량 시스템 설계

### 12.1 차량 상태 데이터

모든 차량은 다음 상태를 가진다.

```json
{
  "id": "vehicle_001",
  "type": "car",
  "position": [10.5, 0.0, 42.2],
  "velocity": [0.0, 0.0, 15.0],
  "acceleration": [0.0, 0.0, 0.0],
  "heading": 90.0,
  "current_lane": "lane_12",
  "target_lane": "lane_15",
  "route": ["lane_12", "lane_13", "lane_15"],
  "behavior_state": "LaneKeeping"
}
```

### 12.2 행동 상태

각 차량은 단순한 finite state machine으로 제어할 수 있다.

```text
Idle
→ RoutePlanning
→ LaneKeeping
→ Following
→ LaneChanging
→ Stopping
→ WaitingAtIntersection
→ EmergencyBraking
→ Replanning
→ Arrived
```

### 12.3 충돌 예측

중앙 서버는 각 차량의 예상 경로를 시간축으로 샘플링한다.

```text
prediction_horizon = 3~5 seconds
sample_interval = 0.1~0.2 seconds
```

예측 결과를 기반으로 차량 간 거리, TTC, 예상 충돌 지점을 계산한다.

```text
if distance(vehicle_A[t], vehicle_B[t]) < safety_distance:
    conflict_detected = True
```

### 12.4 충돌 해결 방법

| 상황 | 해결 방법 |
|---|---|
| 같은 차선에서 앞차가 느림 | 뒤차 감속 |
| 교차로에서 경로가 겹침 | 진입 순서 예약 |
| 고속도로 합류 | 본선 차량 감속 또는 합류 차량 대기 |
| 차선 변경 중 위험 | 차선 변경 취소 또는 지연 |
| 장애물 발생 | 차선 폐쇄 후 재계획 |
| 긴급차량 접근 | 우선순위 상승, 주변 차량 양보 |

---

## 13. 교통 시스템 설계

### 13.1 신호등 기반 시스템

시내 환경의 기본 모드로 사용할 수 있다.

```text
- 신호등 상태: Red / Yellow / Green
- 정지선 위치
- 차량은 정지선 앞에서 감속/정지
- 보행자 신호와 연동 가능
```

### 13.2 중앙 관제 기반 무신호 교차로

이 프로젝트의 핵심 컨셉과 잘 맞는 방식이다.

```text
1. 차량이 교차로 접근
2. 중앙 서버가 예상 도착 시간 계산
3. 교차로 내부 conflict zone 확인
4. 차량별 통과 시간 예약
5. 예약된 시간에 맞춰 감속/가속 명령
```

장점:

```text
- V2X 세계관에 잘 맞음
- 신호등보다 효율적인 통과 가능
- 다중 차량 제어를 보여주기 좋음
```

### 13.3 고속도로 합류 예약 시스템

고속도로에서는 교차로 대신 합류부가 핵심이다.

```text
1. 램프 차량이 본선 진입 요청
2. 중앙 서버가 본선 차량들의 위치/속도 확인
3. 안전 gap을 생성하거나 선택
4. 본선 차량 감속 또는 램프 차량 가속 명령
5. 합류 완료 후 정상 속도 복귀
```

---

## 14. 보행자와 돌발상황 확장

### 14.1 MovingObject 일반화

보행자와 돌발 장애물을 나중에 쉽게 추가하려면 모든 동적 객체를 공통 구조로 관리한다.

```text
MovingObject
├─ Vehicle
├─ Pedestrian
├─ Bicycle
├─ EmergencyVehicle
├─ StaticObstacle
└─ UnexpectedObstacle
```

공통 속성은 다음과 같다.

```json
{
  "id": "object_001",
  "object_type": "pedestrian",
  "position": [0.0, 0.0, 0.0],
  "velocity": [1.2, 0.0, 0.0],
  "radius": 0.4,
  "predicted_trajectory": []
}
```

### 14.2 돌발 이벤트 예시

| 이벤트 | 설명 | 대응 |
|---|---|---|
| PedestrianSuddenCrossing | 보행자가 갑자기 도로 진입 | 긴급 정지, 주변 차량 감속 |
| VehicleBreakdown | 차량이 도로 위에서 멈춤 | 차선 폐쇄, 우회 재계획 |
| FallingObject | 고속도로에 낙하물 생성 | 차선 변경, 감속, 재계획 |
| EmergencyVehicle | 긴급차량 접근 | 우선순위 부여, 차선 양보 |
| ConstructionZone | 도로 공사 구역 생성 | 도로 그래프 업데이트 |

### 14.3 이벤트 처리 흐름

```text
이벤트 발생
→ 중앙 월드 모델 업데이트
→ 위험 대상 차량 탐색
→ 경로 재계획 또는 제동 명령
→ 주변 차량 연쇄 영향 계산
→ Unity에서 결과 시각화
```

---

## 15. 경로 탐색 알고리즘 비교 설계

### 15.1 A\*

A\*는 도로 그래프 기반 경로 탐색에 적합하다.

사용 위치:

```text
- 시내 목적지 경로 탐색
- 고속도로 출구/진입 경로 탐색
- 차선 그래프 기반 최단 경로
- 도로 폐쇄 후 재탐색
```

장점:

```text
- 빠름
- 구현 쉬움
- 도로망에 잘 맞음
- 여러 차량에 적용하기 좋음
```

단점:

```text
- 연속 공간의 부드러운 경로 생성에는 한계
- 장애물이 도로 위에 복잡하게 배치되면 그래프 설계가 중요
```

### 15.2 RRT

RRT는 연속 공간에서 빠르게 feasible path를 찾는 데 적합하다.

사용 위치:

```text
- 도로 위 장애물 회피
- 주차장/공터/비정형 공간
- 차선 변경 후보 궤적 생성
```

장점:

```text
- 복잡한 장애물 환경에서 경로를 찾을 수 있음
- 연속 공간 처리에 적합
```

단점:

```text
- 결과 경로가 들쭉날쭉할 수 있음
- 도로 그래프만 있는 환경에서는 A*보다 불필요하게 무거울 수 있음
```

### 15.3 RRT\*

RRT\*는 RRT보다 더 나은 경로 품질을 얻기 위한 알고리즘이다.

사용 위치:

```text
- 경로 품질 비교
- 차선 변경 궤적 최적화
- 장애물 회피 경로 smoothing 비교
```

장점:

```text
- 시간이 충분하면 더 좋은 경로를 찾을 수 있음
- 경로 길이와 안전 거리 측면에서 비교 가치가 있음
```

단점:

```text
- RRT보다 계산량이 큼
- 실시간 다중 차량 환경에서는 제한적으로 사용해야 함
```

### 15.4 알고리즘별 추천 사용처

| 상황 | 추천 알고리즘 |
|---|---|
| 일반 도로 최단 경로 | A\* |
| 고속도로 출구까지 경로 | A\* |
| 시내 도로 폐쇄 후 우회 | A\* 또는 D\* Lite 확장 |
| 정차 차량 회피 | RRT/RRT\* |
| 고속도로 낙하물 회피 | RRT 또는 차선 그래프 재탐색 |
| 주차장/비정형 공간 | RRT/RRT\* |
| 다중 차량 전체 경로 조정 | A\* + 시간 예약 + 우선순위 제어 |

---

## 16. Unity 구현 설계

### 16.1 Unity 씬 구성

```text
Scenes/
├─ HighwayScene.unity
├─ UrbanScene.unity
├─ LKATestTrack.unity
└─ StressTestScene.unity
```

### 16.2 주요 GameObject

```text
SimulationManager
CentralServerClient
RoadNetworkManager
LaneManager
TrafficLightManager
VehicleSpawner
PedestrianSpawner
EventManager
VehicleAgent
VehicleController
LKAController
PathVisualizer
DebugUI
```

### 16.3 차량 이동 방식

초기에는 Rigidbody 기반 복잡한 차량 물리보다, **Kinematic bicycle model** 또는 waypoint 기반 이동을 추천한다.

초기 구현:

```text
- 차량 위치를 waypoint 방향으로 이동
- 속도와 가속도 제한 적용
- 조향각은 시각적 회전용으로 계산
```

확장 구현:

```text
- Kinematic bicycle model
- wheel collider 기반 차량 물리
- 타이어 마찰, 조향 지연, 제동 거리 반영
```

---

## 17. Python 서버 구현 설계

### 17.1 서버 역할

```text
- Unity에서 차량/보행자 상태 수신
- 월드 모델 업데이트
- 차량별 경로 계획
- 충돌 예측
- 교차로/합류 예약
- LKA/ADAS 명령 또는 목표 차선 전달
- Unity로 제어 명령 송신
```

### 17.2 추천 기술 스택

| 목적 | 후보 |
|---|---|
| Unity-Python 통신 | WebSocket, TCP socket, gRPC |
| 빠른 프로토타입 | Python + FastAPI/WebSocket |
| 경로 탐색 직접 구현 | PythonRobotics 참고 |
| Sampling-based planning | OMPL Python binding 또는 직접 구현 |
| 로봇 시스템 확장 | ROS 2 + Unity 연동 |
| 학습 기반 확장 | Unity ML-Agents |
| 데이터 분석 | pandas, numpy, matplotlib |

초기에는 ROS 2 없이 **Python WebSocket 서버**로 시작하는 것이 가장 단순하다. 이후 시스템이 커지면 ROS 2로 확장할 수 있다.

---

## 18. 통신 메시지 설계

### 18.1 Unity → Python 상태 전송

```json
{
  "time": 12.35,
  "vehicles": [
    {
      "id": "car_01",
      "position": [10.0, 0.0, 20.0],
      "velocity": [0.0, 0.0, 15.0],
      "heading": 90.0,
      "current_lane": "lane_01"
    }
  ],
  "objects": [
    {
      "id": "ped_01",
      "type": "pedestrian",
      "position": [15.0, 0.0, 35.0],
      "velocity": [1.0, 0.0, 0.0]
    }
  ],
  "events": []
}
```

### 18.2 Python → Unity 제어 명령

```json
{
  "time": 12.40,
  "commands": [
    {
      "vehicle_id": "car_01",
      "target_speed": 13.0,
      "target_lane": "lane_01",
      "behavior": "LaneKeeping",
      "path": [[10.0,0.0,20.0], [10.0,0.0,30.0], [10.0,0.0,40.0]],
      "lka_enabled": true
    }
  ]
}
```

---

## 19. 개발 단계 제안

### Phase 1. 기본 도로/차량 시뮬레이션

목표:

```text
- Unity에서 차량이 waypoint를 따라 이동
- 도로 그래프와 차선 데이터 구축
- 차량 1대가 목적지까지 이동
```

결과물:

```text
- HighwayScene 또는 UrbanScene 기본 버전
- VehicleController
- RoadNetworkManager
```

### Phase 2. Python A\* 경로 탐색 연동

목표:

```text
- Unity가 출발지/목적지를 Python에 전달
- Python이 A* 경로 계산
- Unity 차량이 반환된 경로를 따라 이동
```

결과물:

```text
- Python path planning server
- Unity-Python 통신
- A* route visualization
```

### Phase 3. 다중 차량 주행

목표:

```text
- 차량 여러 대 생성
- 각 차량의 경로 계산
- 같은 차선에서 앞차 추종
- 기본 충돌 방지
```

결과물:

```text
- MultiVehicleManager
- basic collision prediction
- following behavior
```

### Phase 4. LKA/ADAS 테스트 트랙

목표:

```text
- 차선 중심선 기반 lateral error 계산
- Pure Pursuit 또는 Stanley Controller 구현
- 직선/곡선/외란/노이즈 테스트
```

결과물:

```text
- LKATestTrack
- LKAController
- lateral error graph
- lane departure count
```

### Phase 5. 고속도로 시나리오

목표:

```text
- 고속도로 맵 구축
- 차선 변경, 합류, 급정거, 낙하물 이벤트 구현
- LKA + ACC 유사 제어 결합
```

결과물:

```text
- HighwayScene
- merge reservation system
- lane change behavior
- highway metrics dashboard
```

### Phase 6. 시내 시나리오

목표:

```text
- 시내 도로와 교차로 구성
- 신호등/무신호 교차로 구현
- 보행자와 횡단보도 추가
- 돌발 무단횡단 이벤트 구현
```

결과물:

```text
- UrbanScene
- TrafficLightManager
- IntersectionReservationManager
- PedestrianSpawner
```

### Phase 7. 알고리즘 비교 실험

목표:

```text
- A*, RRT, RRT* 비교
- 고속도로/시내 각각에서 실험
- 계산 시간, 경로 길이, 안전성, 재계획 성능 측정
```

결과물:

```text
- experiment runner
- csv log output
- comparison charts
- final report
```

---

## 20. 최종 실험 구성

### 20.1 실험 1: A\* vs RRT vs RRT\*

| 조건 | 내용 |
|---|---|
| 환경 | 시내 도로 + 장애물 |
| 차량 수 | 1대, 5대, 20대 |
| 비교 | 계산 시간, 경로 길이, 재계획 성공률 |
| 예상 결과 | 도로 그래프에서는 A\* 우세, 비정형 장애물에서는 RRT/RRT\* 유리 가능 |

### 20.2 실험 2: 고속도로 LKA 성능

| 조건 | 내용 |
|---|---|
| 환경 | 직선/곡선 고속도로 |
| 속도 | 40, 60, 80, 100 km/h |
| 비교 | LKA OFF / Pure Pursuit / Stanley |
| 측정 | lateral error, 차선 이탈, 조향 안정성 |

### 20.3 실험 3: 고속도로 합류 제어

| 조건 | 내용 |
|---|---|
| 환경 | 본선 + 진입 램프 |
| 차량 수 | 10대, 30대, 50대 |
| 비교 | 중앙 관제 없음 / 중앙 관제 있음 |
| 측정 | 합류 성공률, 급제동 횟수, 평균 속도 |

### 20.4 실험 4: 시내 교차로 제어

| 조건 | 내용 |
|---|---|
| 환경 | 4방향 교차로 |
| 비교 | 신호등 방식 / 중앙 reservation 방식 |
| 측정 | 대기 시간, throughput, 충돌 위험 이벤트 |

### 20.5 실험 5: 돌발상황 대응

| 조건 | 내용 |
|---|---|
| 고속도로 | 낙하물 또는 급정거 차량 발생 |
| 시내 | 보행자 돌발 횡단 |
| 측정 | 반응 시간, 충돌 회피 성공률, 교통 회복 시간 |

---

## 21. 데이터 로깅 설계

### 21.1 로그 항목

```csv
time,vehicle_id,scenario,position_x,position_z,speed,lane_id,behavior_state,lateral_error,heading_error,target_speed,collision_risk,ttc,event_type
```

### 21.2 분석 지표

```text
- 평균 속도
- 평균 도착 시간
- 평균/최대 lateral error
- 차선 이탈 횟수
- 급제동 횟수
- TTC 최소값
- 충돌 위험 이벤트 수
- 재계획 횟수
- 경로 계산 시간
- 전체 throughput
```

### 21.3 시각화

```text
- 시간에 따른 차량 속도 그래프
- lateral error 그래프
- 차선 변경 성공/실패 비율
- 고속도로/시내 throughput 비교
- 알고리즘별 경로 길이/계산 시간 비교
- 돌발상황 발생 후 교통 회복 곡선
```

---

## 22. 추천 폴더 구조

```text
autonomous-v2x-sim/
├─ unity/
│  ├─ Assets/
│  │  ├─ Scenes/
│  │  │  ├─ HighwayScene.unity
│  │  │  ├─ UrbanScene.unity
│  │  │  └─ LKATestTrack.unity
│  │  ├─ Scripts/
│  │  │  ├─ Vehicle/
│  │  │  │  ├─ VehicleAgent.cs
│  │  │  │  ├─ VehicleController.cs
│  │  │  │  └─ LKAController.cs
│  │  │  ├─ Road/
│  │  │  │  ├─ RoadNetworkManager.cs
│  │  │  │  └─ Lane.cs
│  │  │  ├─ Traffic/
│  │  │  │  ├─ TrafficLightManager.cs
│  │  │  │  └─ IntersectionManager.cs
│  │  │  ├─ Communication/
│  │  │  │  └─ V2XClient.cs
│  │  │  └─ UI/
│  │  │     └─ DebugDashboard.cs
│  │  └─ Prefabs/
│  │     ├─ Vehicle.prefab
│  │     ├─ Pedestrian.prefab
│  │     └─ Obstacle.prefab
│  └─ ProjectSettings/
│
├─ server/
│  ├─ main.py
│  ├─ world_model.py
│  ├─ vehicle_state.py
│  ├─ traffic_manager.py
│  ├─ collision_predictor.py
│  ├─ planners/
│  │  ├─ astar.py
│  │  ├─ rrt.py
│  │  └─ rrt_star.py
│  ├─ controllers/
│  │  ├─ lka.py
│  │  ├─ acc.py
│  │  └─ behavior_planner.py
│  ├─ scenarios/
│  │  ├─ highway.py
│  │  ├─ urban.py
│  │  └─ lka_test.py
│  └─ logs/
│
├─ experiments/
│  ├─ run_highway_lka.py
│  ├─ run_city_intersection.py
│  ├─ run_algorithm_compare.py
│  └─ analysis.ipynb
│
├─ docs/
│  ├─ project_plan.md
│  ├─ api_protocol.md
│  └─ experiment_results.md
│
└─ README.md
```

---

## 23. 구현 난이도와 우선순위

| 기능 | 난이도 | 우선순위 | 비고 |
|---|---:|---:|---|
| Unity waypoint 차량 이동 | 낮음 | 매우 높음 | 가장 먼저 구현 |
| 도로/차선 그래프 | 중간 | 매우 높음 | A\* 기반 |
| Unity-Python 통신 | 중간 | 매우 높음 | WebSocket 추천 |
| A\* 경로 탐색 | 낮음~중간 | 매우 높음 | MVP 핵심 |
| 차량 여러 대 주행 | 중간 | 높음 | 프로젝트 완성도 상승 |
| 기본 충돌 예측 | 중간 | 높음 | TTC/거리 기반 |
| LKA 제어 | 중간 | 높음 | ADAS 실험 핵심 |
| 고속도로 합류 | 중간~높음 | 중간 | 중앙 관제 장점 표현 |
| 시내 교차로 | 중간~높음 | 중간 | traffic system 표현 |
| 보행자 | 중간 | 중간 | 시내 확장 |
| RRT/RRT\* | 중간~높음 | 중간 | 비교 실험용 |
| MPC | 높음 | 낮음 | 후반 확장 |
| 카메라 인식 | 높음 | 낮음 | 초기 제외 추천 |

---

## 24. 최종 프로젝트 제목 후보

### 한글 제목

- V2X 기반 중앙 관제형 다중 자율주행 시뮬레이션
- 고속도로/시내 환경 비교를 위한 중앙 관제형 자율주행 시스템
- LKA/ADAS와 경로탐색 알고리즘을 포함한 Unity 기반 자율주행 시뮬레이터
- 다중 차량 V2X 자율주행 환경에서의 경로계획 및 차선 유지 제어 비교

### 영어 제목

- Centralized Multi-Agent Autonomous Driving Simulation with V2X-based Global Awareness
- Unity-based V2X Autonomous Driving Simulation for Highway and Urban Scenarios
- Comparative Study of Path Planning and Lane Keeping Assistance in a Centralized V2X Driving System
- Multi-Vehicle Autonomous Driving Simulation with A*, RRT, RRT*, and LKA/ADAS Evaluation

---

## 25. 최종 개발 방향 요약

이 프로젝트는 다음 순서로 개발하는 것이 가장 안정적이다.

```text
1. Unity에서 차량 1대가 waypoint를 따라 주행
2. Python A* 서버와 연동
3. 차량 여러 대를 중앙 관제 방식으로 제어
4. 고속도로 맵에서 LKA/ACC/차선 변경 구현
5. 시내 맵에서 교차로/신호등/보행자 구현
6. 돌발상황 발생 시 재계획 구현
7. A*, RRT, RRT* 비교 실험
8. 고속도로 vs 시내 성능 비교 보고서 작성
```

초기 MVP는 다음까지만 잡아도 충분히 좋다.

```text
- Unity 도로/차선/차량
- Python A* 경로 탐색
- 차량 여러 대 동시 주행
- 기본 충돌 예측
- LKA 차선 유지 테스트
- 고속도로 직선/곡선 테스트
```

이후 확장으로 다음을 추가한다.

```text
- 고속도로 합류/낙하물
- 시내 교차로/보행자
- RRT/RRT* 장애물 회피
- 중앙 관제 유무 비교
- 실험 결과 그래프와 보고서
```

---

## 26. 참고 자료

- NHTSA, Driver Assistance Technologies: Lane Keeping Assistance, Lane Departure Warning, other ADAS descriptions.  
  https://www.nhtsa.gov/vehicle-safety/driver-assistance-technologies
- SAE International, SAE Levels of Driving Automation.  
  https://www.sae.org/news/blog/sae-levels-driving-automation-clarity-refinements
- OMPL, The Open Motion Planning Library.  
  https://ompl.kavrakilab.org/
- OMPL Available Planners.  
  https://ompl.kavrakilab.org/planners.html
- Unity ML-Agents Toolkit Documentation.  
  https://unity-technologies.github.io/ml-agents/ML-Agents-Toolkit-Documentation/
- Unity, ROS 2 and Unity robotics simulation support.  
  https://unity.com/blog/engine-platform/advance-your-robot-autonomy-with-ros-2-and-unity
- PythonRobotics GitHub Repository.  
  https://github.com/AtsushiSakai/PythonRobotics

