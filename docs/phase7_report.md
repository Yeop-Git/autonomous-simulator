# Phase 7 구현 보고서 — 경로탐색 알고리즘 비교 (A\* / RRT / RRT\*)

- **작성일**: 2026-07-02
- **대상**: 중앙 관제형 V2X 자율주행 시뮬레이터, `IMPLEMENTATION_PLAN.md` Phase 7
- **범위**: 이번 작업에서 신규 구현·검증한 서버측 산출물 전부
- **검증 상태**: 서버 전체 테스트 **105 passed** (기존 92 + 신규 13), 실험 러너·차트·노트북 실행 확인 완료

---

## 1. 개요

프로젝트의 마지막 미구현 단계였던 **Phase 7(알고리즘 비교 실험 + 보고서)** 을 완성했다.
직전 상태는 샘플링 플래너 토대(`server/planners/_rrt_common.py`)만 존재하고 어디에도 import되지
않은 상태였다. 이번 작업으로 다음을 채웠다.

1. **RRT / RRT\* 플래너** — A\*와 동일한 `plan(start, goal, world)` 인터페이스, 시드 고정 결정적.
2. **단위 테스트** — `tests/test_rrt.py` 13개.
3. **알고리즘 비교 실험 러너** — A\* vs RRT vs RRT\*, 다중 시드·다중 차량 수, CSV 로깅(plan §20.1).
4. **시각화** — headless 차트 생성기 + 실행검증된 분석 노트북(plan §21.3).
5. **결과 보고서** — `docs/experiment_results.md`.

Phase 0–6(월드모델·A\*·충돌예측·행동FSM·LKA/ACC·고속도로/시내 로직)은 이전에 이미 구현·테스트된
상태였으며, 이번 보고서의 구현 대상은 아니다. 본 문서는 Phase 7 산출물을 중심으로 하되 전체
시스템 안에서의 위치를 함께 기술한다.

---

## 2. 배경 — 왜 RRT/RRT\*인가 (plan §15)

A\*는 **도로 그래프(차선 네트워크)** 위의 최단 경로에 최적이지만, 다음 두 상황에서 한계가 있다.

- 차선이 장애물로 막혔는데 **대체 엣지가 없을 때** → 경로 실패.
- 도로 위 **비정형 장애물 / 주차장 / 자유공간** 우회 → 그래프 자체가 없음.

이때 연속 공간에서 무작위 트리를 성장시키는 **RRT**(빠른 feasible path)와, 재배선으로 품질을
높인 **RRT\***(더 짧고 매끄러운 경로, 대신 계산량 큼)가 필요하다. Phase 7은 이 세 알고리즘을
동일 조건에서 정량 비교해 “언제 무엇을 써야 하는가”를 데이터로 답한다.

---

## 3. 구현 상세

### 3.1 플래너 인터페이스 (고정 계약)

`server/planners/base.py`의 `Planner` 프로토콜을 준수한다.

```python
def plan(self, start: Vec3, goal: Vec3, world: World) -> Path  # list[[x,y,z], ...]
```

`World`는 `neighbors / lane_centerline / nearest_lane / is_blocked` 만 요구하므로, 세 플래너 모두
Unity 없이 합성 월드로 단위 테스트된다. 실험 러너는 이 동일 시그니처 덕분에 플래너를 자유롭게
교체한다.

### 3.2 RRT — `server/planners/rrt.py`

- 연속 xz 평면에서 트리 성장. `_rrt_common`의 `sample / steer / collision_free / reconstruct` 사용.
- 매 반복: 목표편향 샘플 → 최근접 노드 탐색 → `step_size`만큼 steer → 간선 충돌검사 → 노드 추가.
- 새 노드가 `goal_radius` 이내이고 목표까지 충돌 없으면 **즉시 종료**(첫 feasible 연결).
- 시작/목표점이 차단되었으면 빈 경로. 직선이 충돌 없으면 그대로 반환(단축).
- `last_nodes / last_iters / last_cost` 를 남겨 실험·검사에 사용.
- 결정성: `RRTConfig.seed`로 `random.Random` 초기화 → 동일 시드·입력 → 동일 경로.

### 3.3 RRT\* — `server/planners/rrt_star.py`

- RRT에 두 개선을 추가(plan §15.3):
  1. **choose-parent**: 새 노드를 비용(cost-to-come) 최소가 되는 이웃에 연결.
  2. **rewire**: 반경(기본 12 m) 내 이웃을 새 노드 경유로 재배선해 비용 절감.
- 첫 연결에서 멈추지 않고 **전체 반복예산을 소진하며 정제**한 뒤, 목표 근방의 최소 총비용 노드로
  경로를 복원한다.
- RRT와 동일한 인스펙션 필드 제공.

### 3.4 공통 유틸 — `server/planners/_rrt_common.py`

- 기존 함수(`dist_xz / bounds / steer / collision_free / sample / reconstruct`)에 더해
  `polyline_length`를 추가(경로 길이 계산).
- `RRTConfig`: `step_size / goal_sample_rate / max_iters / goal_radius / edge_resolution / margin / seed`.

### 3.5 export — `server/planners/__init__.py`

`AStarPlanner`에 더해 `RRTPlanner / RRTStarPlanner / RRTConfig`를 공개.

### 3.6 단위 테스트 — `server/tests/test_rrt.py` (13개)

두 플래너에 대해 파라미터라이즈로 검증:

- 자유공간 직선 경로(길이 ≈ 직선거리),
- 벽 틈새 우회 — **모든 간선이 충돌 없음** 검증,
- 도달 불가(빈틈 없는 벽) → 빈 경로,
- 목표점이 장애물 내부 → 빈 경로,
- 동일 시드 → 동일 경로(결정성),
- RRT\* 품질 ≤ RRT × 1.05,
- 인스펙션 필드/인터페이스 준수.

### 3.7 실험 러너 — `experiments/run_algorithm_compare.py` (plan §20.1)

- **시나리오 3종**
  - `road_open`: 장애물 없는 2×2 일방통행 격자 → A\*의 홈그라운드.
  - `road_detour`: 단일 직선 차선 중앙을 낙하물이 차단 → 차선그래프에 **대체 엣지 없음**.
  - `obstacle_field`: 자유 회랑에 비정형 장애물 6개 → 오프그래프 회피.
- **매트릭스**: 3 시나리오 × {A\*, RRT, RRT\*} × 시드 {0..4} × 차량 수 {1, 5, 20}.
- **측정**: 성공 여부(경로 존재 + 전 구간 충돌프리), 계산시간(ms), 경로 길이(m), 노드 수.
- **출력**: `results/algo_compare_raw.csv`(1170행), `results/algo_compare_summary.csv`(27행, 평균±모표준편차,
  차량 수 대비 총 계산시간 포함).
- 순수 표준 라이브러리(csv/time/statistics)라 추가 의존성 없이 헤드리스 실행.
- 예산: RRT 3000회(첫 연결 시 조기종료라 저렴), RRT\* 1500회(전 예산 정제).

### 3.8 시각화 — `experiments/make_charts.py` + `experiments/analysis.ipynb` (plan §21.3)

- `make_charts.py`(matplotlib Agg, headless): 계산시간/경로길이/성공률 묶음막대, 차량수 대비 계산부하,
  LKA 속도별 lateral error → `results/charts/*.png`.
- `analysis.ipynb`: 동일 차트의 대화형 버전. **nbclient로 실제 실행해 오류 없음 확인**, 셀 id 정규화 완료.

### 3.9 결과 보고서 — `docs/experiment_results.md`

로그 CSV에서 뽑은 수치·차트·해석. §20.1(A\*/RRT/RRT\*)과 §20.2(LKA) 결과를 포함하고,
§20.3–20.5(합류/교차로/돌발)는 로직 완료·Unity 씬 대기로 명시.

---

## 4. 실험 결과 (핵심 수치)

가장 큰 차량 수 기준, 시드 평균.

| 시나리오 | 플래너 | 성공률 | 계산시간(ms) | 경로길이(m) | 노드 |
|---|---|---:|---:|---:|---:|
| road_open | **A\*** | **100%** | **0.35** | 233.8 | 8 |
| road_open | RRT | 100% | 0.05 | 164.3 | 1 |
| road_open | RRT\* | 100% | 437.0 | 168.5 | 1501 |
| road_detour | A\* | **0%** | 0.07 | — | 1 |
| road_detour | **RRT** | **100%** | **0.53** | 112.8 | 60 |
| road_detour | **RRT\*** | **100%** | 540.2 | **96.7** | 1499 |
| obstacle_field | A\* | **0%** | 0.07 | — | 1 |
| obstacle_field | **RRT** | **100%** | **1.01** | 112.2 | 64 |
| obstacle_field | **RRT\*** | **100%** | 710.5 | **97.0** | 1456 |

**해석**

1. **도로그래프에서는 A\*가 정답.** 도로규칙을 지키는 경로(233.8 m)를 ~0.35 ms에 반환. RRT의
   164 m는 격자를 가로지르는 **비합법 직선**(차선 위상·방향 무시)이라 짧아 보일 뿐 실주행 불가.
2. **A\*는 오프그래프 장애물에 실패.** 차선이 막히면 후속 엣지가 없어 **성공률 0%**(빈 경로를
   0.1 ms 미만에 반환). RRT/RRT\*만 자유공간 우회로 100% 성공.
3. **RRT는 빠르고, RRT\*는 더 좋지만 ~700× 비쌈.** RRT ~1 ms(들쭉날쭉 ~112–113 m) vs
   RRT\* ~97 m(**약 14% 단축**, 시드 간 편차도 훨씬 작음)지만 전 예산 정제로 ~0.5–0.7 s 소요.
4. **실시간 함의**: RRT\*는 다중 차량 실시간 재계획에 부적합(차량 수에 선형 증가:
   20대×5시드 obstacle_field ≈ 71 s). **권고 정책 — 전역 도로경로=A\*, 온라인 장애물 우회=RRT,
   품질 벤치마크=RRT\*.** 이는 plan §15.4 예측과 정확히 일치.

**LKA(참고, §20.2)** — Stanley가 전 속도대 가장 안정(RMS ~0.04–0.06 m), Pure Pursuit는 속도↑에
악화(0.089→0.153 m), PID는 저속 우수·고속 불안정(0.016→0.303 m). 게인 튜닝은 사람 작업.

---

## 5. 검증

- **서버 테스트**: `cd server && python -m pytest -q` → **105 passed** (231.9 s).
- **실험 러너**: `run_algorithm_compare.py` 정상 완료, 27행 요약 CSV 생성.
- **차트**: `make_charts.py` 3종 PNG 생성.
- **노트북**: nbclient로 전 셀 실행, 오류 없음.

재현:
```bash
cd server && python -m pytest -q                     # 105 passed
python experiments/run_algorithm_compare.py          # algo_compare_{raw,summary}.csv
python experiments/run_lka_test.py                   # lka_{drive_log,summary}.csv
python experiments/make_charts.py                    # results/charts/*.png
# 또는 experiments/analysis.ipynb 열기
```

---

## 6. 산출물 파일 목록

**신규**
- `server/planners/rrt.py`
- `server/planners/rrt_star.py`
- `server/tests/test_rrt.py`
- `experiments/run_algorithm_compare.py`
- `experiments/make_charts.py`
- `experiments/analysis.ipynb`
- `docs/experiment_results.md`
- `docs/phase7_report.md` (본 문서)
- `docs/worklog/2026-07-02.md`

**수정**
- `server/planners/__init__.py` (RRT export)
- `server/planners/_rrt_common.py` (`polyline_length` 추가)
- `TASKS.md` (Phase 7 체크박스)
- `docs/worklog/README.md` (색인)

**생성 아티팩트**(재생성 가능, git 미추적)
- `experiments/results/algo_compare_{raw,summary}.csv`
- `experiments/results/charts/{algo_compare,algo_time_vs_vehicles,lka_lateral_error}.png`

---

## 7. 남은 작업

| 항목 | 상태 | 비고 |
|---|---|---|
| Phase 7 서버 구현 | ✅ 완료 | 본 보고서 |
| 커밋/병합 | ⏸ 대기 | 지시 시 브랜치 생성 후 커밋 |
| Unity 씬(트랙/램프/교차로) | 👤 사람 | Phase 1/2/4/5/6 게이트 |
| 실험 §20.3–20.5(합류/교차로/돌발) | 👤 사람 | 로직·단위테스트 완료, 씬 있어야 지표 CSV 산출 |

Phase 0–7의 **서버측은 전부 완료**되었고, 남은 것은 Unity 에디터 작업과 그에 종속된 실험뿐이다.
