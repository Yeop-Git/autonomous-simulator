# 실험 결과 재현 기록

이 디렉터리의 CSV와 차트는 README의 정량 주장에 대한 원자료다. 결과는 Unity 장면
녹화가 아니라 Python 헤드리스 모델에서 산출되며, 각 실험의 해석 범위는 아래와 같다.

## 검증 실행

- 실행일: 2026-08-07 (Asia/Seoul)
- 기준 커밋: `800f40220a0391f26a0912b5c3d2368f055e08be` + 작업 트리 변경
- 환경: Microsoft Windows NT 10.0.26200.0, CPython 3.14.4
- CPU 모델: 샌드박스 권한 제한으로 수집하지 못함

```powershell
python experiments\run_algorithm_compare.py
python experiments\run_lka_test.py
python experiments\run_scene_stats.py
python experiments\make_charts.py
cd server
python -m pytest -q --junitxml=../experiments/results/pytest.xml
cd ..
python experiments\validate_results.py
```

## 산출물과 표본 단위

| 파일 | 표본 단위 | 반복/시드 | 해석 경계 |
|---|---|---|---|
| `algo_compare_raw.csv` | 한 플래너의 `(start, goal)` 질의 | 시드 0–4, 질의 묶음 1/5/20 | 묶음은 고정 난수열 앞부분을 공유하며, 20은 동시 차량이 아닌 순차 질의 |
| `algo_compare_summary.csv` | 시나리오·플래너·질의 수별 요약 | 위 원자료의 평균·모표준편차 | 벽시계 시간은 기기 의존 |
| `lka_drive_log.csv` | 합성 곡선 트랙의 한 시각 스텝 | 제어기·속도 조합당 결정론적 1회 | 잡음·초기 오차·조향 지연과 Unity 물리 미포함 |
| `lka_summary.csv` | 제어기·속도 조합 요약 | 결정론적 1회 | 실행 간 신뢰구간 없음 |
| `scene_stats.csv` | 한 헤드리스 씬 실행의 요약 | 씬당 1회 | p50/p95는 틱 분포이며 raw tick 로그가 없어 결과 지표를 독립 재집계하지 못함 |
| `pytest.xml` | 전체 서버 테스트의 JUnit 기록 | 259건 | 통과/스킵 수와 개별 테스트 결과 보존 |
| `manifest.json` | CSV·차트·JUnit·실행기 해시와 검증 결과 | 검증 실행당 1개 | 동일 source hash와 artifact hash로 provenance 확인 |

알고리즘 성공 판정은 경로의 시작·끝점이 지정한 시작·목표의 각 4 m 이내에 있고 모든 인접 경로점 간 선분이
장애물과 충돌하지 않는 경우다. 성공률 100 %는 해당 유한 표본에서 관측한 비율이며,
모든 환경이나 시드에 대한 성공 보증이 아니다.

`scene_stats.csv`의 `step()` 시간은 Python 중앙 제어기 계산만 포함한다. WebSocket 왕복,
JSON 직렬화·역직렬화, 양방향 스키마 검증, Unity 메인 스레드 명령 적용과 렌더링은
포함하지 않는다. 따라서 이 값은 종단간 폐루프 지연이나 프레임률 보증으로 사용할 수 없다.
`validate_results.py`는 씬 식별·규모·지표 범위와 해시를 검사할 뿐, 요약 CSV에서
간격·TTC·급제동을 다시 산출하지는 못한다.

## 테스트 상태

2026-08-07 전체 259건을 수집해 257건이 통과했고 2건은 의도적으로 스킵됐다. 실행 시간은
242 s였다. 스킵은 도로가 없는 Main 허브에 차선 export 검사를 적용하지 않는 경우다.
pytest 캐시 디렉터리 생성 경고 1건은 있었으나 테스트 실패나 오류는 없었다.
