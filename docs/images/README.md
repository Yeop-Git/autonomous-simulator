# README 미디어 자료

루트 `README.md`에는 Unity Game View에서 직접 촬영한 주행 GIF를 사용한다.
정지 화면 파일은 동일 구도의 고해상도 캡처가 필요할 때 교체용으로 유지한다.

## 실행 영상

| 파일 | 장면 | 확인 기능 |
|---|---|---|
| `docs/videos/lka-test-demo.gif` | LKA_Test | 곡선 차선 추종, 횡오차·tick HUD |
| `docs/videos/highway-merge-lane-change-demo.gif` | Highway | 합류 차량 방향 차선 변경, gap acceptance |
| `docs/videos/urban-straight-demo.gif` | Urban | 신호 대기 후 직진 통과 |
| `docs/videos/urban-left-demo.gif` | Urban | 좌회전 차선 진입, 보호 화살표 대기, 좌회전 |
| `docs/videos/urban-right-demo.gif` | Urban | 적신호 정지·양보 후 우회전 |
| `docs/videos/emergency-avoidance-demo.gif` | EmergencyAvoidance | 낙하물 회피, 긴급차 대응, 차선 복귀 |
| `docs/videos/integrated-city-demo.gif` | IntegratedCity | 통합 경로 계획과 동적 이벤트 구간 |

새로 촬영한 GIF는 800×450, 96색, 무음으로 인코딩했다. IntegratedCity GIF는
960×540이다. 파일별 tick과 촬영 시각은
[`docs/videos/capture-log.csv`](../videos/capture-log.csv)에 기록한다. GIF의 재생
간격은 문서 설명용이며 성능 계측값으로 사용하지 않는다.

## 재촬영

[`ReadmeCaptureTool.cs`](../../unity/Assets/Scripts/Editor/ReadmeCaptureTool.cs)는
Play Mode 진입 직후 Game View 프레임과 `tick`, 행동 상태, 차선, 속도를 함께
기록하는 에디터 전용 도구다. `V2X.ReadmeCapture.Config` EditorPrefs 키에
`CaptureConfig` JSON을 저장한 뒤 Play Mode를 시작하면 설정을 한 번 소비하고
`docs/videos/` 아래에 프레임과 `capture.csv`를 생성한다.

Highway 영상은 합류 차량을 24 m 선행시킨 안전 간격 초기조건에서 우측 차선 변경을
요청했다. Urban 영상은 60 s 신호 주기를 짧게 확인할 수 있도록 2.5–3배속으로
촬영했다. 다른 영상은 기본 시간 배율을 사용했다.

## 정지 화면 파일명

| 파일 | 권장 구도 |
|---|---|
| `hero.png` | Main 허브 또는 IntegratedCity 대표 화면 |
| `scene-lka-test.png` | 곡선 중간 추종 |
| `scene-highway-merge.png` | 합류 또는 차선 변경 완료 순간 |
| `scene-urban-left-turn.png` | 보호 좌회전 진입 |
| `scene-emergency-avoidance.png` | RRT 회피 경로 표시 |
| `scene-emergency-vehicle.png` | 긴급차 양보 |
| `scene-integrated-city.png` | 교차로에서 대로로 진출하는 구간 |
