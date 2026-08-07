# 스크린샷 자리

루트 `README.md`가 참조하는 이미지들입니다. 지금 들어 있는 파일은 **자리표시자**이고,
같은 이름으로 덮어쓰기만 하면 README에 그대로 반영됩니다. 파일명을 바꾸면 링크가
깨지니 이름은 유지하세요.

권장 해상도 **1280×720 이상**, 16:9. Unity Play 모드에서 `Game` 뷰를 그 비율로 두고
찍으면 됩니다(에디터 상단 해상도 드롭다운 → `16:9 Aspect`).

| 파일 | 무엇을 찍나 | 찍는 법 |
|---|---|---|
| `hero.png` | IntegratedCity 전경 한 장 | IntegratedCity 실행 → `3` 전경 카메라 |
| `scene-main-hub.png` | 허브 화면 | Main 실행 → 아무 씬이나 선택해 설명 패널이 채워진 상태 |
| `scene-lka-test.png` | 곡선 트랙 추종 | LKA_Test 실행 → `3` 전경, 차량이 곡선 중간에 있을 때 |
| `scene-highway.png` | 본선 3차선 + 온램프 | Highway 실행 → `3` 전경 |
| `scene-highway-merge.png` | 합류 순간 | Highway → 램프 차량이 본선 갭에 들어가는 프레임 |
| `scene-urban.png` | 교차로 전경 | Urban 실행 → `3` 전경, 신호등이 보이게 |
| `scene-urban-left-turn.png` | 보호 좌회전 진입 | Urban → `좌회전` 선택 → 화살표 녹색에 진입하는 순간 |
| `scene-emergency-avoidance.png` | 낙하물 회피 | EmergencyAvoidance → `4` 낙하물 → 회피 경로가 그려진 순간 |
| `scene-emergency-vehicle.png` | 긴급차 양보 | EmergencyAvoidance → `5` 긴급차 → 갓길 대피 중 |
| `scene-integrated-city.png` | 통합 코스 주행 | IntegratedCity → 교차로에서 대로로 나가는 구간 |

실험 차트(`experiments/results/charts/*.png`)는 자리표시자가 아니라 실제 산출물이며
`python experiments/make_charts.py`로 다시 생성됩니다.

## 영상 촬영 규격

영상은 `docs/videos/`에 두고, README에는 같은 이름의 poster PNG를
먼저 표시한다. 기본 포맷은 1920×1080, 60 fps, H.264 MP4, 오디오 없음이다.
카메라는 영상 중간에 바꾸지 않고, 조작 직전 2초와 결과 이후 3초를 남긴다.

| 영상 파일 | 길이 | 카메라 | 트리거와 시작 시점 | README 위치 |
|---|---:|---|---|---|
| `main-hub.mp4` | 10 s | 허브 고정 | 1→5번 선택 후 설명 패널 변경 | §10.0 Main |
| `lka-test.mp4` | 15 s | `3` 전경 | 곡선 진입 2초 전부터 중간 추종까지 | §10.1 |
| `highway-merge.mp4` | 18 s | `3` 전경 | 램프 차량 ETA 조정 시작부터 합류 완료까지 | §10.2 |
| `urban-left-turn.mp4` | 20 s | `3` 전경 | `좌회전` 선택, 화살표 녹색 2초 전부터 교차 완료까지 | §10.3 |
| `emergency-avoidance.mp4` | 18 s | `1` 차량 | `4` 낙하물 투입 2초 전부터 원차선 복귀까지 | §10.4 |
| `emergency-yield.mp4` | 18 s | `3` 전경 | `5` 긴급차 출동 2초 전부터 갓길 양보까지 | §10.4 |
| `integrated-city.mp4` | 24 s | `3` 전경 | 교차로 진입 3초 전부터 대로 진출까지 | §10.5 |

각 영상의 poster는 동일 파일명의 `.png`로 두고, 촬영 시 `scene`, Unity
`tick`, 서버 로그 시작·종료 시각을 `docs/videos/capture-log.csv`의
`file,scene,start_tick,end_tick,server_log,recorded_at` 열에 기록한다. 이 기록이
없는 영상은 실험 증거가 아닌 설명용 화면으로만 취급한다.

자리표시자는 일회성 산출물이라 생성 스크립트를 남겨두지 않았습니다. 실제
스크린샷으로 덮어쓰면 그걸로 끝입니다.
