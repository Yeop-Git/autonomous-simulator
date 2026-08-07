using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEditor.Events;
using Unity.Cinemachine;
using Unity.Cinemachine.TargetTracking;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem.UI;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using V2X.Communication;
using V2X.Road;
using V2X.Sim;
using V2X.UI;
using V2X.Vehicle;
using V2X.Visualization;

namespace V2X.EditorTools
{
    /// <summary>
    /// Deterministic scene authoring entry point used by Unity MCP.  Keeping the
    /// construction in one command makes the generated demo scenes reviewable,
    /// repeatable, and safe to rebuild after protocol/component changes.
    /// </summary>
    public static class V2XSceneBuilder
    {
        private const string SceneDir = "Assets/Scenes";
        private const string MaterialDir = "Assets/Generated/Materials";
        private const string RoundedUiAssetPath = "Assets/Generated/V2X_UI_Rounded.asset";
        private const string RegularFontAssetPath =
            "Assets/Resources/Fonts/Pretendard-Regular.ttf";
        private const string SemiboldFontAssetPath =
            "Assets/Resources/Fonts/Pretendard-SemiBold.ttf";
        private static readonly Color AppleBlue = new(0f, .4f, .8f, 1f);
        private static readonly Color AppleBluePressed = new(0f, .443f, .89f, 1f);
        private static readonly Color AppleInk = new(.114f, .114f, .122f, 1f);
        private static readonly Color AppleMuted = new(.478f, .478f, .478f, 1f);
        private static readonly Color AppleParchment = new(.961f, .961f, .969f, 1f);
        private static readonly Color AppleWhite = Color.white;
        private const string PedestrianAssetPath =
            "Assets/Pedestrian/Pedestrian.prefab";
        private const string PedestrianBodyMaterialPath =
            "Assets/Pedestrian/Materials/Base.mat";
        private const string PedestrianJointMaterialPath =
            "Assets/Pedestrian/Materials/Joint.mat";
        private const string CarPaletteMaterialPath =
            "Assets/Pack_FREE_Cars/Materials/ColorPalette.mat";
        private static readonly string[] CarPrefabPaths =
        {
            "Assets/Pack_FREE_Cars/Prefabs/Hatchback.prefab",
            "Assets/Pack_FREE_Cars/Prefabs/Pickup.prefab",
            "Assets/Pack_FREE_Cars/Prefabs/Police.prefab",
            "Assets/Pack_FREE_Cars/Prefabs/Taxi.prefab",
            "Assets/Pack_FREE_Cars/Prefabs/Towtruck.prefab",
            "Assets/Pack_FREE_Cars/Prefabs/Truck.prefab",
            "Assets/Pack_FREE_Cars/Prefabs/Van.prefab",
            "Assets/Pack_FREE_Cars/Prefabs/VanBig.prefab",
        };

        [MenuItem("V2X/Build All Demo Scenes")]
        public static void BuildAllScenes()
        {
            EnsureDirectories();
            // Build the demo scenes first: the hub only lists scenes, but it
            // cannot load one that never made it into Build Settings.
            BuildLkaScene();
            BuildHighwayScene();
            BuildUrbanScene();
            BuildEmergencyAvoidanceScene();
            BuildIntegratedCityScene();
            BuildMainScene();
            EditorBuildSettings.scenes = new[]
            {
                new EditorBuildSettingsScene($"{SceneDir}/Main.unity", true),
                new EditorBuildSettingsScene($"{SceneDir}/LKA_Test.unity", true),
                new EditorBuildSettingsScene($"{SceneDir}/Highway.unity", true),
                new EditorBuildSettingsScene($"{SceneDir}/Urban.unity", true),
                new EditorBuildSettingsScene($"{SceneDir}/EmergencyAvoidance.unity", true),
                new EditorBuildSettingsScene($"{SceneDir}/IntegratedCity.unity", true),
            };
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            EditorSceneManager.OpenScene($"{SceneDir}/Main.unity");
            Debug.Log("[V2XSceneBuilder] Built the Main hub and all five demo scenes.");
        }

        /// <summary>Scene menu shown by the Main hub, in display order.</summary>
        private static readonly HubSceneEntry[] HubEntries =
        {
            new()
            {
                sceneName = "LKA_Test",
                title = "LKA_Test · 차선 유지 시험로",
                summary = "구성: 반경 90 m 단일 곡선 트랙, 차량 1대, 교통·이벤트 없음.\n" +
                          "목적: 다른 요인을 모두 제거하고 횡방향 제어기만 격리 계측.",
                // The panel font is proportional, so no attempt is made to
                // align a label column: each item is one bullet, continuation
                // lines carry a fixed two-space indent.
                techniques =
                    "· 횡방향 제어: Pure Pursuit / Stanley — 씬 기본값 Stanley\n" +
                    "  (미조정 기본 이득: Stanley k=1.5, 최대 조향 0.6 rad)\n" +
                    "· 종방향 제어: ACC 자유주행 구간 (선행차 없음)\n" +
                    "· 계측: Frenet 오차 — 횡오차 · 헤딩오차 · Menger 곡률\n" +
                    "· 로깅: 고정 CSV 스키마 → 속도별 RMS 횡오차\n" +
                    "· 별도 근거: Python 합성 트랙(R=140 m) 단일 스윕 — 이 씬의\n" +
                    "  R=90 m Unity 씬 구성과 혼동하지 않음 (README §11.2)",
                controls = "1·2·3 카메라 전환 / Esc 허브로",
            },
            new()
            {
                sceneName = "Highway",
                title = "Highway · 고속도로 합류와 차선 변경",
                summary = "구성: 3차선 본선 300 m(27.8 m/s) + 온램프(18 m/s).\n" +
                          "쟁점: 램프가 본선 끝이 아닌 중간 z=115 m로 접합한다.",
                techniques =
                    "· 합류 예약: 본선 ETA 정렬 → 2.0 s 갭 탐색 → 램프 재타이밍\n" +
                    "  → 갭이 없으면 본선 차량에 감속을 지시해 갭을 연다\n" +
                    "  (램프 차량 혼자서는 불가능한 수 — 중앙 관제의 핵심)\n" +
                    "· 판정: 합류 차선을 이름이 아닌 위상·기하로 도출\n" +
                    "· 추종: 중간 접합을 반영한 선행차 탐색(하류·형제 차선) + ACC\n" +
                    "· 차선 변경: lead/lag 시간 간격 1.5 s 갭 수용\n" +
                    "· 전역 경로: A* + 중간 합류 접합 처리",
                controls = "Q/E 차선 변경 · 1·2·3 카메라 / Esc 허브로",
            },
            new()
            {
                sceneName = "Urban",
                title = "Urban · 신호 교차로와 보호 좌회전",
                summary = "구성: 4방향 8접근로 신호 교차로, 정지선은 중심에서 16 m.\n" +
                          "동작: 직진·좌회전·우회전을 UI로 지정하면 서버가 판단한다.",
                techniques =
                    "· 신호: 60 s 고정 주기 — 서버가 집행하고 Unity는 그린다\n" +
                    "  동서녹 10 → 보행 8 → 남북녹 10 → 좌회전 6 → 보행 8 s\n" +
                    "  + 황색·전적색 버퍼 18 s (전체 60 s, 상세 README §7.3)\n" +
                    "  정지선 5.5 m 전, 1.8 m/s²의 편안한 감속으로 선행 제동\n" +
                    "· 좌회전: 갭 1.25 s 수용 → 정지선 14 m 전 차선 변경 완료\n" +
                    "  → 화살표 대기 → 교차. 매 틱 재평가, 실패 시 직진 취소\n" +
                    "  좌회전 중 보행자만 통로(반폭 3 m) 투영으로 판정 — 인도의\n" +
                    "  사람은 무시. 그 외 충돌은 반경 2.5 m 원형 판정이다\n" +
                    "· 충돌: 신호로 관리되는 충돌은 제외 — 녹색이 적색에 제동하지 않게",
                controls = "직진/좌회전/우회전 토글 · 1·2·3 카메라 / Esc 허브로",
            },
            new()
            {
                sceneName = "EmergencyAvoidance",
                title = "EmergencyAvoidance · 돌발 장애물과 긴급차 회피",
                summary = "구성: 직선 4차선 실험로(주행 3 + 갓길 1). 낙하물이 떨어지고 " +
                          "뒤에서 긴급차가 접근한다.\n" +
                          "쟁점: 국부 샘플링 플래너를 수동 전환해 비교하는 전용 실험로.",
                techniques =
                    "· 국부 계획: 코리도 제한 RRT / RRT* — 실행 중 전환 가능\n" +
                    "  탐색 공간 = 인접 차선군 + 후속 차선. 타 차량은 3 s 예측을\n" +
                    "  반경 2.45 m로, 장애물은 자기 반경 + 1.45 m로 부풀린다\n" +
                    "  중단 한계 RRT 45 ms / RRT* 140 ms — 40 ms 송신 간격보다 길 수 있음\n" +
                    "  RRT* 전환은 비교용이며 25 Hz 운용 기본값은 RRT\n" +
                    "· 후처리: 단축(0.5 m) → 재샘플링(2 m) → 꺾임각 78° 초과 시 폐기\n" +
                    "· 상태기: 감지 → 탈출계획 → 횡방향 이탈 → 복귀계획 → 원차선 합류\n" +
                    "  복귀가 막히면 정지 대신 회피 차선 주행 후 1.5 s 뒤 재시도\n" +
                    "· 긴급차: 반경 60 m에서 최우측 갓길 대피, 통과 확인 후 복귀\n" +
                    "· 계측: 계획 시간 / 최소 여유거리를 명령에 실어 실시간 보고",
                controls = "4 낙하물 · 5 긴급차 · 6 RRT↔RRT* · 0 리셋 / Esc 허브로",
            },
            new()
            {
                sceneName = "IntegratedCity",
                title = "IntegratedCity · 통합 시나리오",
                summary = "구성: 교차로 → 대로 → 순환로. 도심 신호, 갓길 테이퍼, " +
                          "장애물·긴급차 이벤트를 선택적으로 통합한다.\n" +
                          "쟁점: 도심 격자(urban_*)와 간선(city_*), 명명 규칙이 다른 두 계열의 공존.",
                techniques =
                    "· 합류: 갓길↔대로 테이퍼 — 접합 15 m 전 나란히 달리는 더 느린\n" +
                    "  차선으로 판정. 차선 이름이 아니라 위상·기하가 근거다\n" +
                    "· 회피: 서로 다른 도로 계열을 가로지르는 코리도 구성\n" +
                    "· 인지: 다차량 동시 주행 중 전역 상황 인지와 해석적 충돌 예측\n" +
                    "  (지평선 4 s, 안전거리 2.5 m, 샘플 사이 터널링 없음)\n" +
                    "· 부하: 3대 50 s 헤드리스 단일 실행에서 Python step()\n" +
                    "  p50 0.74 ms / p95 1.40 ms — 네트워크·Unity 적용 시간 제외",
                controls = "시나리오 디렉터가 이벤트를 자동 진행 · 1·2·3 카메라 / Esc 허브로",
            },
        };

        [MenuItem("V2X/Build Main Hub")]
        public static void BuildMainScene()
        {
            EnsureDirectories();
            NewScene("Main", out var camera);
            camera.transform.position = new Vector3(0f, 2f, -10f);
            camera.transform.rotation = Quaternion.identity;
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = AppleParchment;

            var canvasGo = new GameObject("Hub Canvas");
            var canvas = canvasGo.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            var scaler = canvasGo.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);
            canvasGo.AddComponent<GraphicRaycaster>();
            if (UnityEngine.Object.FindFirstObjectByType<EventSystem>() == null)
            {
                var eventGo = new GameObject("EventSystem");
                eventGo.AddComponent<EventSystem>();
                eventGo.AddComponent<InputSystemUIInputModule>();
            }

            CreateHubLabel(canvas.transform, "Hub Title",
                "중앙 집중형 V2X 자율주행 시뮬레이터",
                new Vector2(0f, -70f), new Vector2(1400f, 52f), 38,
                TextAnchor.MiddleCenter, anchorY: 1f, color: AppleInk,
                fontStyle: FontStyle.Bold);
            CreateHubLabel(canvas.transform, "Hub Subtitle",
                "중앙 서버의 판단을 Unity에서 적용·시각화하는 완전 V2X 시뮬레이터",
                new Vector2(0f, -122f), new Vector2(1400f, 34f), 20,
                TextAnchor.MiddleCenter, anchorY: 1f, color: AppleMuted);

            // Left: the menu. Right: what the selected scene actually exercises.
            var buttons = new Button[HubEntries.Length];
            for (int i = 0; i < HubEntries.Length; i++)
            {
                buttons[i] = CreateHubMenuButton(canvas.transform,
                    $"{i + 1}.  {HubEntries[i].sceneName}",
                    new Vector2(-620f, 120f - i * 66f));
            }

            // The detail panel is a spec sheet, so it is laid out as one:
            // heading, two-line abstract, then the technique block that gets
            // most of the height. Labels overflow downward rather than clip
            // (CreateHubLabel), so the vertical gaps below are the real
            // guarantee that a long entry does not run into the next field.
            var panel = CreateHubPanel(canvas.transform, "Detail Panel",
                new Vector2(300f, -10f), new Vector2(900f, 460f));
            var title = CreateHubLabel(panel, "Detail Title", "",
                new Vector2(0f, 198f), new Vector2(840f, 34f), 26, TextAnchor.MiddleLeft,
                color: AppleInk, fontStyle: FontStyle.Bold);
            var summary = CreateHubLabel(panel, "Detail Summary", "",
                new Vector2(0f, 142f), new Vector2(840f, 72f), 18, TextAnchor.UpperLeft,
                color: AppleMuted);
            var techniques = CreateHubLabel(panel, "Detail Techniques", "",
                new Vector2(0f, -12f), new Vector2(840f, 230f), 16, TextAnchor.UpperLeft,
                color: AppleInk);
            var controls = CreateHubLabel(panel, "Detail Controls", "",
                new Vector2(0f, -190f), new Vector2(840f, 40f), 16, TextAnchor.UpperLeft,
                color: AppleBlue);

            var run = CreateButton(canvas.transform, "▶  이 씬 실행 (Enter)",
                new Vector2(0f, 96f), 320f);
            StylePrimaryButton(run);
            var status = CreateHubLabel(canvas.transform, "Hub Status", "",
                new Vector2(0f, 50f), new Vector2(1400f, 30f), 16,
                TextAnchor.MiddleCenter, anchorY: 0f, color: AppleMuted);

            var hub = canvasGo.AddComponent<SceneHubController>();
            hub.entries = HubEntries;
            hub.sceneButtons = buttons;
            hub.runButton = run;
            hub.titleText = title;
            hub.summaryText = summary;
            hub.techniquesText = techniques;
            hub.controlsText = controls;
            hub.statusText = status;

            // No Lane components here: the hub is a menu, not a road.
            Save("Main", exportLanes: false);
        }

        [MenuItem("V2X/Restyle All Scene UI")]
        public static void RestyleAllSceneUi()
        {
            EnsureDirectories();
            string activePath = SceneManager.GetActiveScene().path;
            string[] names = { "Main", "LKA_Test", "Highway", "Urban",
                "EmergencyAvoidance", "IntegratedCity" };
            foreach (string name in names)
            {
                string path = $"{SceneDir}/{name}.unity";
                if (!File.Exists(path)) continue;
                var scene = EditorSceneManager.OpenScene(path);
                foreach (var scaler in UnityEngine.Object.FindObjectsByType<CanvasScaler>(
                             FindObjectsInactive.Include, FindObjectsSortMode.None))
                {
                    scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
                    scaler.referenceResolution = new Vector2(1920f, 1080f);
                    scaler.matchWidthOrHeight = .5f;
                }
                var buttons = UnityEngine.Object.FindObjectsByType<Button>(
                    FindObjectsInactive.Include, FindObjectsSortMode.None);
                bool hasReturn = false;
                foreach (var button in buttons)
                {
                    string label = button.GetComponentInChildren<Text>(true)?.text ?? "";
                    if (label.Contains("허브")) { hasReturn = true; break; }
                }
                if (name != "Main" && !hasReturn)
                {
                    var canvas = UnityEngine.Object.FindFirstObjectByType<Canvas>();
                    if (canvas != null) CreateReturnToHubControl(canvas);
                    buttons = UnityEngine.Object.FindObjectsByType<Button>(
                        FindObjectsInactive.Include, FindObjectsSortMode.None);
                }
                foreach (var button in buttons)
                {
                    string label = button.GetComponentInChildren<Text>(true)?.text ?? "";
                    StyleSecondaryButton(button);
                    if (label.Contains("RETRY") || label.Contains("이 씬 실행"))
                        StylePrimaryButton(button);
                    if (label.Contains("허브"))
                    {
                        var rect = button.GetComponent<RectTransform>();
                        rect.anchorMin = rect.anchorMax = Vector2.one;
                        rect.pivot = Vector2.one;
                        rect.anchoredPosition = new Vector2(-16f, -16f);
                    }
                }
                foreach (var text in UnityEngine.Object.FindObjectsByType<Text>(
                             FindObjectsInactive.Include, FindObjectsSortMode.None))
                {
                    bool emphasized = text.name.Contains("Title") ||
                                      text.transform.parent?.GetComponent<Button>() != null;
                    ApplyTypography(text, emphasized);
                }
                foreach (var toggle in UnityEngine.Object.FindObjectsByType<Toggle>(
                             FindObjectsInactive.Include, FindObjectsSortMode.None))
                {
                    if (toggle.targetGraphic is Image background)
                    {
                        background.sprite = GetRoundedUiSprite();
                        background.type = Image.Type.Sliced;
                        background.color = AppleWhite;
                    }
                    if (toggle.graphic is Image selected)
                    {
                        selected.sprite = GetRoundedUiSprite();
                        selected.type = Image.Type.Sliced;
                        selected.color = AppleBlue;
                    }
                    var label = toggle.GetComponentInChildren<Text>(true);
                    if (label != null) label.color = AppleInk;
                }
                foreach (var image in UnityEngine.Object.FindObjectsByType<Image>(
                             FindObjectsInactive.Include, FindObjectsSortMode.None))
                {
                    if (image.name != "Retry Panel") continue;
                    image.sprite = GetRoundedUiSprite();
                    image.type = Image.Type.Sliced;
                    image.color = AppleWhite;
                    var message = image.GetComponentInChildren<Text>(true);
                    if (message != null) message.color = AppleInk;
                }
                EditorSceneManager.SaveScene(scene);
            }
            if (!string.IsNullOrEmpty(activePath) && File.Exists(activePath))
                EditorSceneManager.OpenScene(activePath);
            AssetDatabase.SaveAssets();
            Debug.Log("[V2XSceneBuilder] Restyled UI in all six scenes.");
        }

        [MenuItem("V2X/Build LKA Test Scene")]
        public static void BuildLkaScene()
        {
            EnsureDirectories();
            NewScene("LKA_Test", out var camera);

            var curve = new List<Vector3>();
            const float radius = 90f;
            for (int i = 0; i <= 24; i++)
            {
                float a = Mathf.Lerp(-55f, 55f, i / 24f) * Mathf.Deg2Rad;
                curve.Add(new Vector3(radius * Mathf.Sin(a), 0f,
                    radius * (1f - Mathf.Cos(a))));
            }
            CreateRoadRibbon("LKA Track", curve, 7f, Color.black);
            var lane = CreateLane("lka_curve", 27.8f, curve);
            var vehicle = CreateVehicle("lka_ego", curve[0] + Vector3.up * 0.5f,
                curve[^1], new Color(0.2f, 0.8f, 1f));
            vehicle.lateralLaw = LateralLaw.Stanley;
            WireSimulation("lka_test", new[] { lane }, new[] { vehicle }, Array.Empty<DynamicObjectAgent>());
            CreateCameraSystem(camera, vehicle, false,
                new Vector3(0f, 145f, -20f), new Vector3(0f, 0f, 35f));

            camera.transform.position = new Vector3(0f, 105f, -30f);
            camera.transform.rotation = Quaternion.Euler(68f, 0f, 0f);
            Save("LKA_Test");
        }

        [MenuItem("V2X/Build Highway Scene")]
        public static void BuildHighwayScene()
        {
            EnsureDirectories();
            NewScene("Highway", out var camera);
            CreateCube("Highway Surface", new Vector3(0f, -0.1f, 150f),
                new Vector3(14f, 0.2f, 320f), Color.black);

            var lanes = new List<Lane>();
            for (int i = 0; i < 3; i++)
            {
                float x = (i - 1) * 3.5f;
                lanes.Add(CreateLane($"hw_l{i}", 27.8f,
                    Points(new Vector3(x, 0f, 0f), new Vector3(x, 0f, 300f), 16)));
                if (i < 2) CreateDashedLine(x + 1.75f, 0f, 300f);
            }
            CreateSolidLineZ("Highway Left Edge", -5.3f, 0f, 300f, Color.white, .18f);
            CreateSolidLineZ("Highway Right Edge", 5.3f, 0f, 300f, Color.white, .18f);
            for (int i = 0; i < lanes.Count; i++)
            {
                lanes[i].leftLane = i > 0 ? lanes[i - 1] : null;
                lanes[i].rightLane = i < lanes.Count - 1 ? lanes[i + 1] : null;
            }

            var rampPoints = new List<Vector3>();
            for (int i = 0; i <= 10; i++)
            {
                float t = i / 10f;
                rampPoints.Add(new Vector3(Mathf.Lerp(13f, 3.5f, t), 0f,
                    Mathf.Lerp(15f, 115f, t)));
            }
            CreateRoadRibbon("On Ramp", rampPoints, 4f, new Color(0.08f, 0.08f, 0.08f));
            CreatePolylineEdges("Ramp Edge", rampPoints, 1.85f, Color.white);
            var ramp = CreateLane("hw_ramp", 18f, rampPoints);
            ramp.nextLanes.Add(lanes[2]);
            lanes.Add(ramp);

            var vehicles = new[]
            {
                CreateVehicle("car_0", new Vector3(0f, .5f, 8f), new Vector3(0f, 0f, 285f), Color.cyan),
                CreateVehicle("car_1", new Vector3(-3.5f, .5f, 38f), new Vector3(-3.5f, 0f, 285f), Color.yellow),
                CreateVehicle("car_2", new Vector3(3.5f, .5f, 68f), new Vector3(3.5f, 0f, 285f), Color.green),
                CreateVehicle("car_3", new Vector3(0f, .5f, 100f), new Vector3(0f, 0f, 285f), new Color(.8f, .35f, 1f)),
                CreateVehicle("car_4", new Vector3(-3.5f, .5f, 135f), new Vector3(-3.5f, 0f, 285f), new Color(.2f, .6f, 1f)),
                CreateVehicle("car_5", new Vector3(3.5f, .5f, 170f), new Vector3(3.5f, 0f, 285f), new Color(1f, .25f, .2f)),
                CreateVehicle("car_6", new Vector3(0f, .5f, 210f), new Vector3(0f, 0f, 285f), new Color(.4f, 1f, .7f)),
                CreateVehicle("ramp_car", rampPoints[0] + Vector3.up * .5f, new Vector3(3.5f, 0f, 285f), new Color(1f, .5f, .1f)),
            };
            WireSimulation("highway", lanes, vehicles, Array.Empty<DynamicObjectAgent>());
            CreateCameraSystem(camera, vehicles[0], true,
                new Vector3(90f, 145f, -40f), new Vector3(0f, 0f, 150f));

            camera.transform.position = new Vector3(45f, 95f, -50f);
            camera.transform.LookAt(new Vector3(0f, 0f, 110f));
            camera.fieldOfView = 55f;
            Save("Highway");
        }

        [MenuItem("V2X/Build Emergency Avoidance Lab")]
        public static void BuildEmergencyAvoidanceScene()
        {
            EnsureDirectories();
            NewScene("EmergencyAvoidance", out var camera);

            const float zEnd = 330f;
            CreateCube("Emergency Lab Asphalt", new Vector3(1.75f, -.1f, zEnd * .5f),
                new Vector3(17f, .2f, zEnd + 20f), new Color(.025f, .032f, .045f));
            CreateCube("Left Landscape", new Vector3(-25f, -.22f, zEnd * .5f),
                new Vector3(32f, .25f, zEnd + 30f), new Color(.055f, .12f, .075f));
            CreateCube("Right Landscape", new Vector3(28f, -.22f, zEnd * .5f),
                new Vector3(37f, .25f, zEnd + 30f), new Color(.055f, .12f, .075f));

            var lanes = new List<Lane>();
            string[] ids = { "ea_left", "ea_center", "ea_right", "ea_shoulder" };
            float[] xs = { -3.5f, 0f, 3.5f, 7f };
            for (int i = 0; i < ids.Length; i++)
            {
                var lane = CreateLane(ids[i], i == 3 ? 7f : 22f,
                    Points(new Vector3(xs[i], 0f, 0f), new Vector3(xs[i], 0f, zEnd), 23));
                lane.width = i == 3 ? 3.2f : 3.5f;
                lanes.Add(lane);
                if (i < 2) CreateDashedLine(xs[i] + 1.75f, 0f, zEnd);
            }
            CreateSolidLineZ("Shoulder Boundary", 5.25f, 0f, zEnd,
                new Color(1f, .82f, .18f), .2f);
            CreateSolidLineZ("Left Road Edge", -5.35f, 0f, zEnd, Color.white, .18f);
            CreateSolidLineZ("Right Shoulder Edge", 8.65f, 0f, zEnd, Color.white, .18f);
            for (int i = 0; i < lanes.Count; i++)
            {
                lanes[i].leftLane = i > 0 ? lanes[i - 1] : null;
                lanes[i].rightLane = i < lanes.Count - 1 ? lanes[i + 1] : null;
            }

            CreateEmergencyLabEnvironment(zEnd);
            var ego = CreateVehicle("avoidance_ego", new Vector3(0f, .5f, 12f),
                new Vector3(0f, 0f, 315f), new Color(.04f, .9f, 1f));
            ego.maxSpeed = 24f;
            ego.maxAccel = 2.8f;
            ego.maxDecel = 7.5f;
            ego.destroyOnArrival = false;
            ego.destroyOutsideBoundary = false;
            ego.showRetryOnExit = false;

            WireSimulation("emergency_avoidance", lanes, new[] { ego },
                Array.Empty<DynamicObjectAgent>());
            var runtime = GameObject.Find("V2X Runtime");
            var sim = runtime.GetComponent<SimulationManager>();
            sim.plannerMode = "rrt";
            runtime.GetComponent<DebugDashboard>().show = false;
            var scenario = runtime.AddComponent<EmergencyAvoidanceScenarioController>();
            scenario.simulation = sim;
            scenario.ego = ego;
            scenario.policePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(
                "Assets/Pack_FREE_Cars/Prefabs/Police.prefab");
            scenario.automaticDemo = true;
            var labDashboard = runtime.AddComponent<EmergencyAvoidanceDashboard>();
            labDashboard.ego = ego;
            labDashboard.simulation = sim;
            labDashboard.scenario = scenario;
            labDashboard.client = runtime.GetComponent<V2XClient>();

            var canvas = CreateCameraSystem(camera, ego, false,
                new Vector3(62f, 92f, 62f), new Vector3(1.5f, 0f, 145f));
            CreateEmergencyLabControls(canvas.transform, scenario);
            ego.showRetryOnExit = false;
            camera.transform.position = new Vector3(23f, 28f, -28f);
            camera.transform.LookAt(new Vector3(1.5f, 1.2f, 58f));
            camera.fieldOfView = 56f;

            Save("EmergencyAvoidance");
            AddSceneToBuildSettings($"{SceneDir}/EmergencyAvoidance.unity");
            AssetDatabase.SaveAssets();
            Debug.Log("[V2XSceneBuilder] Built EmergencyAvoidance RRT/RRT* lab.");
        }

        [MenuItem("V2X/Build Integrated 10-Minute City")]
        public static void BuildIntegratedCityScene()
        {
            EnsureDirectories();
            string source = $"{SceneDir}/Urban.unity";
            string destination = $"{SceneDir}/IntegratedCity.unity";
            if (AssetDatabase.LoadAssetAtPath<SceneAsset>(source) == null)
                throw new InvalidOperationException("Build the Urban scene before the integrated city.");
            if (AssetDatabase.LoadAssetAtPath<SceneAsset>(destination) != null)
                AssetDatabase.DeleteAsset(destination);
            if (!AssetDatabase.CopyAsset(source, destination))
                throw new InvalidOperationException("Failed to copy Urban.unity.");

            EditorSceneManager.OpenScene(destination, OpenSceneMode.Single);
            var road = UnityEngine.Object.FindFirstObjectByType<RoadNetworkManager>();
            var simulation = UnityEngine.Object.FindFirstObjectByType<SimulationManager>();
            var ego = GameObject.Find("urban_ego")?.GetComponent<VehicleController>();
            if (road == null || simulation == null || ego == null)
                throw new InvalidOperationException("Urban base scene is missing its road, simulation, or ego.");

            var nbOut = GameObject.Find("urban_nb_0_out")?.GetComponent<Lane>();
            var nbIn = GameObject.Find("urban_nb_0_in")?.GetComponent<Lane>();
            if (nbOut == null || nbIn == null)
                throw new InvalidOperationException("Urban northbound route is incomplete.");

            var boulevardPoints = Points(new Vector3(5.4f, 0f, 70f), new Vector3(5.4f, 0f, 360f), 25);
            // The shoulder is an emergency escape lane, not a second through
            // lane. It has to taper back onto the boulevard so that it ENDS
            // where the eastbound connector starts; a shoulder held at x=9 all
            // the way to z=360 makes the escape->turn lane-graph edge join 3.6 m
            // off the connector centreline, and any route stitched through it
            // steps sideways at the join.
            var escapePoints = Points(new Vector3(9f, 0f, 70f), new Vector3(9f, 0f, 320f), 21);
            escapePoints.AddRange(BezierPoints(new Vector3(9f, 0f, 320f),
                new Vector3(9f, 0f, 346f), new Vector3(5.4f, 0f, 360f), 5).GetRange(1, 4));
            var turnEastPoints = BezierPoints(new Vector3(5.4f, 0f, 360f),
                new Vector3(5.4f, 0f, 430f), new Vector3(80f, 0f, 430f), 18);
            var turnSouthPoints = BezierPoints(new Vector3(80f, 0f, 430f),
                new Vector3(115f, 0f, 430f), new Vector3(115f, 0f, 395f), 14);
            var southPoints = Points(new Vector3(115f, 0f, 395f), new Vector3(115f, 0f, -160f), 35);
            var turnWestPoints = BezierPoints(new Vector3(115f, 0f, -160f),
                new Vector3(115f, 0f, -200f), new Vector3(75f, 0f, -200f), 14);
            var westPoints = Points(new Vector3(75f, 0f, -200f), new Vector3(45f, 0f, -200f), 5);
            var turnNorthPoints = BezierPoints(new Vector3(45f, 0f, -200f),
                new Vector3(5.4f, 0f, -200f), new Vector3(5.4f, 0f, -160f), 14);
            var returnPoints = Points(new Vector3(5.4f, 0f, -160f), new Vector3(5.4f, 0f, -70f), 10);

            var boulevard = CreateLane("city_boulevard_main", 20f, boulevardPoints);
            var escape = CreateLane("city_boulevard_escape", 8f, escapePoints);
            var turnEast = CreateLane("city_turn_east", 12f, turnEastPoints);
            var turnSouth = CreateLane("city_turn_south", 12f, turnSouthPoints);
            var south = CreateLane("city_south", 18f, southPoints);
            var turnWest = CreateLane("city_turn_west", 12f, turnWestPoints);
            var west = CreateLane("city_west", 14f, westPoints);
            var turnNorth = CreateLane("city_turn_north", 12f, turnNorthPoints);
            var cityReturn = CreateLane("city_return", 16f, returnPoints);
            boulevard.rightLane = escape;
            escape.leftLane = boulevard;
            nbOut.nextLanes.Add(boulevard);
            boulevard.nextLanes.Add(turnEast);
            escape.nextLanes.Add(turnEast);
            turnEast.nextLanes.Add(turnSouth);
            turnSouth.nextLanes.Add(south);
            south.nextLanes.Add(turnWest);
            turnWest.nextLanes.Add(west);
            west.nextLanes.Add(turnNorth);
            turnNorth.nextLanes.Add(cityReturn);
            cityReturn.nextLanes.Add(nbIn);
            road.lanes.AddRange(new[] { boulevard, escape, turnEast, turnSouth, south,
                turnWest, west, turnNorth, cityReturn });

            CreateIntegratedCityEnvironment(boulevardPoints, turnEastPoints, turnSouthPoints,
                southPoints, turnWestPoints, westPoints, turnNorthPoints, returnPoints);

            simulation.scenario = "integrated_city";
            simulation.plannerMode = "rrt";
            simulation.vehicles = new List<VehicleController>(
                UnityEngine.Object.FindObjectsByType<VehicleController>(FindObjectsSortMode.None));
            ego.destroyOnArrival = false;
            ego.destroyOutsideBoundary = false;
            ego.showRetryOnExit = false;
            ego.maxSpeed = 22f;
            ego.maneuver = "straight";

            var strategy = UnityEngine.Object.FindFirstObjectByType<UrbanDrivingStrategyController>();
            if (strategy != null) strategy.enabled = false;
            var retry = UnityEngine.Object.FindFirstObjectByType<RetryPanelController>();
            if (retry != null && retry.panel != null) retry.panel.SetActive(false);
            var debug = simulation.GetComponent<DebugDashboard>();
            if (debug != null) debug.show = false;

            var eventController = simulation.gameObject.AddComponent<EmergencyAvoidanceScenarioController>();
            eventController.simulation = simulation;
            eventController.ego = ego;
            eventController.scenarioName = "integrated_city";
            eventController.automaticDemo = false;
            eventController.obstacleDistance = 44f;
            eventController.emergencySpawnDistance = 38f;
            eventController.policePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(
                "Assets/Pack_FREE_Cars/Prefabs/Police.prefab");

            Transform northCheckpoint = CreateCheckpoint("Outer Loop Checkpoint",
                new Vector3(115f, 0f, 20f));
            Transform southCheckpoint = CreateCheckpoint("Urban Return Checkpoint",
                new Vector3(5.4f, 0f, -145f));
            var director = simulation.gameObject.AddComponent<IntegratedCityScenarioDirector>();
            director.simulation = simulation;
            director.ego = ego;
            director.events = eventController;
            director.northCheckpoint = northCheckpoint;
            director.southCheckpoint = southCheckpoint;
            director.showcaseDuration = 600f;

            var integratedDashboard = simulation.gameObject.AddComponent<IntegratedCityDashboard>();
            integratedDashboard.ego = ego;
            integratedDashboard.simulation = simulation;
            integratedDashboard.director = director;
            integratedDashboard.client = simulation.GetComponent<V2XClient>();

            var overview = GameObject.Find("CM Overview");
            var overviewTarget = GameObject.Find("CM Overview Look Target");
            if (overview != null) overview.transform.position = new Vector3(245f, 340f, -90f);
            if (overviewTarget != null) overviewTarget.transform.position = new Vector3(48f, 0f, 105f);

            Save("IntegratedCity");
            AddSceneToBuildSettings(destination);
            AssetDatabase.SaveAssets();
            Debug.Log("[V2XSceneBuilder] Built IntegratedCity 10-minute showcase.");
        }

        [MenuItem("V2X/Build Urban Scene")]
        public static void BuildUrbanScene()
        {
            EnsureDirectories();
            NewScene("Urban", out var camera);
            CreateCube("North South Road", new Vector3(0f, -0.1f, 0f),
                new Vector3(18f, 0.2f, 150f), Color.black);
            CreateCube("East West Road", new Vector3(0f, -0.09f, 0f),
                new Vector3(150f, 0.2f, 18f), Color.black);
            CreateCube("Ground", new Vector3(0f, -0.25f, 0f),
                new Vector3(170f, 0.3f, 170f), new Color(.18f, .22f, .18f));
            CreateUrbanLaneMarkings();

            var lanes = new List<Lane>();
            // Republic of Korea right-hand traffic.  Lane 0 is the rightmost
            // lane in its travel direction; lane 1 is the inner/left lane.
            Lane nb0In = CreateLane("urban_nb_0_in", 13.9f, Points(new Vector3(5.4f, 0f, -70f), new Vector3(5.4f, 0f, -11f), 7));
            Lane nb1In = CreateLane("urban_nb_1_in", 13.9f, Points(new Vector3(1.8f, 0f, -70f), new Vector3(1.8f, 0f, -11f), 7));
            Lane nb0Out = CreateLane("urban_nb_0_out", 13.9f, Points(new Vector3(5.4f, 0f, 11f), new Vector3(5.4f, 0f, 70f), 7));
            Lane nb1Out = CreateLane("urban_nb_1_out", 13.9f, Points(new Vector3(1.8f, 0f, 11f), new Vector3(1.8f, 0f, 70f), 7));
            Lane sb0In = CreateLane("urban_sb_0_in", 13.9f, Points(new Vector3(-5.4f, 0f, 70f), new Vector3(-5.4f, 0f, 11f), 7));
            Lane sb1In = CreateLane("urban_sb_1_in", 13.9f, Points(new Vector3(-1.8f, 0f, 70f), new Vector3(-1.8f, 0f, 11f), 7));
            Lane sb0Out = CreateLane("urban_sb_0_out", 13.9f, Points(new Vector3(-5.4f, 0f, -11f), new Vector3(-5.4f, 0f, -70f), 7));
            Lane sb1Out = CreateLane("urban_sb_1_out", 13.9f, Points(new Vector3(-1.8f, 0f, -11f), new Vector3(-1.8f, 0f, -70f), 7));
            Lane eb0In = CreateLane("urban_eb_0_in", 13.9f, Points(new Vector3(-70f, 0f, -5.4f), new Vector3(-11f, 0f, -5.4f), 7));
            Lane eb1In = CreateLane("urban_eb_1_in", 13.9f, Points(new Vector3(-70f, 0f, -1.8f), new Vector3(-11f, 0f, -1.8f), 7));
            Lane eb0Out = CreateLane("urban_eb_0_out", 13.9f, Points(new Vector3(11f, 0f, -5.4f), new Vector3(70f, 0f, -5.4f), 7));
            Lane eb1Out = CreateLane("urban_eb_1_out", 13.9f, Points(new Vector3(11f, 0f, -1.8f), new Vector3(70f, 0f, -1.8f), 7));
            Lane wb0In = CreateLane("urban_wb_0_in", 13.9f, Points(new Vector3(70f, 0f, 5.4f), new Vector3(11f, 0f, 5.4f), 7));
            Lane wb1In = CreateLane("urban_wb_1_in", 13.9f, Points(new Vector3(70f, 0f, 1.8f), new Vector3(11f, 0f, 1.8f), 7));
            Lane wb0Out = CreateLane("urban_wb_0_out", 13.9f, Points(new Vector3(-11f, 0f, 5.4f), new Vector3(-70f, 0f, 5.4f), 7));
            Lane wb1Out = CreateLane("urban_wb_1_out", 13.9f, Points(new Vector3(-11f, 0f, 1.8f), new Vector3(-70f, 0f, 1.8f), 7));
            lanes.AddRange(new[] { nb0In, nb1In, nb0Out, nb1Out, sb0In, sb1In, sb0Out, sb1Out,
                eb0In, eb1In, eb0Out, eb1Out, wb0In, wb1In, wb0Out, wb1Out });
            nb0In.leftLane = nb1In;
            nb1In.rightLane = nb0In;

            Lane Connect(string id, Lane input, Lane output)
            {
                var connector = CreateLane(id, 9f,
                    Points(input.Centerline()[^1], output.Centerline()[0], 5));
                input.nextLanes.Add(connector);
                connector.nextLanes.Add(output);
                lanes.Add(connector);
                return connector;
            }
            Connect("urban_nb_0_straight", nb0In, nb0Out);
            Connect("urban_nb_1_straight", nb1In, nb1Out);
            Connect("urban_sb_0_straight", sb0In, sb0Out);
            Connect("urban_sb_1_straight", sb1In, sb1Out);
            Connect("urban_eb_0_straight", eb0In, eb0Out);
            Connect("urban_eb_1_straight", eb1In, eb1Out);
            Connect("urban_wb_0_straight", wb0In, wb0Out);
            Connect("urban_wb_1_straight", wb1In, wb1Out);

            var northRight = CreateLane("urban_nb_right", 8f,
                BezierPoints(nb0In.Centerline()[^1], new Vector3(5.4f, 0f, -5.4f), eb0Out.Centerline()[0], 8));
            nb0In.nextLanes.Add(northRight); northRight.nextLanes.Add(eb0Out); lanes.Add(northRight);
            var northLeft = CreateLane("urban_nb_left", 8f,
                BezierPoints(nb1In.Centerline()[^1], new Vector3(1.8f, 0f, 5.4f), wb0Out.Centerline()[0], 16));
            nb1In.nextLanes.Add(northLeft); northLeft.nextLanes.Add(wb0Out); lanes.Add(northLeft);

            // Stripes are perpendicular to the pedestrian walking direction.
            CreateCrosswalk(new Vector3(0f, .02f, -13f), true);
            CreateCrosswalk(new Vector3(0f, .02f, 13f), true);
            CreateCrosswalk(new Vector3(-13f, .02f, 0f), false);
            CreateCrosswalk(new Vector3(13f, .02f, 0f), false);
            var traffic = CreateTrafficLights();

            var vehicles = new[]
            {
                CreateVehicle("urban_ego", new Vector3(5.4f, .5f, -64f), new Vector3(5.4f, 0f, 66f), Color.cyan),
                CreateVehicle("urban_left_turn", new Vector3(1.8f, .5f, -48f), new Vector3(-66f, 0f, 5.4f), new Color(.8f, .35f, 1f)),
                CreateVehicle("urban_right_turn", new Vector3(5.4f, .5f, -38f), new Vector3(66f, 0f, -5.4f), Color.green),
                CreateVehicle("urban_oncoming_0", new Vector3(-5.4f, .5f, 64f), new Vector3(-5.4f, 0f, -66f), Color.yellow),
                CreateVehicle("urban_oncoming_1", new Vector3(-1.8f, .5f, 42f), new Vector3(-1.8f, 0f, -66f), new Color(1f, .5f, .1f)),
                CreateVehicle("urban_eastbound", new Vector3(-64f, .5f, -5.4f), new Vector3(66f, 0f, -5.4f), new Color(.2f, .6f, 1f)),
                CreateVehicle("urban_westbound", new Vector3(64f, .5f, 5.4f), new Vector3(-66f, 0f, 5.4f), new Color(1f, .25f, .2f)),
            };
            vehicles[3].transform.rotation = Quaternion.Euler(0f, 180f, 0f);
            vehicles[4].transform.rotation = Quaternion.Euler(0f, 180f, 0f);
            vehicles[5].transform.rotation = Quaternion.Euler(0f, 90f, 0f);
            vehicles[6].transform.rotation = Quaternion.Euler(0f, 270f, 0f);
            vehicles[1].maneuver = "left";
            vehicles[2].maneuver = "right";
            traffic.focusVehicle = vehicles[0];
            WireSimulation("urban", lanes, vehicles, Array.Empty<DynamicObjectAgent>());
            CreatePedestrianSpawner();
            var controlCanvas = CreateCameraSystem(camera, vehicles[0], false,
                new Vector3(100f, 165f, -100f), Vector3.zero);
            CreateUrbanStrategyControls(controlCanvas.transform, vehicles[0]);

            camera.transform.position = new Vector3(45f, 70f, -55f);
            camera.transform.rotation = Quaternion.Euler(52f, -38f, 0f);
            Save("Urban");
        }

        private static void NewScene(string name, out Camera camera)
        {
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var lightGo = new GameObject("Directional Light");
            var light = lightGo.AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 1.1f;
            lightGo.transform.rotation = Quaternion.Euler(48f, -28f, 0f);

            var cameraGo = new GameObject("Main Camera");
            cameraGo.tag = "MainCamera";
            camera = cameraGo.AddComponent<Camera>();
            camera.farClipPlane = 1000f;
            cameraGo.AddComponent<AudioListener>();
            var brain = cameraGo.AddComponent<CinemachineBrain>();
            brain.DefaultBlend = new CinemachineBlendDefinition(
                CinemachineBlendDefinition.Styles.EaseInOut, .25f);
            RenderSettings.ambientIntensity = .8f;
            SceneManager.SetActiveScene(SceneManager.GetActiveScene());
        }

        private static Lane CreateLane(string id, float speedLimit, IReadOnlyList<Vector3> points)
        {
            var root = new GameObject(id);
            var lane = root.AddComponent<Lane>();
            lane.laneId = id;
            lane.speedLimit = speedLimit;
            for (int i = 0; i < points.Count; i++)
            {
                var wp = new GameObject($"WP_{i:00}");
                wp.transform.SetParent(root.transform);
                wp.transform.position = points[i];
                lane.waypoints.Add(wp.transform);
            }
            return lane;
        }

        private static VehicleController CreateVehicle(string id, Vector3 position, Vector3 goalPosition, Color color)
        {
            var go = new GameObject(id);
            go.transform.position = position;

            string prefabPath = SelectCarPrefabPath(id);
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
            if (prefab != null)
            {
                var visual = (GameObject)PrefabUtility.InstantiatePrefab(prefab, go.transform);
                visual.name = $"Visual_{prefab.name}";
                visual.transform.localPosition = Vector3.down * .5f;
                visual.transform.localRotation = Quaternion.identity;
                ApplyCompatibleCarMaterial(visual, color);
            }
            else
            {
                var visual = GameObject.CreatePrimitive(PrimitiveType.Cube);
                visual.name = "Fallback Vehicle Visual";
                visual.transform.SetParent(go.transform, false);
                visual.transform.localScale = new Vector3(1.8f, .9f, 4.2f);
                ApplyMaterial(visual, color);
            }

            var controller = go.AddComponent<VehicleController>();
            controller.vehicleId = id;
            controller.maxSpeed = 30f;

            var goal = new GameObject($"{id}_Goal");
            goal.transform.position = goalPosition;
            controller.goal = goal.transform;
            controller.ConfigureThroughRoute(goalPosition);

            var line = go.AddComponent<LineRenderer>();
            line.widthMultiplier = .22f;
            var visualizer = go.AddComponent<PathVisualizer>();
            visualizer.vehicle = controller;
            visualizer.color = color;
            return controller;
        }

        private static int StableAssetIndex(string id, int count)
        {
            unchecked
            {
                uint hash = 2166136261;
                foreach (char character in id)
                    hash = (hash ^ character) * 16777619;
                return (int)(hash % (uint)count);
            }
        }

        private static string SelectCarPrefabPath(string id)
        {
            if (id.Contains("left_turn", StringComparison.OrdinalIgnoreCase))
                return CarPrefabPaths[3]; // Taxi: compact wheelbase for the protected turn.
            if (id.Contains("right_turn", StringComparison.OrdinalIgnoreCase))
                return CarPrefabPaths[0]; // Hatchback: avoids a long truck clipping the corner.
            return CarPrefabPaths[StableAssetIndex(id, CarPrefabPaths.Length)];
        }

        private static DynamicObjectAgent CreateDynamicObject(string id, string type, Vector3 position,
            float radius, Color color)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            go.name = id;
            go.transform.position = position;
            go.transform.localScale = new Vector3(.7f, 1f, .7f);
            ApplyMaterial(go, color);
            var agent = go.AddComponent<DynamicObjectAgent>();
            agent.objectId = id;
            agent.objectType = type;
            agent.radius = radius;
            return agent;
        }

        private static void CreatePedestrianSpawner()
        {
            var go = new GameObject("Pedestrian Signal and Spawner");
            var spawner = go.AddComponent<PedestrianSpawner>();
            spawner.signals = UnityEngine.Object.FindFirstObjectByType<TrafficLightSystem>();
            spawner.pedestrianPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(PedestrianAssetPath);
            spawner.pedestrianBodyMaterial =
                AssetDatabase.LoadAssetAtPath<Material>(PedestrianBodyMaterialPath);
            spawner.pedestrianJointMaterial =
                AssetDatabase.LoadAssetAtPath<Material>(PedestrianJointMaterialPath);
            spawner.pedestrianColors = new[]
            {
                new Color(1f, .78f, .05f),
                new Color(1f, .38f, .12f),
                new Color(.12f, .82f, .78f),
                new Color(.48f, .9f, .16f),
                new Color(.72f, .38f, 1f),
                new Color(1f, .58f, .08f),
            };
            spawner.walkingSpeed = 2.5f;
            spawner.pedestrianScale = 1.25f;
            spawner.postCrossingDistance = 3f;
            spawner.postCrossingWait = 2f;
            spawner.routes = new[]
            {
                new PedestrianRoute { spawn = new Vector3(-9f, 1f, -13f), destination = new Vector3(9f, 1f, -13f) },
                new PedestrianRoute { spawn = new Vector3(9f, 1f, 13f), destination = new Vector3(-9f, 1f, 13f) },
                new PedestrianRoute { spawn = new Vector3(-13f, 1f, -9f), destination = new Vector3(-13f, 1f, 9f) },
                new PedestrianRoute { spawn = new Vector3(13f, 1f, 9f), destination = new Vector3(13f, 1f, -9f) },
            };
            var indicators = new List<PedestrianSignalVisual>();
            foreach (var route in spawner.routes)
            {
                indicators.Add(CreatePedestrianSignal(route));
            }
            spawner.pedestrianSignals = indicators.ToArray();
        }

        private static PedestrianSignalVisual CreatePedestrianSignal(PedestrianRoute route)
        {
            Vector3 travel = (route.destination - route.spawn).normalized;
            Vector3 position = route.destination + travel * 1.2f;
            var root = new GameObject("Pedestrian Signal");
            root.transform.position = new Vector3(position.x, 0f, position.z);
            // The signal face is local -Z, aimed back toward the waiting pedestrian.
            root.transform.rotation = Quaternion.LookRotation(travel, Vector3.up);

            var pole = CreateCube("Pedestrian Signal Pole", new Vector3(0f, 1.7f, .42f),
                new Vector3(.2f, 3.4f, .2f), Color.gray);
            pole.transform.SetParent(root.transform, false);
            var bracket = CreateCube("Pedestrian Signal Bracket", new Vector3(0f, 3.1f, .2f),
                new Vector3(.22f, .22f, .55f), Color.gray);
            bracket.transform.SetParent(root.transform, false);
            var housing = CreateCube("Pedestrian Signal Housing", new Vector3(0f, 3.05f, 0f),
                new Vector3(1.15f, 2.15f, .62f), new Color(.02f, .02f, .02f));
            housing.transform.SetParent(root.transform, false);
            foreach (float y in new[] { 3.55f, 2.55f })
            {
                var panel = CreateCube("Icon Panel", new Vector3(0f, y, -.34f),
                    new Vector3(.9f, .85f, .08f), new Color(.008f, .008f, .008f));
                panel.transform.SetParent(root.transform, false);
            }

            Renderer[] HumanIcon(string prefix, float y, Color color, bool walking)
            {
                var parts = new List<Renderer>();
                GameObject Part(string name, PrimitiveType primitive, Vector3 local, Vector3 scale, float roll = 0f)
                {
                    var part = GameObject.CreatePrimitive(primitive);
                    part.name = prefix + " " + name;
                    part.transform.SetParent(root.transform, false);
                    part.transform.localPosition = local;
                    part.transform.localScale = scale;
                    part.transform.localRotation = Quaternion.Euler(0f, 0f, roll);
                    ApplyMaterial(part, color);
                    parts.Add(part.GetComponent<Renderer>());
                    return part;
                }
                Part("Head", PrimitiveType.Sphere, new Vector3(0f, y + .24f, -.41f),
                    Vector3.one * .16f);
                Part("Body", PrimitiveType.Cube, new Vector3(0f, y, -.41f),
                    new Vector3(.14f, .38f, .07f));
                Part("Left Arm", PrimitiveType.Cube, new Vector3(-.15f, y + .02f, -.41f),
                    new Vector3(.28f, .07f, .07f), walking ? -22f : 0f);
                Part("Right Arm", PrimitiveType.Cube, new Vector3(.15f, y + .02f, -.41f),
                    new Vector3(.28f, .07f, .07f), walking ? -22f : 0f);
                Part("Left Leg", PrimitiveType.Cube, new Vector3(-.08f, y - .28f, -.41f),
                    new Vector3(.08f, .34f, .07f), walking ? -24f : 0f);
                Part("Right Leg", PrimitiveType.Cube, new Vector3(.08f, y - .28f, -.41f),
                    new Vector3(.08f, .34f, .07f), walking ? 24f : 0f);
                return parts.ToArray();
            }

            return new PedestrianSignalVisual
            {
                greenIcon = HumanIcon("Green Walk", 3.53f, new Color(.15f, 1f, .4f), true),
                redIcon = HumanIcon("Red Stop", 2.53f, Color.red, false),
            };
        }

        private static void WireSimulation(string scenario, IEnumerable<Lane> lanes,
            IEnumerable<VehicleController> vehicles, IEnumerable<DynamicObjectAgent> objects)
        {
            var managerGo = new GameObject("V2X Runtime");
            var road = managerGo.AddComponent<RoadNetworkManager>();
            road.lanes = new List<Lane>(lanes);
            var sim = managerGo.AddComponent<SimulationManager>();
            sim.scenario = scenario;
            sim.road = road;
            sim.vehicles = new List<VehicleController>(vehicles);
            sim.objects = new List<DynamicObjectAgent>(objects);

            var client = managerGo.AddComponent<V2XClient>();
            client.serverUrl = "ws://localhost:8765";
            client.sendEveryNTicks = 2;
            client.maxCommandLagTicks = 6;
            client.stateProviderSource = sim;
            client.commandSinkSource = sim;

            var dashboard = managerGo.AddComponent<DebugDashboard>();
            dashboard.client = client;
            dashboard.vehicles = sim.vehicles.ToArray();
        }

        private static void CreateRoadRibbon(string name, IReadOnlyList<Vector3> points, float width, Color color)
        {
            var root = new GameObject(name);
            for (int i = 0; i < points.Count - 1; i++)
            {
                var a = points[i];
                var b = points[i + 1];
                var mid = (a + b) * .5f;
                var segment = CreateCube($"Segment_{i:00}", mid + Vector3.down * .1f,
                    new Vector3(width, .2f, Vector3.Distance(a, b) + .5f), color);
                segment.transform.rotation = Quaternion.LookRotation((b - a).normalized, Vector3.up);
                segment.transform.SetParent(root.transform);
            }
        }

        private static void CreateEmergencyLabEnvironment(float zEnd)
        {
            var barrierColor = new Color(.48f, .53f, .61f);
            for (float z = 5f; z <= zEnd; z += 10f)
            {
                CreateCube("Left Guardrail", new Vector3(-6.25f, .42f, z),
                    new Vector3(.18f, .55f, 9.5f), barrierColor);
                CreateCube("Right Guardrail", new Vector3(9.55f, .42f, z),
                    new Vector3(.18f, .55f, 9.5f), barrierColor);
            }
            for (float z = 18f; z <= zEnd; z += 34f)
            {
                foreach (float x in new[] { -8.3f, 11.6f })
                {
                    CreateCube("Smart Road Pole", new Vector3(x, 3.6f, z),
                        new Vector3(.18f, 7.2f, .18f), new Color(.25f, .3f, .36f));
                    var lamp = CreateCube("LED Road Lamp", new Vector3(x, 7.15f, z),
                        new Vector3(.8f, .18f, .35f), new Color(.55f, .85f, 1f));
                    var light = lamp.AddComponent<Light>();
                    light.type = LightType.Point;
                    light.color = new Color(.55f, .8f, 1f);
                    light.range = 20f;
                    light.intensity = 2.5f;
                }
            }

            foreach (float z in new[] { 72f, 185f, 286f })
            {
                var root = new GameObject($"V2X Gantry {z:000}");
                CreateLabGantryPart(root.transform, "Left Post", new Vector3(-6f, 3.6f, 0f),
                    new Vector3(.32f, 7.2f, .32f));
                CreateLabGantryPart(root.transform, "Right Post", new Vector3(9.3f, 3.6f, 0f),
                    new Vector3(.32f, 7.2f, .32f));
                CreateLabGantryPart(root.transform, "Beam", new Vector3(1.65f, 7f, 0f),
                    new Vector3(15.6f, .35f, .35f));
                var sign = CreateCube("V2X Detection Sign", new Vector3(1.65f, 6.25f, -.05f),
                    new Vector3(5.8f, 1.25f, .22f), new Color(.025f, .22f, .34f));
                sign.transform.SetParent(root.transform, false);
                var label = new GameObject("Sign Label");
                label.transform.SetParent(root.transform, false);
                label.transform.localPosition = new Vector3(1.65f, 6.2f, -.18f);
                label.transform.localRotation = Quaternion.Euler(0f, 180f, 0f);
                var text = label.AddComponent<TextMesh>();
                text.text = z < 100f ? "V2X HAZARD LAB" : z < 220f ? "RRT ESCAPE ZONE" : "SAFE REJOIN";
                text.fontSize = 46;
                text.characterSize = .12f;
                text.anchor = TextAnchor.MiddleCenter;
                text.color = new Color(.55f, .95f, 1f);
                root.transform.position = new Vector3(0f, 0f, z);
            }

            for (int i = 0; i < 7; i++)
            {
                float z = 42f + i * 34f;
                var sensor = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sensor.name = "V2X Roadside Sensor";
                sensor.transform.position = new Vector3(10.3f, 1.35f, z);
                sensor.transform.localScale = Vector3.one * .42f;
                ApplyMaterial(sensor, new Color(.15f, .9f, 1f));
            }
        }

        private static void CreateIntegratedCityEnvironment(
            IReadOnlyList<Vector3> boulevard, IReadOnlyList<Vector3> turnEast,
            IReadOnlyList<Vector3> turnSouth, IReadOnlyList<Vector3> south,
            IReadOnlyList<Vector3> turnWest, IReadOnlyList<Vector3> west,
            IReadOnlyList<Vector3> turnNorth, IReadOnlyList<Vector3> cityReturn)
        {
            var ground = CreateCube("Integrated Metro Ground", new Vector3(45f, -.32f, 110f),
                new Vector3(310f, .35f, 700f), new Color(.055f, .085f, .075f));
            ground.transform.SetAsFirstSibling();
            Color asphalt = new Color(.025f, .032f, .043f);
            CreateCube("V2X Boulevard Asphalt", new Vector3(7.2f, -.1f, 215f),
                new Vector3(11.2f, .2f, 290f), asphalt);
            CreateRoadRibbon("Outer Turn East", turnEast, 10f, asphalt);
            CreateRoadRibbon("Outer Turn South", turnSouth, 10f, asphalt);
            CreateRoadRibbon("South Smart Avenue", south, 10f, asphalt);
            CreateRoadRibbon("Outer Turn West", turnWest, 10f, asphalt);
            CreateRoadRibbon("West Connector", west, 10f, asphalt);
            CreateRoadRibbon("Outer Turn North", turnNorth, 10f, asphalt);
            CreateRoadRibbon("Urban Return Avenue", cityReturn, 10f, asphalt);

            // Divider and shoulder edge stop at the taper start (z=320); the
            // merge taper itself is deliberately unmarked, as on a real road.
            CreateDashedLine(7.2f, 70f, 320f);
            CreateSolidLineZ("Boulevard Left Edge", 3.5f, 70f, 360f, Color.white, .16f);
            CreateSolidLineZ("Boulevard Shoulder Edge", 10.8f, 70f, 320f,
                new Color(1f, .78f, .08f), .18f);
            foreach (var route in new[] { turnEast, turnSouth, south, turnWest, west, turnNorth, cityReturn })
                CreatePolylineEdges("Outer Loop Edge", route, 4.65f, Color.white);

            Color concrete = new Color(.22f, .27f, .31f);
            for (float z = 82f; z < 355f; z += 12f)
            {
                CreateCube("Boulevard Left Barrier", new Vector3(2.2f, .35f, z),
                    new Vector3(.22f, .6f, 11.5f), concrete);
                CreateCube("Boulevard Right Barrier", new Vector3(11.9f, .35f, z),
                    new Vector3(.22f, .6f, 11.5f), concrete);
            }

            for (int i = 0; i < 14; i++)
            {
                float z = 92f + i * 23f;
                CreateCityLamp(new Vector3(-.2f, 0f, z), i % 3 == 0);
                CreateCityLamp(new Vector3(14.2f, 0f, z), false);
            }
            for (int i = 0; i < 13; i++)
                CreateCityLamp(new Vector3(121.5f, 0f, 365f - i * 42f), i % 4 == 0);

            foreach (float z in new[] { 130f, 235f, 335f })
                CreateIntegratedGantry(new Vector3(7.2f, 0f, z),
                    z < 200f ? "CENTRAL V2X GRID" : z < 300f ? "RRT EVENT ZONE" : "SMART CITY LOOP");

            Color[] facades =
            {
                new Color(.10f, .19f, .27f), new Color(.18f, .14f, .25f),
                new Color(.13f, .23f, .20f), new Color(.24f, .18f, .13f),
            };
            for (int i = 0; i < 13; i++)
            {
                float z = 92f + i * 25f;
                CreateCityBuilding(new Vector3(-24f - (i % 2) * 10f, 0f, z),
                    new Vector3(18f, 16f + (i % 5) * 6f, 16f), facades[i % facades.Length], i);
                CreateCityBuilding(new Vector3(38f + (i % 3) * 11f, 0f, z + 8f),
                    new Vector3(17f, 20f + (i % 4) * 8f, 18f), facades[(i + 1) % facades.Length], i + 20);
            }
            for (int i = 0; i < 11; i++)
            {
                float z = 365f - i * 48f;
                CreateCityBuilding(new Vector3(145f + (i % 2) * 13f, 0f, z),
                    new Vector3(22f, 24f + (i % 5) * 7f, 25f), facades[(i + 2) % facades.Length], i + 40);
            }

            var park = CreateCube("Mobility Park", new Vector3(63f, -.08f, 334f),
                new Vector3(70f, .12f, 75f), new Color(.045f, .2f, .095f));
            for (int i = 0; i < 18; i++)
            {
                float x = 34f + (i % 6) * 10f;
                float z = 310f + (i / 6) * 18f;
                var trunk = CreateCube("Park Tree Trunk", new Vector3(x, 1.2f, z),
                    new Vector3(.45f, 2.4f, .45f), new Color(.25f, .14f, .07f));
                var crown = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                crown.name = "Park Tree Crown";
                crown.transform.position = new Vector3(x, 3.2f, z);
                crown.transform.localScale = new Vector3(3.6f, 4f, 3.6f);
                ApplyMaterial(crown, new Color(.08f, .38f, .16f));
            }
        }

        private static void CreateCityLamp(Vector3 basePosition, bool connectedLight)
        {
            CreateCube("Smart City Lamp Pole", basePosition + Vector3.up * 3.5f,
                new Vector3(.16f, 7f, .16f), new Color(.24f, .3f, .36f));
            var head = CreateCube("Smart City LED", basePosition + Vector3.up * 7f,
                new Vector3(.75f, .16f, .32f), new Color(.48f, .82f, 1f));
            if (connectedLight)
            {
                var light = head.AddComponent<Light>();
                light.type = LightType.Point;
                light.color = new Color(.48f, .75f, 1f);
                light.range = 24f;
                light.intensity = 2.3f;
            }
        }

        private static void CreateIntegratedGantry(Vector3 position, string caption)
        {
            var root = new GameObject($"Integrated V2X Gantry {caption}");
            root.transform.position = position;
            CreateLabGantryPart(root.transform, "Left Post", new Vector3(-5.2f, 3.6f, 0f),
                new Vector3(.3f, 7.2f, .3f));
            CreateLabGantryPart(root.transform, "Right Post", new Vector3(5.2f, 3.6f, 0f),
                new Vector3(.3f, 7.2f, .3f));
            CreateLabGantryPart(root.transform, "Beam", new Vector3(0f, 7f, 0f),
                new Vector3(10.7f, .32f, .32f));
            var sign = CreateCube("Connected Mobility Sign", new Vector3(0f, 6.25f, 0f),
                new Vector3(6.4f, 1.2f, .2f), new Color(.02f, .18f, .31f));
            sign.transform.SetParent(root.transform, false);
            var label = new GameObject("Gantry Caption");
            label.transform.SetParent(root.transform, false);
            label.transform.localPosition = new Vector3(0f, 6.25f, -.15f);
            label.transform.localRotation = Quaternion.Euler(0f, 180f, 0f);
            var text = label.AddComponent<TextMesh>();
            text.text = caption;
            text.fontSize = 40;
            text.characterSize = .1f;
            text.anchor = TextAnchor.MiddleCenter;
            text.color = new Color(.55f, .95f, 1f);
        }

        private static void CreateCityBuilding(Vector3 basePosition, Vector3 size,
            Color facade, int seed)
        {
            var root = new GameObject($"Connected Building {seed:00}");
            var tower = CreateCube("Tower", basePosition + Vector3.up * (size.y * .5f), size, facade);
            tower.transform.SetParent(root.transform);
            var crown = CreateCube("Illuminated Crown", basePosition + Vector3.up * (size.y + .45f),
                new Vector3(size.x * .82f, .75f, size.z * .82f), new Color(.12f, .6f, .78f));
            crown.transform.SetParent(root.transform);
            for (int floor = 3; floor < size.y - 2f; floor += 4)
            {
                var windows = CreateCube("Window Band", basePosition + new Vector3(0f, floor, -size.z * .505f),
                    new Vector3(size.x * .78f, .42f, .08f), new Color(.55f, .78f, .86f));
                windows.transform.SetParent(root.transform);
            }
        }

        private static Transform CreateCheckpoint(string name, Vector3 position)
        {
            var checkpoint = new GameObject(name);
            checkpoint.transform.position = position;
            return checkpoint.transform;
        }

        private static void CreateLabGantryPart(Transform parent, string name,
            Vector3 localPosition, Vector3 scale)
        {
            var part = CreateCube(name, localPosition, scale, new Color(.28f, .33f, .4f));
            part.transform.SetParent(parent, false);
        }

        private static void CreateEmergencyLabControls(
            Transform canvas, EmergencyAvoidanceScenarioController scenario)
        {
            var obstacle = CreateButton(canvas, "4  낙하물 발생", new Vector2(-285f, 105f), 170f);
            var emergency = CreateButton(canvas, "5  긴급차 출동", new Vector2(-95f, 105f), 170f);
            var planner = CreateButton(canvas, "6  RRT / RRT*", new Vector2(95f, 105f), 170f);
            var reset = CreateButton(canvas, "0  실험 재시작", new Vector2(285f, 105f), 170f);
            UnityEventTools.AddPersistentListener(obstacle.onClick, scenario.SpawnObstacle);
            UnityEventTools.AddPersistentListener(emergency.onClick, scenario.DispatchEmergencyVehicle);
            UnityEventTools.AddPersistentListener(planner.onClick, scenario.TogglePlanner);
            UnityEventTools.AddPersistentListener(reset.onClick, scenario.ResetScenario);
        }

        private static void CreateDashedLine(float x, float z0, float z1)
        {
            var root = new GameObject($"Lane Marking {x:0.0}");
            for (float z = z0 + 3f; z < z1; z += 10f)
            {
                var dash = CreateCube("Dash", new Vector3(x, .02f, z),
                    new Vector3(.12f, .03f, 5f), Color.white);
                dash.transform.SetParent(root.transform);
            }
        }

        private static void CreateSolidLineZ(
            string name, float x, float z0, float z1, Color color, float width = .12f)
        {
            CreateCube(name, new Vector3(x, .028f, (z0 + z1) * .5f),
                new Vector3(width, .035f, Mathf.Abs(z1 - z0)), color);
        }

        private static void CreateSolidLineX(
            string name, float z, float x0, float x1, Color color, float width = .12f)
        {
            CreateCube(name, new Vector3((x0 + x1) * .5f, .029f, z),
                new Vector3(Mathf.Abs(x1 - x0), .035f, width), color);
        }

        private static void CreatePolylineEdges(
            string name, IReadOnlyList<Vector3> points, float halfWidth, Color color)
        {
            var root = new GameObject(name);
            for (int i = 0; i < points.Count - 1; i++)
            {
                Vector3 a = points[i];
                Vector3 b = points[i + 1];
                Vector3 forward = (b - a).normalized;
                Vector3 side = Vector3.Cross(Vector3.up, forward) * halfWidth;
                foreach (float sign in new[] { -1f, 1f })
                {
                    var line = CreateCube("Edge", (a + b) * .5f + side * sign + Vector3.up * .028f,
                        new Vector3(.14f, .035f, Vector3.Distance(a, b) + .15f), color);
                    line.transform.rotation = Quaternion.LookRotation(forward, Vector3.up);
                    line.transform.SetParent(root.transform);
                }
            }
        }

        private static void CreateDashedLineX(float z, float x0, float x1)
        {
            var root = new GameObject($"East-West Lane Marking {z:0.0}");
            for (float x = x0 + 3f; x < x1; x += 10f)
            {
                var dash = CreateCube("Dash", new Vector3(x, .025f, z),
                    new Vector3(5f, .03f, .12f), Color.white);
                dash.transform.SetParent(root.transform);
            }
        }

        private static void CreateUrbanLaneMarkings()
        {
            Color yellow = new Color(1f, .75f, .05f);
            foreach (float x in new[] { -.18f, .18f })
            {
                CreateCube("NS Center Yellow", new Vector3(x, .025f, -42f),
                    new Vector3(.1f, .03f, 56f), yellow);
                CreateCube("NS Center Yellow", new Vector3(x, .025f, 42f),
                    new Vector3(.1f, .03f, 56f), yellow);
            }
            foreach (float z in new[] { -.18f, .18f })
            {
                CreateCube("EW Center Yellow", new Vector3(-42f, .026f, z),
                    new Vector3(56f, .03f, .1f), yellow);
                CreateCube("EW Center Yellow", new Vector3(42f, .026f, z),
                    new Vector3(56f, .03f, .1f), yellow);
            }
            foreach (float x in new[] { -3.6f, 3.6f })
            {
                CreateDashedLine(x, -70f, -15f);
                CreateDashedLine(x, 15f, 70f);
            }
            foreach (float z in new[] { -3.6f, 3.6f })
            {
                CreateDashedLineX(z, -70f, -15f);
                CreateDashedLineX(z, 15f, 70f);
            }
            foreach (float x in new[] { -8.7f, 8.7f })
            {
                CreateSolidLineZ("NS Road Edge", x, -70f, -16f, Color.white, .16f);
                CreateSolidLineZ("NS Road Edge", x, 16f, 70f, Color.white, .16f);
            }
            foreach (float z in new[] { -8.7f, 8.7f })
            {
                CreateSolidLineX("EW Road Edge", z, -70f, -16f, Color.white, .16f);
                CreateSolidLineX("EW Road Edge", z, 16f, 70f, Color.white, .16f);
            }
            CreateCube("NB Stop Line", new Vector3(3.6f, .04f, -16f),
                new Vector3(7.2f, .04f, .35f), Color.white);
            CreateCube("SB Stop Line", new Vector3(-3.6f, .04f, 16f),
                new Vector3(7.2f, .04f, .35f), Color.white);
            CreateCube("EB Stop Line", new Vector3(-16f, .04f, -3.6f),
                new Vector3(.35f, .04f, 7.2f), Color.white);
            CreateCube("WB Stop Line", new Vector3(16f, .04f, 3.6f),
                new Vector3(.35f, .04f, 7.2f), Color.white);
        }

        private static void CreateCrosswalk(Vector3 center, bool alongX)
        {
            var root = new GameObject("Crosswalk");
            for (int i = -8; i <= 8; i++)
            {
                var scale = alongX ? new Vector3(.65f, .03f, 4f) : new Vector3(4f, .03f, .65f);
                var offset = alongX ? new Vector3(i * 1.1f, 0f, 0f) : new Vector3(0f, 0f, i * 1.1f);
                var stripe = CreateCube("Stripe", center + offset, scale, Color.white);
                stripe.transform.SetParent(root.transform);
            }
        }

        private static TrafficLightSystem CreateTrafficLights()
        {
            var systemGo = new GameObject("Traffic Light System");
            var system = systemGo.AddComponent<TrafficLightSystem>();
            // Phase timings are not configured here: they must match the
            // server's plan, so they live in TrafficLightSystem next to the
            // comment that says so.
            system.heads = new[]
            {
                CreateSignalGantry("NS South", new Vector3(10.5f, 0f, -18f), 0f),
                CreateSignalGantry("NS North", new Vector3(-10.5f, 0f, 18f), 180f),
                CreateSignalGantry("EW West", new Vector3(-18f, 0f, -10.5f), 90f),
                CreateSignalGantry("EW East", new Vector3(18f, 0f, 10.5f), 270f),
            };
            return system;
        }

        private static TrafficSignalHead CreateSignalGantry(
            string label, Vector3 polePosition, float yaw)
        {
            var root = new GameObject($"Signal {label}");
            root.transform.position = polePosition;
            root.transform.rotation = Quaternion.Euler(0f, yaw, 0f);
            var pole = CreateCube("L Pole", new Vector3(0f, 3.5f, 0f),
                new Vector3(.38f, 7f, .38f), Color.gray);
            pole.transform.SetParent(root.transform, false);
            var arm = CreateCube("L Horizontal Arm", new Vector3(-3.2f, 7f, 0f),
                new Vector3(6.4f, .28f, .28f), Color.gray);
            arm.transform.SetParent(root.transform, false);
            var hanger = CreateCube("Signal Hanger", new Vector3(-6.2f, 6.65f, 0f),
                new Vector3(.2f, .8f, .2f), Color.gray);
            hanger.transform.SetParent(root.transform, false);
            var housing = CreateCube("Integrated Four-Lamp Housing", new Vector3(-6.2f, 6.1f, 0f),
                new Vector3(5.25f, 1.65f, 1.05f), new Color(.025f, .025f, .025f));
            housing.transform.SetParent(root.transform, false);

            Renderer Bulb(string name, float x, Color color)
            {
                var bulb = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                bulb.name = name;
                bulb.transform.SetParent(root.transform, false);
                bulb.transform.localPosition = new Vector3(-6.2f + x, 6.1f, -.6f);
                bulb.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
                bulb.transform.localScale = new Vector3(.5f, .12f, .5f);
                ApplyMaterial(bulb, color);
                return bulb.GetComponent<Renderer>();
            }

            // The third aperture stays black; an emissive three-piece arrow
            // inside it is toggled for the protected-left phase.
            Bulb("Left Arrow Aperture", .65f, new Color(.015f, .015f, .015f));
            var arrowRenderers = new List<Renderer>();
            GameObject ArrowPart(string name, Vector3 localPosition, Vector3 scale, float roll)
            {
                var part = CreateCube(name, localPosition, scale, new Color(.15f, 1f, .45f));
                part.transform.SetParent(root.transform, false);
                part.transform.localRotation = Quaternion.Euler(0f, 0f, roll);
                arrowRenderers.Add(part.GetComponent<Renderer>());
                return part;
            }
            float arrowX = -6.2f + .65f;
            ArrowPart("Left Arrow Shaft", new Vector3(arrowX + .08f, 6.1f, -.69f),
                new Vector3(.65f, .12f, .08f), 0f);
            ArrowPart("Left Arrow Upper", new Vector3(arrowX - .22f, 6.28f, -.69f),
                new Vector3(.42f, .12f, .08f), 45f);
            ArrowPart("Left Arrow Lower", new Vector3(arrowX - .22f, 5.92f, -.69f),
                new Vector3(.42f, .12f, .08f), -45f);

            return new TrafficSignalHead
            {
                label = label,
                red = Bulb("Red", -1.8f, Color.red),
                yellow = Bulb("Yellow", -.6f, Color.yellow),
                left = arrowRenderers.ToArray(),
                green = Bulb("Green", 1.85f, Color.green),
            };
        }

        private static Canvas CreateCameraSystem(
            Camera outputCamera, VehicleController target, bool highwayControls = false,
            Vector3? overviewPosition = null, Vector3? overviewLookAt = null)
        {
            var followAnchor = new GameObject("Camera Follow Target").transform;
            followAnchor.SetParent(target.transform);
            followAnchor.localPosition = new Vector3(0f, 1f, 0f);
            followAnchor.localRotation = Quaternion.identity;
            var lookAnchor = new GameObject("Camera Look Target").transform;
            lookAnchor.SetParent(target.transform);
            lookAnchor.localPosition = new Vector3(0f, 1.2f, 14f);
            lookAnchor.localRotation = Quaternion.identity;

            // Order matters: CameraViewController selects index 0 on Awake and
            // binds keys 1/2/3 to the slots in order, so the chase camera comes
            // first — that is the shot a scene should open on.
            var cameras = new[]
            {
                CreateCinemachineCamera("CM Vehicle", followAnchor, lookAnchor,
                    new Vector3(0f, 4.2f, -11f), 58f, BindingMode.LockToTargetWithWorldUp),
                CreateCinemachineCamera("CM Driver", followAnchor, lookAnchor,
                    new Vector3(0f, 1.45f, .55f), 72f, BindingMode.LockToTargetWithWorldUp),
                CreateOverviewCamera("CM Overview",
                    overviewPosition ?? target.transform.position + new Vector3(50f, 90f, -50f),
                    overviewLookAt ?? target.transform.position, 68f),
            };

            var canvasGo = new GameObject("V2X Control Canvas");
            var canvas = canvasGo.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            var scaler = canvasGo.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);
            scaler.matchWidthOrHeight = .5f;
            canvasGo.AddComponent<GraphicRaycaster>();

            if (UnityEngine.Object.FindFirstObjectByType<EventSystem>() == null)
            {
                var eventGo = new GameObject("EventSystem");
                eventGo.AddComponent<EventSystem>();
                eventGo.AddComponent<InputSystemUIInputModule>();
            }

            var cameraButtons = new[]
            {
                CreateButton(canvas.transform, "1 차량", new Vector2(-130f, 38f)),
                CreateButton(canvas.transform, "2 운전자", new Vector2(0f, 38f)),
                CreateButton(canvas.transform, "3 전경", new Vector2(130f, 38f)),
            };
            var view = canvasGo.AddComponent<CameraViewController>();
            view.cameras = cameras;
            view.buttons = cameraButtons;

            if (highwayControls)
            {
                var left = CreateButton(canvas.transform, "Q  ← 차선", new Vector2(-310f, 105f), 150f);
                var right = CreateButton(canvas.transform, "차선 →  E", new Vector2(310f, 105f), 150f);
                var lane = canvasGo.AddComponent<HighwayLaneChangeController>();
                lane.ego = target;
                lane.road = UnityEngine.Object.FindFirstObjectByType<RoadNetworkManager>();
                lane.leftButton = left;
                lane.rightButton = right;
            }
            CreateReturnToHubControl(canvas);
            CreateRetryPanel(canvas, target);
            return canvas;
        }

        /// <summary>Every demo scene is entered from the hub, so every demo
        /// scene gets a way back out of it.</summary>
        private static void CreateReturnToHubControl(Canvas canvas)
        {
            var button = CreateButton(canvas.transform, "← 허브 (Esc)",
                Vector2.zero, 150f);
            var rect = button.GetComponent<RectTransform>();
            rect.anchorMin = rect.anchorMax = new Vector2(1f, 1f);
            rect.pivot = new Vector2(1f, 1f);
            rect.anchoredPosition = new Vector2(-16f, -16f);
            var hub = canvas.gameObject.AddComponent<ReturnToHubController>();
            hub.hubSceneName = "Main";
            hub.returnButton = button;
        }

        private static void CreateRetryPanel(Canvas canvas, VehicleController target)
        {
            var panel = new GameObject("Retry Panel");
            panel.transform.SetParent(canvas.transform, false);
            var panelRect = panel.AddComponent<RectTransform>();
            panelRect.anchorMin = panelRect.anchorMax = new Vector2(.5f, .5f);
            panelRect.sizeDelta = new Vector2(360f, 170f);
            var panelImage = panel.AddComponent<Image>();
            panelImage.sprite = GetRoundedUiSprite();
            panelImage.type = Image.Type.Sliced;
            panelImage.color = AppleWhite;

            var messageGo = new GameObject("Message");
            messageGo.transform.SetParent(panel.transform, false);
            var messageRect = messageGo.AddComponent<RectTransform>();
            messageRect.anchorMin = messageRect.anchorMax = new Vector2(.5f, .5f);
            messageRect.anchoredPosition = new Vector2(0f, 36f);
            messageRect.sizeDelta = new Vector2(320f, 44f);
            var message = messageGo.AddComponent<Text>();
            message.text = "Reference vehicle reached the exit";
            message.alignment = TextAnchor.MiddleCenter;
            message.fontSize = 19;
            message.color = AppleInk;
            ApplyTypography(message, false);

            var retryButton = CreateButton(panel.transform, "RETRY", new Vector2(0f, -38f), 170f);
            StylePrimaryButton(retryButton);
            var buttonRect = retryButton.GetComponent<RectTransform>();
            buttonRect.anchorMin = buttonRect.anchorMax = new Vector2(.5f, .5f);

            var controller = canvas.gameObject.AddComponent<RetryPanelController>();
            controller.panel = panel;
            controller.retryButton = retryButton;
            target.showRetryOnExit = true;
            target.retryUI = controller;
            panel.SetActive(false);
        }

        private static void CreateUrbanStrategyControls(Transform canvas, VehicleController ego)
        {
            var group = canvas.gameObject.AddComponent<ToggleGroup>();
            var straight = CreateToggle(canvas, group, "직진", new Vector2(-190f, 105f));
            var left = CreateToggle(canvas, group, "좌회전", new Vector2(0f, 105f));
            var right = CreateToggle(canvas, group, "우회전", new Vector2(190f, 105f));
            straight.isOn = true;

            var statusGo = new GameObject("Driving Strategy Status");
            statusGo.transform.SetParent(canvas, false);
            var rect = statusGo.AddComponent<RectTransform>();
            rect.anchorMin = rect.anchorMax = new Vector2(.5f, 0f);
            rect.anchoredPosition = new Vector2(0f, 150f);
            rect.sizeDelta = new Vector2(520f, 34f);
            var status = statusGo.AddComponent<Text>();
            status.alignment = TextAnchor.MiddleCenter;
            status.fontSize = 17;
            status.color = AppleInk;
            ApplyTypography(status, true);

            var strategy = canvas.gameObject.AddComponent<UrbanDrivingStrategyController>();
            strategy.ego = ego;
            strategy.straightToggle = straight;
            strategy.leftToggle = left;
            strategy.rightToggle = right;
            strategy.status = status;
        }

        private static Toggle CreateToggle(
            Transform parent, ToggleGroup group, string label, Vector2 position)
        {
            var root = new GameObject($"Strategy {label}");
            root.transform.SetParent(parent, false);
            var rect = root.AddComponent<RectTransform>();
            rect.anchorMin = rect.anchorMax = new Vector2(.5f, 0f);
            rect.anchoredPosition = position;
            rect.sizeDelta = new Vector2(160f, 42f);
            var background = root.AddComponent<Image>();
            background.sprite = GetRoundedUiSprite();
            background.type = Image.Type.Sliced;
            background.color = AppleWhite;
            var toggle = root.AddComponent<Toggle>();
            toggle.group = group;
            toggle.targetGraphic = background;

            var check = new GameObject("Selected");
            check.transform.SetParent(root.transform, false);
            var checkRect = check.AddComponent<RectTransform>();
            checkRect.anchorMin = Vector2.zero;
            checkRect.anchorMax = Vector2.one;
            checkRect.offsetMin = new Vector2(3f, 3f);
            checkRect.offsetMax = new Vector2(-3f, -3f);
            var checkImage = check.AddComponent<Image>();
            checkImage.sprite = GetRoundedUiSprite();
            checkImage.type = Image.Type.Sliced;
            checkImage.color = AppleBlue;
            toggle.graphic = checkImage;

            var textGo = new GameObject("Label");
            textGo.transform.SetParent(root.transform, false);
            var textRect = textGo.AddComponent<RectTransform>();
            textRect.anchorMin = Vector2.zero;
            textRect.anchorMax = Vector2.one;
            textRect.offsetMin = textRect.offsetMax = Vector2.zero;
            var text = textGo.AddComponent<Text>();
            text.text = label;
            text.alignment = TextAnchor.MiddleCenter;
            text.fontSize = 18;
            text.color = AppleInk;
            ApplyTypography(text, true);
            return toggle;
        }

        private static CinemachineCamera CreateCinemachineCamera(
            string name, Transform followTarget, Transform lookTarget,
            Vector3 offset, float fieldOfView, BindingMode binding)
        {
            var go = new GameObject(name);
            var camera = go.AddComponent<CinemachineCamera>();
            camera.Follow = followTarget;
            camera.LookAt = lookTarget;
            camera.Lens.FieldOfView = fieldOfView;
            camera.Priority = 0;
            var follow = go.AddComponent<CinemachineFollow>();
            follow.FollowOffset = offset;
            follow.TrackerSettings.BindingMode = binding;
            follow.TrackerSettings.PositionDamping = new Vector3(.35f, .5f, .35f);
            var aim = go.AddComponent<CinemachineRotationComposer>();
            aim.Damping = new Vector2(.3f, .3f);
            return camera;
        }

        private static CinemachineCamera CreateOverviewCamera(
            string name, Vector3 position, Vector3 lookAt, float fieldOfView)
        {
            var target = new GameObject(name + " Look Target");
            target.transform.position = lookAt;
            var go = new GameObject(name);
            go.transform.position = position;
            var camera = go.AddComponent<CinemachineCamera>();
            camera.LookAt = target.transform;
            camera.Lens.FieldOfView = fieldOfView;
            camera.Priority = 0;
            var aim = go.AddComponent<CinemachineRotationComposer>();
            aim.Damping = Vector2.zero;
            return camera;
        }

        // ---- Main hub widgets ------------------------------------------- //
        /// <param name="anchorY">0 = pinned to the bottom edge, .5 = centre,
        /// 1 = pinned to the top edge. Explicit so the layout survives a
        /// window resize instead of drifting.</param>
        private static Text CreateHubLabel(
            Transform parent, string name, string content, Vector2 position,
            Vector2 size, int fontSize, TextAnchor anchor, float anchorY = .5f,
            Color? color = null, FontStyle fontStyle = FontStyle.Normal)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            var rect = go.AddComponent<RectTransform>();
            rect.anchorMin = rect.anchorMax = new Vector2(.5f, anchorY);
            rect.pivot = new Vector2(.5f, .5f);
            rect.anchoredPosition = position;
            rect.sizeDelta = size;
            var text = go.AddComponent<Text>();
            text.text = content;
            text.alignment = anchor;
            text.fontSize = fontSize;
            text.color = color ?? AppleInk;
            text.fontStyle = fontStyle;
            text.horizontalOverflow = HorizontalWrapMode.Wrap;
            text.verticalOverflow = VerticalWrapMode.Overflow;
            ApplyTypography(text, fontStyle == FontStyle.Bold);
            return text;
        }

        private static Transform CreateHubPanel(
            Transform parent, string name, Vector2 position, Vector2 size)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            var rect = go.AddComponent<RectTransform>();
            rect.anchorMin = rect.anchorMax = new Vector2(.5f, .5f);
            rect.anchoredPosition = position;
            rect.sizeDelta = size;
            var image = go.AddComponent<Image>();
            image.sprite = GetRoundedUiSprite();
            image.type = Image.Type.Sliced;
            image.color = AppleWhite;
            return go.transform;
        }

        private static Button CreateHubMenuButton(
            Transform parent, string label, Vector2 position)
        {
            var button = CreateButton(parent, label, Vector2.zero, 380f);
            var rect = button.GetComponent<RectTransform>();
            rect.anchorMin = rect.anchorMax = new Vector2(.5f, .5f);
            rect.anchoredPosition = position;
            rect.sizeDelta = new Vector2(380f, 54f);
            var text = button.GetComponentInChildren<Text>();
            if (text != null)
            {
                text.alignment = TextAnchor.MiddleLeft;
                text.fontSize = 21;
                ApplyTypography(text, true);
                var textRect = text.GetComponent<RectTransform>();
                textRect.offsetMin = new Vector2(18f, 0f);
            }
            return button;
        }

        private static Button CreateButton(
            Transform parent, string label, Vector2 position, float width = 120f)
        {
            var go = new GameObject($"Button {label}");
            go.transform.SetParent(parent, false);
            var rect = go.AddComponent<RectTransform>();
            rect.anchorMin = new Vector2(.5f, 0f);
            rect.anchorMax = new Vector2(.5f, 0f);
            rect.pivot = new Vector2(.5f, .5f);
            rect.anchoredPosition = position;
            rect.sizeDelta = new Vector2(width, 44f);
            var image = go.AddComponent<Image>();
            image.sprite = GetRoundedUiSprite();
            image.type = Image.Type.Sliced;
            image.color = new Color(1f, 1f, 1f, .94f);
            var button = go.AddComponent<Button>();
            button.targetGraphic = image;
            var colors = button.colors;
            colors.normalColor = AppleWhite;
            colors.highlightedColor = AppleParchment;
            colors.pressedColor = new Color(.88f, .88f, .9f, 1f);
            colors.selectedColor = AppleParchment;
            colors.disabledColor = new Color(.82f, .82f, .84f, .72f);
            colors.colorMultiplier = 1f;
            button.colors = colors;

            var textGo = new GameObject("Label");
            textGo.transform.SetParent(go.transform, false);
            var textRect = textGo.AddComponent<RectTransform>();
            textRect.anchorMin = Vector2.zero;
            textRect.anchorMax = Vector2.one;
            textRect.offsetMin = Vector2.zero;
            textRect.offsetMax = Vector2.zero;
            var text = textGo.AddComponent<Text>();
            text.text = label;
            text.alignment = TextAnchor.MiddleCenter;
            text.fontSize = 18;
            text.color = AppleInk;
            ApplyTypography(text, true);
            return button;
        }

        private static void StylePrimaryButton(Button button)
        {
            if (button == null) return;
            var image = button.targetGraphic as Image;
            if (image != null) image.color = AppleBlue;
            var colors = button.colors;
            colors.normalColor = AppleBlue;
            colors.highlightedColor = AppleBluePressed;
            colors.pressedColor = new Color(0f, .32f, .68f, 1f);
            colors.selectedColor = AppleBluePressed;
            colors.disabledColor = new Color(.62f, .72f, .84f, .7f);
            button.colors = colors;
            var text = button.GetComponentInChildren<Text>();
            if (text != null)
            {
                text.color = AppleWhite;
                ApplyTypography(text, true);
            }
        }

        private static void StyleSecondaryButton(Button button)
        {
            if (button == null) return;
            if (button.targetGraphic is Image image)
            {
                image.sprite = GetRoundedUiSprite();
                image.type = Image.Type.Sliced;
                image.color = AppleWhite;
            }
            var colors = button.colors;
            colors.normalColor = AppleWhite;
            colors.highlightedColor = AppleParchment;
            colors.pressedColor = new Color(.88f, .88f, .9f, 1f);
            colors.selectedColor = AppleParchment;
            colors.disabledColor = new Color(.82f, .82f, .84f, .72f);
            colors.colorMultiplier = 1f;
            button.colors = colors;
            var text = button.GetComponentInChildren<Text>(true);
            if (text != null)
            {
                text.color = AppleInk;
                ApplyTypography(text, true);
            }
        }

        private static void ApplyTypography(Text text, bool emphasized)
        {
            if (text == null) return;
            var preferred = AssetDatabase.LoadAssetAtPath<Font>(
                emphasized ? SemiboldFontAssetPath : RegularFontAssetPath);
            text.font = preferred ?? Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            text.fontStyle = FontStyle.Normal;
            text.fontSize = Mathf.Max(text.fontSize, emphasized ? 18 : 16);
            text.lineSpacing = 1.08f;
            text.resizeTextForBestFit = false;
        }

        private static Sprite GetRoundedUiSprite()
        {
            foreach (var asset in AssetDatabase.LoadAllAssetsAtPath(RoundedUiAssetPath))
                if (asset is Sprite existing) return existing;

            const int size = 64;
            const float radius = 18f;
            var texture = new Texture2D(size, size, TextureFormat.RGBA32, false)
            {
                name = "V2X UI Rounded Texture",
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
            };
            var pixels = new Color[size * size];
            for (int y = 0; y < size; y++)
            for (int x = 0; x < size; x++)
            {
                float dx = Mathf.Max(radius - x - .5f, 0f, x + .5f - (size - radius));
                float dy = Mathf.Max(radius - y - .5f, 0f, y + .5f - (size - radius));
                float alpha = Mathf.Clamp01(radius + .5f - Mathf.Sqrt(dx * dx + dy * dy));
                pixels[y * size + x] = new Color(1f, 1f, 1f, alpha);
            }
            texture.SetPixels(pixels);
            texture.Apply();
            AssetDatabase.CreateAsset(texture, RoundedUiAssetPath);
            var sprite = Sprite.Create(texture, new Rect(0f, 0f, size, size),
                new Vector2(.5f, .5f), 100f, 0, SpriteMeshType.FullRect,
                new Vector4(radius, radius, radius, radius));
            sprite.name = "V2X UI Rounded Sprite";
            AssetDatabase.AddObjectToAsset(sprite, texture);
            AssetDatabase.SaveAssets();
            return sprite;
        }

        private static GameObject CreateCube(string name, Vector3 position, Vector3 scale, Color color)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = name;
            go.transform.position = position;
            go.transform.localScale = scale;
            ApplyMaterial(go, color);
            return go;
        }

        private static void ApplyMaterial(GameObject go, Color color)
        {
            string key = ColorUtility.ToHtmlStringRGB(color);
            string path = $"{MaterialDir}/V2X_{key}.mat";
            var material = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (material == null)
            {
                var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
                material = new Material(shader) { color = color };
                AssetDatabase.CreateAsset(material, path);
            }
            var renderer = go.GetComponent<Renderer>();
            if (renderer != null) renderer.sharedMaterial = material;
        }

        private static void ApplyCompatibleCarMaterial(GameObject visual, Color color)
        {
            var source = AssetDatabase.LoadAssetAtPath<Material>(CarPaletteMaterialPath);
            Texture palette = source != null
                ? source.GetTexture("_BaseMap") ?? source.mainTexture
                : null;
            var material = GetGeneratedMaterial(
                $"V2X_FreeCar_{ColorUtility.ToHtmlStringRGB(color)}",
                color, palette, .29f, .25f);

            foreach (var renderer in visual.GetComponentsInChildren<Renderer>(true))
            {
                var materials = renderer.sharedMaterials;
                for (int i = 0; i < materials.Length; i++) materials[i] = material;
                renderer.sharedMaterials = materials;
            }
        }

        private static Material GetGeneratedMaterial(string name, Color color,
            Texture texture = null, float metallic = 0f, float smoothness = .2f)
        {
            string path = $"{MaterialDir}/{name}.mat";
            var material = AssetDatabase.LoadAssetAtPath<Material>(path);
            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            if (material == null)
            {
                material = new Material(shader);
                AssetDatabase.CreateAsset(material, path);
            }

            material.shader = shader;
            material.color = color;
            if (material.HasProperty("_BaseMap")) material.SetTexture("_BaseMap", texture);
            if (material.HasProperty("_MainTex")) material.SetTexture("_MainTex", texture);
            if (material.HasProperty("_Metallic")) material.SetFloat("_Metallic", metallic);
            if (material.HasProperty("_Smoothness")) material.SetFloat("_Smoothness", smoothness);
            EditorUtility.SetDirty(material);
            return material;
        }

        private static List<Vector3> Points(Vector3 start, Vector3 end, int count)
        {
            var result = new List<Vector3>(count);
            for (int i = 0; i < count; i++) result.Add(Vector3.Lerp(start, end, i / (count - 1f)));
            return result;
        }

        private static List<Vector3> BezierPoints(
            Vector3 start, Vector3 control, Vector3 end, int count)
        {
            var result = new List<Vector3>(count);
            for (int i = 0; i < count; i++)
            {
                float t = i / (count - 1f);
                float u = 1f - t;
                result.Add(u * u * start + 2f * u * t * control + t * t * end);
            }
            return result;
        }

        private static void EnsureDirectories()
        {
            Directory.CreateDirectory(Path.Combine(Application.dataPath, "Scenes"));
            Directory.CreateDirectory(Path.Combine(Application.dataPath, "Generated", "Materials"));
            AssetDatabase.Refresh();
        }

        private static void Save(string sceneName, bool exportLanes = true)
        {
            EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());
            if (!EditorSceneManager.SaveScene(SceneManager.GetActiveScene(), $"{SceneDir}/{sceneName}.unity"))
                throw new InvalidOperationException($"Failed to save scene {sceneName}");
            // A menu scene has no road; the exporter would throw on it, and any
            // export left over from an earlier build would now be a lie.
            if (!exportLanes)
            {
                DeleteStaleLaneExport(sceneName);
                return;
            }
            LaneNetworkExporter.ExportToDefaultLocation();
        }

        private static void DeleteStaleLaneExport(string sceneName)
        {
            string projectRoot = Directory.GetParent(Application.dataPath)?.Parent?.FullName;
            if (projectRoot == null) return;
            string export = Path.Combine(
                projectRoot, "server", "scenarios", $"{sceneName}_lanes.json");
            if (!File.Exists(export)) return;
            File.Delete(export);
            Debug.Log($"[V2XSceneBuilder] removed lane export for lane-less scene " +
                      $"'{sceneName}': {export}");
        }

        private static void AddSceneToBuildSettings(string path)
        {
            var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
            if (scenes.Exists(scene => scene.path == path)) return;
            scenes.Add(new EditorBuildSettingsScene(path, true));
            EditorBuildSettings.scenes = scenes.ToArray();
        }
    }
}
