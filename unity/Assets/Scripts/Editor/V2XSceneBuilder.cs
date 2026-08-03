using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
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

        [MenuItem("V2X/Build All Demo Scenes")]
        public static void BuildAllScenes()
        {
            EnsureDirectories();
            BuildMainScene();
            BuildLkaScene();
            BuildHighwayScene();
            BuildUrbanScene();
            EditorBuildSettings.scenes = new[]
            {
                new EditorBuildSettingsScene($"{SceneDir}/Main.unity", true),
                new EditorBuildSettingsScene($"{SceneDir}/LKA_Test.unity", true),
                new EditorBuildSettingsScene($"{SceneDir}/Highway.unity", true),
                new EditorBuildSettingsScene($"{SceneDir}/Urban.unity", true),
            };
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            EditorSceneManager.OpenScene($"{SceneDir}/Main.unity");
            Debug.Log("[V2XSceneBuilder] Built Main, LKA_Test, Highway, and Urban scenes.");
        }

        [MenuItem("V2X/Build Main Vertical Slice")]
        public static void BuildMainScene()
        {
            EnsureDirectories();
            NewScene("Main", out var camera);

            var roadSurface = CreateCube("Road", new Vector3(0f, -0.1f, 70f),
                new Vector3(7f, 0.2f, 150f), Color.black);
            roadSurface.transform.SetAsFirstSibling();
            CreateLaneMarkers(0f, 0f, 140f);

            var lane = CreateLane("main_l0", 13.9f,
                Points(new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 140f), 8));
            var vehicle = CreateVehicle("ego", new Vector3(0f, 0.5f, 2f),
                new Vector3(0f, 0f, 132f), Color.cyan);
            WireSimulation("highway", new[] { lane }, new[] { vehicle }, Array.Empty<DynamicObjectAgent>());
            CreateCameraSystem(camera, vehicle, false,
                new Vector3(45f, 85f, -20f), new Vector3(0f, 0f, 70f));

            camera.transform.position = new Vector3(0f, 20f, -22f);
            camera.transform.rotation = Quaternion.Euler(28f, 0f, 0f);
            Save("Main");
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
                BezierPoints(nb1In.Centerline()[^1], new Vector3(1.8f, 0f, 5.4f), wb0Out.Centerline()[0], 10));
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
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = id;
            go.transform.position = position;
            go.transform.localScale = new Vector3(1.8f, .9f, 4.2f);
            ApplyMaterial(go, color);
            var controller = go.AddComponent<VehicleController>();
            controller.vehicleId = id;
            controller.maxSpeed = 30f;

            var goal = new GameObject($"{id}_Goal");
            goal.transform.position = goalPosition;
            controller.goal = goal.transform;

            var line = go.AddComponent<LineRenderer>();
            line.widthMultiplier = .22f;
            var visualizer = go.AddComponent<PathVisualizer>();
            visualizer.vehicle = controller;
            return controller;
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
            spawner.walkingSpeed = 2.5f;
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

        private static void CreateLaneMarkers(float x, float z0, float z1)
        {
            CreateDashedLine(x - 3.5f, z0, z1);
            CreateDashedLine(x + 3.5f, z0, z1);
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
            system.greenTime = 12f;
            system.yellowTime = 3f;
            system.redTime = 31f;
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

            var cameras = new[]
            {
                CreateCinemachineCamera("CM Driver", followAnchor, lookAnchor,
                    new Vector3(0f, 1.45f, .55f), 72f, BindingMode.LockToTargetWithWorldUp),
                CreateCinemachineCamera("CM Vehicle", followAnchor, lookAnchor,
                    new Vector3(0f, 4.2f, -11f), 58f, BindingMode.LockToTargetWithWorldUp),
                CreateOverviewCamera("CM Overview",
                    overviewPosition ?? target.transform.position + new Vector3(50f, 90f, -50f),
                    overviewLookAt ?? target.transform.position, 68f),
            };

            var canvasGo = new GameObject("V2X Control Canvas");
            var canvas = canvasGo.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvasGo.AddComponent<CanvasScaler>().uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            canvasGo.AddComponent<GraphicRaycaster>();

            if (UnityEngine.Object.FindFirstObjectByType<EventSystem>() == null)
            {
                var eventGo = new GameObject("EventSystem");
                eventGo.AddComponent<EventSystem>();
                eventGo.AddComponent<InputSystemUIInputModule>();
            }

            var cameraButtons = new[]
            {
                CreateButton(canvas.transform, "1 운전자", new Vector2(-130f, 38f)),
                CreateButton(canvas.transform, "2 차량", new Vector2(0f, 38f)),
                CreateButton(canvas.transform, "3 전경", new Vector2(130f, 38f)),
            };
            var view = canvasGo.AddComponent<CameraViewController>();
            view.cameras = cameras;
            view.buttons = cameraButtons;

            if (highwayControls)
            {
                var left = CreateButton(canvas.transform, "Q  ← 차선", new Vector2(-310f, 105f), 150f);
                var right = CreateButton(canvas.transform, "차선 →  E", new Vector2(310f, 105f), 150f);
                var automatic = CreateButton(canvas.transform, "R 자동 ON", new Vector2(0f, 105f), 150f);
                var lane = canvasGo.AddComponent<HighwayLaneChangeController>();
                lane.ego = target;
                lane.road = UnityEngine.Object.FindFirstObjectByType<RoadNetworkManager>();
                lane.leftButton = left;
                lane.rightButton = right;
                lane.automaticButton = automatic;
                lane.automaticInterval = 12f;
            }
            return canvas;
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
            status.color = Color.white;
            status.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");

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
            background.color = new Color(.08f, .12f, .18f, .92f);
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
            checkImage.color = new Color(.1f, .55f, .85f, .7f);
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
            text.color = Color.white;
            text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
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
            image.color = new Color(.08f, .12f, .18f, .92f);
            var button = go.AddComponent<Button>();
            button.targetGraphic = image;

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
            text.color = Color.white;
            text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            return button;
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

        private static void Save(string sceneName)
        {
            EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());
            if (!EditorSceneManager.SaveScene(SceneManager.GetActiveScene(), $"{SceneDir}/{sceneName}.unity"))
                throw new InvalidOperationException($"Failed to save scene {sceneName}");
            LaneNetworkExporter.ExportToDefaultLocation();
        }
    }
}
