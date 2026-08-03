using System;
using System.Collections.Generic;

// Wire-format mirrors shared/protocol/*.schema.json.
// If you change a schema, change this file too (and the Python side).
// Serialized with Newtonsoft.Json (V2XClient) — NOT JsonUtility, which cannot
// handle the jagged arrays (List<float[]> path / vec3 lists) in this format.
// Field names match the JSON keys exactly; vec3 is float[3] = [x, y, z].
// Defaults are schema-safe so a freshly-constructed message always validates.

namespace V2X.Protocol
{
    [Serializable]
    public class StateMessage
    {
        public float time;
        public int tick;
        public string scenario = "highway"; // "highway" | "urban" | "lka_test"
        public List<VehicleState> vehicles = new();
        public List<MovingObject> objects = new();
        public List<WorldEvent> events = new();
    }

    [Serializable]
    public class VehicleState
    {
        public string id;
        public string type = "car";
        public float[] position = new float[3];     // [x, y, z]
        public float[] velocity = new float[3];     // [x, y, z]
        public float[] acceleration = new float[3]; // [x, y, z]
        public float heading;              // yaw degrees, 0=+Z, CW
        public string current_lane = "";   // required string; never null on the wire
        public string target_lane;         // nullable per schema
        public string maneuver = "straight";
        public bool has_goal;              // true => server should route to `goal`
        public float[] goal = new float[3];// destination [x,y,z], meaningful iff has_goal
        public string behavior_state = "LaneKeeping";
    }

    [Serializable]
    public class MovingObject
    {
        public string id;
        public string type;                // pedestrian | bicycle | ...
        public float[] position = new float[3]; // required vec3 — never null on the wire
        public float[] velocity = new float[3];
        public float radius = 0.4f;
    }

    [Serializable]
    public class WorldEvent
    {
        public string type;
        public float[] position;
        public string lane_id;
    }

    [Serializable]
    public class CommandMessage
    {
        public float time;
        public int tick;
        public List<VehicleCommand> commands = new();
    }

    [Serializable]
    public class VehicleCommand
    {
        public string vehicle_id;
        public float target_speed;         // m/s, 0 = stop
        public string target_lane;
        public string behavior;
        public List<float[]> path = new();
        public bool lka_enabled = true;
    }
}
