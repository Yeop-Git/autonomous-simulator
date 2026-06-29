using System;
using System.Collections.Generic;

// Wire-format mirrors shared/protocol/*.schema.json.
// If you change a schema, change this file too (and the Python side).
// JsonUtility is used for (de)serialization, so fields must be plain and
// match the JSON keys exactly. Vec3 is sent as float[3] = [x, y, z].

namespace V2X.Protocol
{
    [Serializable]
    public class StateMessage
    {
        public float time;
        public int tick;
        public string scenario;            // "highway" | "urban" | "lka_test"
        public List<VehicleState> vehicles = new();
        public List<MovingObject> objects = new();
        public List<WorldEvent> events = new();
    }

    [Serializable]
    public class VehicleState
    {
        public string id;
        public string type = "car";
        public float[] position;           // [x, y, z]
        public float[] velocity;           // [x, y, z]
        public float[] acceleration;       // [x, y, z]
        public float heading;              // yaw degrees, 0=+Z, CW
        public string current_lane;
        public string target_lane;
        public string behavior_state;
    }

    [Serializable]
    public class MovingObject
    {
        public string id;
        public string type;                // pedestrian | bicycle | ...
        public float[] position;
        public float[] velocity;
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
