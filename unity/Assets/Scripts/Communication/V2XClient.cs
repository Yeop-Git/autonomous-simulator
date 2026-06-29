using System;
using System.Collections.Concurrent;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using V2X.Protocol;

// Minimal V2X client for the Phase 2 vertical slice.
//
// Responsibilities:
//   - connect to the Python central server over WebSocket,
//   - each tick, gather world state and send a StateMessage,
//   - receive CommandMessage and hand commands to vehicles.
//
// NOTE: this uses System.Net.WebSockets (ClientWebSocket), which works in
// the Editor and standalone builds. WebGL builds need a different transport.
//
// Time-sync is the #1 risk: we tag every StateMessage with an incrementing
// tick and FixedUpdate time, and check that returning commands aren't stale.

namespace V2X.Communication
{
    public class V2XClient : MonoBehaviour
    {
        [Header("Connection")]
        public string serverUrl = "ws://localhost:8765";
        [Tooltip("How many ticks a command may lag before we warn.")]
        public int maxCommandLagTicks = 3;

        private ClientWebSocket _socket;
        private CancellationTokenSource _cts;
        private readonly ConcurrentQueue<CommandMessage> _inbox = new();
        private int _tick;
        private bool _connected;

        // Hook these up in the inspector / a manager. Stubs for now.
        // public WorldStateProvider stateProvider;
        // public CommandDispatcher dispatcher;

        public bool IsConnected => _connected;

        private async void Start()
        {
            await Connect();
        }

        private async Task Connect()
        {
            _cts = new CancellationTokenSource();
            _socket = new ClientWebSocket();
            try
            {
                await _socket.ConnectAsync(new Uri(serverUrl), _cts.Token);
                _connected = true;
                Debug.Log($"[V2XClient] connected to {serverUrl}");
                _ = ReceiveLoop();
            }
            catch (Exception e)
            {
                Debug.LogError($"[V2XClient] connect failed: {e.Message}");
            }
        }

        // Drive the send cadence from physics so it matches vehicle motion.
        private async void FixedUpdate()
        {
            if (!_connected) return;

            // TODO(Phase 1/2): build a real StateMessage from the scene.
            var state = new StateMessage
            {
                time = Time.fixedTime,
                tick = _tick++,
                scenario = "lka_test",
                // vehicles = stateProvider.CollectVehicles(),
            };

            await SendState(state);
            DrainCommands();
        }

        private async Task SendState(StateMessage state)
        {
            try
            {
                string json = JsonUtility.ToJson(state);
                var bytes = Encoding.UTF8.GetBytes(json);
                await _socket.SendAsync(
                    new ArraySegment<byte>(bytes),
                    WebSocketMessageType.Text, true, _cts.Token);
            }
            catch (Exception e)
            {
                Debug.LogError($"[V2XClient] send failed: {e.Message}");
                _connected = false;
            }
        }

        private async Task ReceiveLoop()
        {
            var buffer = new byte[1 << 20];
            while (_connected && !_cts.IsCancellationRequested)
            {
                try
                {
                    var result = await _socket.ReceiveAsync(
                        new ArraySegment<byte>(buffer), _cts.Token);
                    if (result.MessageType == WebSocketMessageType.Close)
                    {
                        _connected = false;
                        break;
                    }
                    string json = Encoding.UTF8.GetString(buffer, 0, result.Count);
                    var cmd = JsonUtility.FromJson<CommandMessage>(json);
                    _inbox.Enqueue(cmd);
                }
                catch (Exception e)
                {
                    Debug.LogError($"[V2XClient] receive failed: {e.Message}");
                    _connected = false;
                }
            }
        }

        // Apply commands on the main thread (Unity API is not thread-safe).
        private void DrainCommands()
        {
            while (_inbox.TryDequeue(out var cmd))
            {
                int lag = _tick - cmd.tick;
                if (lag > maxCommandLagTicks)
                    Debug.LogWarning($"[V2XClient] stale command, lag={lag} ticks");

                // TODO(Phase 2): dispatcher.Apply(cmd);
            }
        }

        private async void OnDestroy()
        {
            _cts?.Cancel();
            if (_socket is { State: WebSocketState.Open })
            {
                await _socket.CloseAsync(
                    WebSocketCloseStatus.NormalClosure, "bye", CancellationToken.None);
            }
            _socket?.Dispose();
        }
    }
}
