using System;
using System.Collections.Concurrent;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json;
using UnityEngine;
using V2X.Protocol;

// V2X client for the Unity <-> Python central server loop.
//
//   - connects to the Python server over WebSocket (ClientWebSocket; works in
//     Editor + standalone, NOT WebGL),
//   - every FixedUpdate, asks an IWorldStateProvider for the current world and
//     sends it as a StateMessage,
//   - receives CommandMessages on a background task, queues them, and applies
//     them on the main thread via an ICommandSink.
//
// Serialization uses Newtonsoft.Json (NOT JsonUtility): the wire format has
// jagged arrays (path = list of [x,y,z]) that JsonUtility cannot handle.
//
// Time-sync is the project's #1 risk: every StateMessage carries an
// incrementing tick + FixedUpdate time, and we warn when a returned command
// lags more than maxCommandLagTicks behind the tick we are now on.

namespace V2X.Communication
{
    public interface IWorldStateProvider
    {
        // Fill a StateMessage from the live scene for this tick.
        StateMessage CollectState(int tick, float time);
    }

    public interface ICommandSink
    {
        // Apply one tick's worth of commands to the vehicles.
        void Apply(CommandMessage command);
    }

    public class V2XClient : MonoBehaviour
    {
        [Header("Connection")]
        public string serverUrl = "ws://localhost:8765";
        [Tooltip("How many ticks a command may lag before we warn.")]
        public int maxCommandLagTicks = 3;
        [Tooltip("Send a state message every N FixedUpdates (1 = every tick).")]
        public int sendEveryNTicks = 1;

        [Header("Wiring (assign in inspector)")]
        public MonoBehaviour stateProviderSource; // must implement IWorldStateProvider
        public MonoBehaviour commandSinkSource;    // must implement ICommandSink

        private IWorldStateProvider _provider;
        private ICommandSink _sink;

        private ClientWebSocket _socket;
        private CancellationTokenSource _cts;
        private readonly ConcurrentQueue<CommandMessage> _inbox = new();
        private int _tick;
        private bool _connected;
        private bool _isSending; // guards against overlapping SendAsync calls

        public bool IsConnected => _connected;
        public int CurrentTick => _tick;
        public int LastAppliedTick { get; private set; } = -1;
        public int LastLagTicks { get; private set; }

        private static readonly JsonSerializerSettings JsonSettings = new()
        {
            NullValueHandling = NullValueHandling.Include,
            Formatting = Formatting.None,
        };

        private async void Start()
        {
            _provider = stateProviderSource as IWorldStateProvider;
            _sink = commandSinkSource as ICommandSink;
            if (_provider == null)
                Debug.LogWarning("[V2XClient] no IWorldStateProvider assigned; sending empty state.");
            if (_sink == null)
                Debug.LogWarning("[V2XClient] no ICommandSink assigned; commands will be dropped.");
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

            DrainCommands(); // apply anything received since last tick first

            // Skip this tick's send if the previous one is still in flight:
            // ClientWebSocket forbids concurrent SendAsync, and a second send
            // would throw and silently drop the connection (the #1 sync risk).
            if (_tick % Mathf.Max(1, sendEveryNTicks) == 0 && !_isSending)
            {
                StateMessage state = _provider != null
                    ? _provider.CollectState(_tick, Time.fixedTime)
                    : new StateMessage { time = Time.fixedTime, tick = _tick };
                state.tick = _tick;
                state.time = Time.fixedTime;
                await SendState(state);
            }
            else if (_isSending)
            {
                Debug.LogWarning($"[V2XClient] send still in flight at tick {_tick}; " +
                                 "skipping (server slower than physics tick).");
            }
            _tick++;
        }

        private async Task SendState(StateMessage state)
        {
            _isSending = true;
            try
            {
                string json = JsonConvert.SerializeObject(state, JsonSettings);
                var bytes = Encoding.UTF8.GetBytes(json);
                await _socket.SendAsync(new ArraySegment<byte>(bytes),
                    WebSocketMessageType.Text, true, _cts.Token);
            }
            catch (Exception e)
            {
                Debug.LogError($"[V2XClient] send failed: {e.Message}");
                _connected = false;
            }
            finally
            {
                _isSending = false;
            }
        }

        private async Task ReceiveLoop()
        {
            var buffer = new byte[1 << 20];
            var sb = new StringBuilder();
            while (_connected && !_cts.IsCancellationRequested)
            {
                try
                {
                    sb.Clear();
                    WebSocketReceiveResult result;
                    do
                    {
                        result = await _socket.ReceiveAsync(
                            new ArraySegment<byte>(buffer), _cts.Token);
                        if (result.MessageType == WebSocketMessageType.Close)
                        {
                            _connected = false;
                            return;
                        }
                        sb.Append(Encoding.UTF8.GetString(buffer, 0, result.Count));
                    } while (!result.EndOfMessage);

                    var cmd = JsonConvert.DeserializeObject<CommandMessage>(sb.ToString());
                    if (cmd != null) _inbox.Enqueue(cmd);
                }
                catch (OperationCanceledException) { break; }
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
                LastLagTicks = lag;
                if (lag > maxCommandLagTicks)
                    Debug.LogWarning($"[V2XClient] stale command, lag={lag} ticks " +
                                     $"(cmd tick {cmd.tick}, now {_tick})");
                _sink?.Apply(cmd);
                LastAppliedTick = cmd.tick;
            }
        }

        private async void OnDestroy()
        {
            _cts?.Cancel();
            try
            {
                if (_socket is { State: WebSocketState.Open })
                    await _socket.CloseAsync(WebSocketCloseStatus.NormalClosure, "bye",
                        CancellationToken.None);
            }
            catch { /* shutting down */ }
            _socket?.Dispose();
        }
    }
}
