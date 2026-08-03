using Unity.AI.MCP.Editor;
using UnityEditor;

/// <summary>
/// Restarts Unity's MCP named-pipe listener after script domain reloads.
/// Unity AI Assistant 2.17 beta can leave the bridge marked as enabled while
/// its listener is no longer accepting connections.
/// </summary>
internal static class V2XMcpBridgeRecovery
{
    [InitializeOnLoadMethod]
    private static void RecoverAfterDomainReload()
    {
        if (!UnityMCPBridge.Enabled)
        {
            return;
        }

        UnityMCPBridge.Stop();
        EditorApplication.delayCall += Restart;
    }

    [MenuItem("V2X/Restart Unity MCP Bridge")]
    private static void Restart()
    {
        EditorApplication.delayCall -= Restart;
        UnityMCPBridge.Start();
    }
}
