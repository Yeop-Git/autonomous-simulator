using UnityEngine;

namespace V2X.UI
{
    /// <summary>
    /// Shared low-chrome IMGUI style for runtime telemetry overlays. The
    /// palette mirrors the uGUI controls authored by V2XSceneBuilder: one blue
    /// action color, parchment canvas, white surfaces, dark ink, no shadows.
    /// </summary>
    internal static class AppleGui
    {
        internal const string BlueHex = "#0066cc";
        internal const string BlueOnDarkHex = "#2997ff";
        internal const string GoodHex = "#248a3d";
        internal const string DangerHex = "#d70015";

        internal static readonly Color Ink = new(.114f, .114f, .122f, 1f);
        internal static readonly Color Muted = new(.478f, .478f, .478f, 1f);
        internal static readonly Color Blue = new(0f, .4f, .8f, 1f);
        internal static readonly Color White = Color.white;
        internal static readonly Color Parchment = new(.961f, .961f, .969f, .96f);

        private static GUIStyle _panel;
        private static GUIStyle _subtlePanel;
        private static GUIStyle _title;
        private static GUIStyle _body;
        private static GUIStyle _muted;
        private static GUIStyle _state;
        private static GUIStyle _statusBox;

        internal static GUIStyle Panel => _panel ??= MakePanel(White, 18, 24);
        internal static GUIStyle SubtlePanel => _subtlePanel ??= MakePanel(Parchment, 12, 16);
        internal static GUIStyle Title => _title ??= MakeLabel(24, true, Ink);
        internal static GUIStyle Body => _body ??= MakeLabel(16, false, Ink);
        internal static GUIStyle MutedBody => _muted ??= MakeLabel(14, false, Muted);
        internal static GUIStyle State => _state ??= MakeLabel(18, true, Ink);
        internal static GUIStyle StatusBox => _statusBox ??= MakeStatusBox();

        /// <summary>Keep IMGUI telemetry legible on high-DPI Game views while
        /// preserving the authored pixel size on smaller windows.</summary>
        internal static float BeginFrame()
        {
            float scale = Mathf.Max(1f,
                Mathf.Min(Screen.width / 1920f, Screen.height / 1080f));
            GUI.matrix = Matrix4x4.Scale(new Vector3(scale, scale, 1f));
            return Screen.width / scale;
        }

        private static GUIStyle MakePanel(Color color, int radius, int padding)
        {
            return new GUIStyle(GUI.skin.box)
            {
                normal = { background = RoundedTexture(color, radius), textColor = Ink },
                border = new RectOffset(radius, radius, radius, radius),
                padding = new RectOffset(padding, padding, padding, padding),
                margin = new RectOffset(0, 0, 0, 0),
            };
        }

        private static GUIStyle MakeLabel(int size, bool emphasized, Color color)
        {
            return new GUIStyle(GUI.skin.label)
            {
                font = Resources.Load<Font>(emphasized
                    ? "Fonts/Pretendard-SemiBold"
                    : "Fonts/Pretendard-Regular"),
                fontSize = size,
                fontStyle = FontStyle.Normal,
                richText = true,
                wordWrap = true,
                normal = { textColor = color },
            };
        }

        private static GUIStyle MakeStatusBox()
        {
            var style = MakePanel(White, 12, 12);
            style.font = Resources.Load<Font>("Fonts/Pretendard-SemiBold");
            style.fontSize = 16;
            style.fontStyle = FontStyle.Normal;
            style.alignment = TextAnchor.MiddleCenter;
            style.richText = true;
            style.normal.textColor = Ink;
            return style;
        }

        private static Texture2D RoundedTexture(Color color, int radius)
        {
            const int size = 64;
            var texture = new Texture2D(size, size, TextureFormat.RGBA32, false)
            {
                hideFlags = HideFlags.HideAndDontSave,
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
            };
            var pixels = new Color[size * size];
            for (int y = 0; y < size; y++)
            for (int x = 0; x < size; x++)
            {
                float dx = Mathf.Max(radius - x - .5f, 0f,
                    x + .5f - (size - radius));
                float dy = Mathf.Max(radius - y - .5f, 0f,
                    y + .5f - (size - radius));
                float alpha = Mathf.Clamp01(radius + .5f - Mathf.Sqrt(dx * dx + dy * dy));
                pixels[y * size + x] = new Color(color.r, color.g, color.b, color.a * alpha);
            }
            texture.SetPixels(pixels);
            texture.Apply();
            return texture;
        }
    }
}
