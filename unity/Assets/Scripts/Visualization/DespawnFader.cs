using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace V2X.Visualization
{
    /// <summary>Converts this instance's materials to transparent and fades it out.</summary>
    public sealed class DespawnFader : MonoBehaviour
    {
        private static readonly int BaseColor = Shader.PropertyToID("_BaseColor");
        private static readonly int Color = Shader.PropertyToID("_Color");
        private static readonly int Surface = Shader.PropertyToID("_Surface");
        private static readonly int Blend = Shader.PropertyToID("_Blend");
        private static readonly int SrcBlend = Shader.PropertyToID("_SrcBlend");
        private static readonly int DstBlend = Shader.PropertyToID("_DstBlend");
        private static readonly int ZWrite = Shader.PropertyToID("_ZWrite");

        private readonly List<MaterialSlot> _slots = new();
        private bool _started;

        private readonly struct MaterialSlot
        {
            public readonly Material Material;
            public readonly Color InitialColor;
            public readonly bool HasBaseColor;
            public readonly bool HasColor;

            public MaterialSlot(Material material, Color initialColor,
                bool hasBaseColor, bool hasColor)
            {
                Material = material;
                InitialColor = initialColor;
                HasBaseColor = hasBaseColor;
                HasColor = hasColor;
            }
        }

        public static void FadeAndDestroy(GameObject target, float duration)
        {
            if (target == null) return;
            var fader = target.GetComponent<DespawnFader>();
            if (fader == null) fader = target.AddComponent<DespawnFader>();
            fader.Begin(duration);
        }

        private void Begin(float duration)
        {
            if (_started) return;
            _started = true;
            PrepareTransparentMaterials();
            StartCoroutine(FadeRoutine(Mathf.Max(.05f, duration)));
        }

        private void PrepareTransparentMaterials()
        {
            foreach (var renderer in GetComponentsInChildren<Renderer>(true))
            {
                var materials = renderer.materials;
                for (int index = 0; index < materials.Length; index++)
                {
                    var material = materials[index];
                    if (material == null) continue;

                    bool hasBaseColor = material.HasProperty(BaseColor);
                    bool hasColor = material.HasProperty(Color);
                    Color tint = hasBaseColor ? material.GetColor(BaseColor) :
                        hasColor ? material.GetColor(Color) : UnityEngine.Color.white;

                    var properties = new MaterialPropertyBlock();
                    renderer.GetPropertyBlock(properties, index);
                    if (properties.HasColor(BaseColor))
                        tint = properties.GetColor(BaseColor);
                    else if (properties.HasColor(Color))
                        tint = properties.GetColor(Color);
                    renderer.SetPropertyBlock(null, index);

                    ConfigureTransparent(material);
                    SetColor(material, tint, hasBaseColor, hasColor);
                    _slots.Add(new MaterialSlot(
                        material, tint, hasBaseColor, hasColor));
                }
            }
        }

        private static void ConfigureTransparent(Material material)
        {
            material.SetOverrideTag("RenderType", "Transparent");
            if (material.HasProperty(Surface)) material.SetFloat(Surface, 1f);
            if (material.HasProperty(Blend)) material.SetFloat(Blend, 0f);
            if (material.HasProperty(SrcBlend))
                material.SetFloat(SrcBlend, (float)BlendMode.SrcAlpha);
            if (material.HasProperty(DstBlend))
                material.SetFloat(DstBlend, (float)BlendMode.OneMinusSrcAlpha);
            if (material.HasProperty(ZWrite)) material.SetFloat(ZWrite, 0f);
            material.DisableKeyword("_ALPHATEST_ON");
            material.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
            material.EnableKeyword("_ALPHABLEND_ON");
            material.renderQueue = (int)RenderQueue.Transparent;
            material.SetShaderPassEnabled("ShadowCaster", false);
        }

        private IEnumerator FadeRoutine(float duration)
        {
            float elapsed = 0f;
            while (elapsed < duration)
            {
                elapsed += Time.deltaTime;
                SetAlpha(1f - Mathf.Clamp01(elapsed / duration));
                yield return null;
            }
            Destroy(gameObject);
        }

        private void SetAlpha(float alpha)
        {
            foreach (var slot in _slots)
            {
                if (slot.Material == null) continue;
                Color tint = slot.InitialColor;
                tint.a *= alpha;
                SetColor(slot.Material, tint, slot.HasBaseColor, slot.HasColor);
            }
        }

        private static void SetColor(Material material, Color tint,
            bool hasBaseColor, bool hasColor)
        {
            if (hasBaseColor) material.SetColor(BaseColor, tint);
            if (hasColor) material.SetColor(Color, tint);
        }
    }
}
