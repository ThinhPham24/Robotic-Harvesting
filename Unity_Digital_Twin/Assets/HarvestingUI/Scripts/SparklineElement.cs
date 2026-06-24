using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

namespace Harvesting.DigitalTwin.UI
{
    public sealed class SparklineElement : VisualElement
    {
        private IReadOnlyList<float> values;
        private Color lineColor = new(0.22f, 0.74f, 0.97f);
        private float configuredMin;
        private float configuredMax = 1f;

        public SparklineElement()
        {
            generateVisualContent += Draw;
        }

        public void SetData(IReadOnlyList<float> source, float min, float max, Color color)
        {
            values = source;
            configuredMin = min;
            configuredMax = Mathf.Max(min + 0.0001f, max);
            lineColor = color;
            MarkDirtyRepaint();
        }

        private void Draw(MeshGenerationContext context)
        {
            if (values == null || values.Count < 2)
                return;

            var painter = context.painter2D;
            painter.lineWidth = 2f;
            painter.strokeColor = lineColor;
            var width = contentRect.width;
            var height = contentRect.height;

            for (var index = 0; index < values.Count; index++)
            {
                var x = width * index / (values.Count - 1f);
                var normalized = Mathf.InverseLerp(configuredMin, configuredMax, values[index]);
                var y = height - normalized * height;
                if (index == 0)
                    painter.BeginPath();
                if (index == 0)
                    painter.MoveTo(new Vector2(x, y));
                else
                    painter.LineTo(new Vector2(x, y));
            }

            painter.Stroke();
        }
    }
}
