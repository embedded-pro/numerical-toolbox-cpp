#pragma once

#include <QPointF>

namespace simulator::widgets
{
    class ChartInteraction
    {
    public:
        void SetDataRange(float min, float max);
        void ResetView();
        void Zoom(float wheelDelta, float cursorXRatio);
        void StartPan(QPointF pos, int plotLeft, int plotWidth);
        void UpdatePan(QPointF pos, int plotLeft, int plotWidth);
        void EndPan();
        [[nodiscard]] bool IsZoomed() const;
        [[nodiscard]] bool IsPanning() const;

        float viewMin = 0.0f;
        float viewMax = 1.0f;

        bool showCrosshair = false;
        QPointF cursorPos;

    private:
        void ClampView();

        float dataMin = 0.0f;
        float dataMax = 1.0f;
        bool panning = false;
        QPointF panStartPos;
        float panStartViewMin = 0.0f;
        float panStartViewMax = 0.0f;

        static constexpr float zoomFactor = 1.15f;
        static constexpr float minViewSpan = 1e-4f;
    };
}
