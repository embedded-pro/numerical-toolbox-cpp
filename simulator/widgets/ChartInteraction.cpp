#include "simulator/widgets/ChartInteraction.hpp"
#include <algorithm>
#include <cmath>

namespace simulator::widgets
{
    void ChartInteraction::SetDataRange(float min, float max)
    {
        dataMin = min;
        dataMax = max;
        viewMin = min;
        viewMax = max;
    }

    void ChartInteraction::ResetView()
    {
        viewMin = dataMin;
        viewMax = dataMax;
    }

    void ChartInteraction::Zoom(float wheelDelta, float cursorXRatio)
    {
        auto span = viewMax - viewMin;
        if (span <= 0.0f)
            return;

        auto factor = (wheelDelta > 0.0f) ? (1.0f / zoomFactor) : zoomFactor;
        auto newSpan = span * factor;

        auto maxSpan = dataMax - dataMin;
        newSpan = std::clamp(newSpan, minViewSpan, maxSpan);

        auto anchor = viewMin + cursorXRatio * span;
        viewMin = anchor - cursorXRatio * newSpan;
        viewMax = anchor + (1.0f - cursorXRatio) * newSpan;

        ClampView();
    }

    void ChartInteraction::StartPan(QPointF pos, int plotLeft, int plotWidth)
    {
        panning = true;
        panStartPos = pos;
        panStartViewMin = viewMin;
        panStartViewMax = viewMax;
    }

    void ChartInteraction::UpdatePan(QPointF pos, int plotLeft, int plotWidth)
    {
        if (!panning || plotWidth <= 0)
            return;

        auto span = panStartViewMax - panStartViewMin;
        auto pixelDelta = pos.x() - panStartPos.x();
        auto viewDelta = -pixelDelta / plotWidth * span;

        viewMin = panStartViewMin + static_cast<float>(viewDelta);
        viewMax = panStartViewMax + static_cast<float>(viewDelta);

        ClampView();
    }

    void ChartInteraction::EndPan()
    {
        panning = false;
    }

    bool ChartInteraction::IsZoomed() const
    {
        auto tolerance = (dataMax - dataMin) * 0.001f;
        return (viewMin - dataMin) > tolerance || (dataMax - viewMax) > tolerance;
    }

    bool ChartInteraction::IsPanning() const
    {
        return panning;
    }

    void ChartInteraction::ClampView()
    {
        auto span = viewMax - viewMin;
        auto maxSpan = dataMax - dataMin;

        if (span > maxSpan)
        {
            viewMin = dataMin;
            viewMax = dataMax;
            return;
        }

        if (viewMin < dataMin)
        {
            viewMin = dataMin;
            viewMax = viewMin + span;
        }

        if (viewMax > dataMax)
        {
            viewMax = dataMax;
            viewMin = viewMax - span;
        }
    }
}
