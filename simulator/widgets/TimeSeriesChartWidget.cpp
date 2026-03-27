#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include <QPainter>
#include <QPen>
#include <algorithm>
#include <cmath>
#include <limits>

namespace simulator::widgets
{
    TimeSeriesChartWidget::TimeSeriesChartWidget(QWidget* parent)
        : QWidget(parent)
    {
        setMinimumSize(400, 300);
        setBackgroundRole(QPalette::Base);
        setAutoFillBackground(true);
        setMouseTracking(true);
    }

    void TimeSeriesChartWidget::SetTimeAxis(std::span<const float> time)
    {
        timeData.assign(time.begin(), time.end());
        maxTime = timeData.empty() ? 0.0f : timeData.back();
        interaction.SetDataRange(0.0f, maxTime);
        update();
    }

    void TimeSeriesChartWidget::SetTimeAxis(std::vector<float> time)
    {
        timeData = std::move(time);
        maxTime = timeData.empty() ? 0.0f : timeData.back();
        interaction.SetDataRange(0.0f, maxTime);
        update();
    }

    void TimeSeriesChartWidget::SetPanels(std::vector<ChartPanel> panels)
    {
        chartPanels = std::move(panels);
        update();
    }

    void TimeSeriesChartWidget::Clear()
    {
        timeData.clear();
        chartPanels.clear();
        maxTime = 0.0f;
        interaction.SetDataRange(0.0f, 0.0f);
        update();
    }

    float TimeSeriesChartWidget::TimeToX(float t, int plotLeft, int plotWidth) const
    {
        auto span = interaction.viewMax - interaction.viewMin;
        if (span <= 0.0f)
            return static_cast<float>(plotLeft);

        auto ratio = (t - interaction.viewMin) / span;
        return static_cast<float>(plotLeft) + ratio * plotWidth;
    }

    std::vector<TimeSeriesChartWidget::PanelLayout> TimeSeriesChartWidget::ComputeLayouts() const
    {
        std::vector<PanelLayout> layouts;

        auto plotWidth = width() - leftMargin - rightMargin;
        if (plotWidth <= 0 || chartPanels.empty())
            return layouts;

        auto totalWeight = 0;
        for (const auto& panel : chartPanels)
            totalWeight += std::max(panel.heightWeight, 1);

        auto availableHeight = height() - topMargin - bottomMargin - static_cast<int>(chartPanels.size() - 1) * panelSpacing;
        if (availableHeight <= 0)
            return layouts;

        auto currentY = topMargin;

        for (std::size_t i = 0; i < chartPanels.size(); ++i)
        {
            auto panelHeight = availableHeight * std::max(chartPanels[i].heightWeight, 1) / totalWeight;
            QRect plotArea(leftMargin, currentY, plotWidth, panelHeight);
            auto bounds = ComputeBounds(chartPanels[i]);
            layouts.push_back({ plotArea, bounds, i });
            currentY += panelHeight + panelSpacing;
        }

        return layouts;
    }

    void TimeSeriesChartWidget::paintEvent(QPaintEvent* event)
    {
        Q_UNUSED(event);

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        if (chartPanels.empty())
            return;

        cachedLayouts = ComputeLayouts();
        if (cachedLayouts.empty())
            return;

        auto plotWidth = width() - leftMargin - rightMargin;
        auto currentY = topMargin;

        for (const auto& layout : cachedLayouts)
        {
            DrawPanel(painter, layout.plotArea, chartPanels[layout.panelIndex], layout.bounds);
            currentY = layout.plotArea.bottom() + panelSpacing;
        }

        QFont labelFont = painter.font();
        labelFont.setPointSize(9);
        painter.setFont(labelFont);
        painter.setPen(Qt::black);

        auto lastPanelBottom = cachedLayouts.back().plotArea.bottom();
        painter.drawText(leftMargin + plotWidth / 2 - 20, lastPanelBottom + 25, "Time (s)");

        labelFont.setPointSize(8);
        painter.setFont(labelFont);

        for (auto i = 0; i <= gridLines; ++i)
        {
            auto x = leftMargin + (i * plotWidth / gridLines);
            auto t = (interaction.viewMax > interaction.viewMin)
                         ? interaction.viewMin + (static_cast<float>(i) / gridLines) * (interaction.viewMax - interaction.viewMin)
                         : 0.0f;
            QRect labelRect(x - 25, lastPanelBottom + 2, 50, 14);
            painter.drawText(labelRect, Qt::AlignCenter, QString::number(static_cast<double>(t), 'f', 2));
        }

        DrawCrosshair(painter);
    }

    void TimeSeriesChartWidget::DrawPanel(QPainter& painter, const QRect& plotArea, const ChartPanel& panel, const PanelBounds& bounds)
    {
        DrawAxes(painter, plotArea);
        DrawGridLines(painter, plotArea);

        QFont titleFont = painter.font();
        titleFont.setPointSize(10);
        titleFont.setBold(true);
        painter.setFont(titleFont);
        painter.setPen(Qt::black);
        painter.drawText(plotArea.left(), plotArea.top() - 3, panel.title);

        DrawYLabels(painter, plotArea, bounds);

        if (!timeData.empty())
        {
            for (const auto& series : panel.series)
                DrawSeries(painter, plotArea, series, bounds);
        }

        DrawLegend(painter, plotArea, panel);
    }

    void TimeSeriesChartWidget::DrawSeries(QPainter& painter, const QRect& plotArea, const Series& series, const PanelBounds& bounds)
    {
        auto viewSpan = interaction.viewMax - interaction.viewMin;
        if (viewSpan <= 0.0f || series.data.empty())
            return;

        auto range = bounds.maxY - bounds.minY;
        if (range <= 0.0f)
            range = 1.0f;

        QPen pen(series.color, 2);
        painter.setPen(pen);
        painter.setClipRect(plotArea);

        auto count = std::min(timeData.size(), series.data.size());
        auto stride = std::max<std::size_t>(1, count / static_cast<std::size_t>(plotArea.width() * 2));

        QPointF prev;
        auto hasPrev = false;

        for (std::size_t i = 0; i < count; i += stride)
        {
            auto x = TimeToX(timeData[i], plotArea.left(), plotArea.width());
            auto yRatio = (series.data[i] - bounds.minY) / range;
            auto y = static_cast<float>(plotArea.bottom()) - yRatio * plotArea.height();

            QPointF curr(x, y);
            if (hasPrev)
                painter.drawLine(prev, curr);
            prev = curr;
            hasPrev = true;
        }

        auto lastIdx = count - 1;
        if (hasPrev && lastIdx % stride != 0)
        {
            auto x = TimeToX(timeData[lastIdx], plotArea.left(), plotArea.width());
            auto yRatio = (series.data[lastIdx] - bounds.minY) / range;
            auto y = static_cast<float>(plotArea.bottom()) - yRatio * plotArea.height();
            painter.drawLine(prev, QPointF(x, y));
        }

        painter.setClipping(false);
    }

    void TimeSeriesChartWidget::DrawAxes(QPainter& painter, const QRect& plotArea)
    {
        QPen axisPen(Qt::darkGray, 1);
        painter.setPen(axisPen);
        painter.drawLine(plotArea.bottomLeft(), plotArea.bottomRight());
        painter.drawLine(plotArea.bottomLeft(), plotArea.topLeft());
    }

    void TimeSeriesChartWidget::DrawGridLines(QPainter& painter, const QRect& plotArea)
    {
        QPen gridPen(QColor(220, 220, 220), 1, Qt::DashLine);
        painter.setPen(gridPen);

        for (auto i = 1; i <= gridLines; ++i)
        {
            auto y = plotArea.bottom() - (i * plotArea.height() / gridLines);
            painter.drawLine(plotArea.left(), y, plotArea.right(), y);
        }

        for (auto i = 1; i <= gridLines; ++i)
        {
            auto x = plotArea.left() + (i * plotArea.width() / gridLines);
            painter.drawLine(x, plotArea.top(), x, plotArea.bottom());
        }
    }

    void TimeSeriesChartWidget::DrawYLabels(QPainter& painter, const QRect& plotArea, const PanelBounds& bounds)
    {
        QFont labelFont = painter.font();
        labelFont.setPointSize(8);
        labelFont.setBold(false);
        painter.setFont(labelFont);
        painter.setPen(Qt::black);

        for (auto i = 0; i <= gridLines; ++i)
        {
            auto y = plotArea.bottom() - (i * plotArea.height() / gridLines);
            auto val = bounds.minY + (static_cast<float>(i) / gridLines) * (bounds.maxY - bounds.minY);
            painter.drawText(plotArea.left() - 60, y + 4, QString::number(static_cast<double>(val), 'f', 2));
        }
    }

    void TimeSeriesChartWidget::DrawLegend(QPainter& painter, const QRect& plotArea, const ChartPanel& panel)
    {
        if (panel.series.size() <= 1)
            return;

        QFont legendFont = painter.font();
        legendFont.setPointSize(8);
        painter.setFont(legendFont);

        auto legendX = plotArea.right() - 160;
        auto legendY = plotArea.top() + 15;

        for (std::size_t i = 0; i < panel.series.size(); ++i)
        {
            auto y = legendY + static_cast<int>(i) * 16;
            painter.setPen(QPen(panel.series[i].color, 2));
            painter.drawLine(legendX, y, legendX + 20, y);
            painter.setPen(Qt::black);
            painter.drawText(legendX + 25, y + 4, panel.series[i].name);
        }
    }

    void TimeSeriesChartWidget::DrawCrosshair(QPainter& painter)
    {
        if (!interaction.showCrosshair || cachedLayouts.empty() || timeData.empty())
            return;

        auto cursorX = static_cast<int>(interaction.cursorPos.x());
        auto cursorY = static_cast<int>(interaction.cursorPos.y());

        const PanelLayout* hoveredLayout = nullptr;
        for (const auto& layout : cachedLayouts)
        {
            if (layout.plotArea.contains(cursorX, cursorY))
            {
                hoveredLayout = &layout;
                break;
            }
        }

        if (!hoveredLayout)
            return;

        auto plotArea = hoveredLayout->plotArea;
        auto bounds = hoveredLayout->bounds;
        const auto& panel = chartPanels[hoveredLayout->panelIndex];

        QPen crosshairPen(QColor(100, 100, 100, 150), 1, Qt::DashLine);
        painter.setPen(crosshairPen);
        painter.drawLine(cursorX, plotArea.top(), cursorX, plotArea.bottom());
        painter.drawLine(plotArea.left(), cursorY, plotArea.right(), cursorY);

        auto viewSpan = interaction.viewMax - interaction.viewMin;
        if (viewSpan <= 0.0f)
            return;

        auto xRatio = static_cast<float>(cursorX - plotArea.left()) / plotArea.width();
        auto timeAtCursor = interaction.viewMin + xRatio * viewSpan;

        auto yRange = bounds.maxY - bounds.minY;
        if (yRange <= 0.0f)
            yRange = 1.0f;
        auto yRatio = static_cast<float>(plotArea.bottom() - cursorY) / plotArea.height();
        auto yAtCursor = bounds.minY + yRatio * yRange;

        QStringList lines;
        lines << QString("t = %1 s").arg(static_cast<double>(timeAtCursor), 0, 'f', 3);

        for (const auto& series : panel.series)
        {
            auto count = std::min(timeData.size(), series.data.size());
            if (count == 0)
                continue;

            auto it = std::lower_bound(timeData.begin(), timeData.begin() + static_cast<std::ptrdiff_t>(count), timeAtCursor);
            std::size_t idx = 0;

            if (it == timeData.begin() + static_cast<std::ptrdiff_t>(count))
                idx = count - 1;
            else if (it == timeData.begin())
                idx = 0;
            else
            {
                auto pos = static_cast<std::size_t>(std::distance(timeData.begin(), it));
                auto prev = pos - 1;
                idx = (timeAtCursor - timeData[prev] < timeData[pos] - timeAtCursor) ? prev : pos;
            }

            lines << QString("%1 = %2").arg(series.name).arg(static_cast<double>(series.data[idx]), 0, 'f', 3);
        }

        auto text = lines.join("\n");

        QFont tooltipFont = painter.font();
        tooltipFont.setPointSize(8);
        painter.setFont(tooltipFont);

        QFontMetrics fm(tooltipFont);
        auto textRect = fm.boundingRect(QRect(0, 0, 300, 200), Qt::AlignLeft | Qt::TextWordWrap, text);

        auto tooltipX = cursorX + 12;
        auto tooltipY = cursorY - textRect.height() - 8;
        if (tooltipX + textRect.width() + 12 > plotArea.right())
            tooltipX = cursorX - textRect.width() - 20;
        if (tooltipY < plotArea.top())
            tooltipY = cursorY + 12;

        QRect bgRect(tooltipX - 4, tooltipY - 2, textRect.width() + 12, textRect.height() + 8);
        painter.setPen(QPen(QColor(180, 180, 180), 1));
        painter.setBrush(QColor(255, 255, 255, 230));
        painter.drawRoundedRect(bgRect, 3, 3);

        painter.setPen(Qt::black);
        painter.drawText(bgRect.adjusted(4, 2, -4, -2), Qt::AlignLeft, text);
        painter.setBrush(Qt::NoBrush);
    }

    TimeSeriesChartWidget::PanelBounds TimeSeriesChartWidget::ComputeBounds(const ChartPanel& panel) const
    {
        PanelBounds bounds{ std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest() };

        bool hasData = false;
        for (const auto& series : panel.series)
        {
            auto count = std::min(timeData.size(), series.data.size());
            for (std::size_t i = 0; i < count; ++i)
            {
                if (timeData[i] >= interaction.viewMin && timeData[i] <= interaction.viewMax)
                {
                    bounds.maxY = std::max(bounds.maxY, series.data[i]);
                    bounds.minY = std::min(bounds.minY, series.data[i]);
                    hasData = true;
                }
            }
        }

        if (!hasData)
        {
            bounds.minY = -1.0f;
            bounds.maxY = 1.0f;
            return bounds;
        }

        auto pad = (bounds.maxY - bounds.minY) * 0.1f;
        if (pad < 0.1f)
            pad = 0.1f;
        bounds.maxY += pad;
        bounds.minY -= pad;

        return bounds;
    }

    void TimeSeriesChartWidget::wheelEvent(QWheelEvent* event)
    {
        auto plotWidth = width() - leftMargin - rightMargin;
        if (plotWidth <= 0)
            return;

        auto cursorXRatio = static_cast<float>(event->position().x() - leftMargin) / plotWidth;
        cursorXRatio = std::clamp(cursorXRatio, 0.0f, 1.0f);

        interaction.Zoom(static_cast<float>(event->angleDelta().y()), cursorXRatio);
        update();
    }

    void TimeSeriesChartWidget::mousePressEvent(QMouseEvent* event)
    {
        if (event->button() == Qt::LeftButton)
        {
            auto plotWidth = width() - leftMargin - rightMargin;
            interaction.StartPan(event->pos(), leftMargin, plotWidth);
            setCursor(Qt::ClosedHandCursor);
        }
    }

    void TimeSeriesChartWidget::mouseMoveEvent(QMouseEvent* event)
    {
        interaction.cursorPos = event->pos();
        interaction.showCrosshair = true;

        if (interaction.IsPanning())
        {
            auto plotWidth = width() - leftMargin - rightMargin;
            interaction.UpdatePan(event->pos(), leftMargin, plotWidth);
        }

        update();
    }

    void TimeSeriesChartWidget::mouseReleaseEvent(QMouseEvent* event)
    {
        if (event->button() == Qt::LeftButton)
        {
            interaction.EndPan();
            setCursor(Qt::ArrowCursor);
        }
    }

    void TimeSeriesChartWidget::mouseDoubleClickEvent(QMouseEvent* event)
    {
        Q_UNUSED(event);
        interaction.ResetView();
        update();
    }

    void TimeSeriesChartWidget::leaveEvent(QEvent* event)
    {
        Q_UNUSED(event);
        interaction.showCrosshair = false;
        update();
    }
}
