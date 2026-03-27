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
    }

    void TimeSeriesChartWidget::SetTimeAxis(std::span<const float> time)
    {
        timeData = time;
        maxTime = timeData.empty() ? 0.0f : timeData.back();
        update();
    }

    void TimeSeriesChartWidget::SetPanels(std::vector<ChartPanel> panels)
    {
        chartPanels = std::move(panels);
        update();
    }

    void TimeSeriesChartWidget::Clear()
    {
        timeData = {};
        chartPanels.clear();
        maxTime = 0.0f;
        update();
    }

    void TimeSeriesChartWidget::paintEvent(QPaintEvent* event)
    {
        Q_UNUSED(event);

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        if (chartPanels.empty())
            return;

        auto plotWidth = width() - leftMargin - rightMargin;
        if (plotWidth <= 0)
            return;

        auto totalWeight = 0;
        for (const auto& panel : chartPanels)
            totalWeight += std::max(panel.heightWeight, 1);

        auto availableHeight = height() - topMargin - bottomMargin - static_cast<int>(chartPanels.size() - 1) * panelSpacing;
        if (availableHeight <= 0)
            return;

        auto currentY = topMargin;

        for (const auto& panel : chartPanels)
        {
            auto panelHeight = availableHeight * std::max(panel.heightWeight, 1) / totalWeight;
            QRect plotArea(leftMargin, currentY, plotWidth, panelHeight);

            auto bounds = ComputeBounds(panel);
            DrawPanel(painter, plotArea, panel, bounds);

            currentY += panelHeight + panelSpacing;
        }

        QFont labelFont = painter.font();
        labelFont.setPointSize(9);
        painter.setFont(labelFont);
        painter.setPen(Qt::black);

        auto lastPanelBottom = currentY - panelSpacing;
        painter.drawText(leftMargin + plotWidth / 2 - 20, lastPanelBottom + 25, "Time (s)");

        labelFont.setPointSize(8);
        painter.setFont(labelFont);

        auto lastPanelY = currentY - panelSpacing;
        for (auto i = 0; i <= gridLines; ++i)
        {
            auto x = leftMargin + (i * plotWidth / gridLines);
            auto t = maxTime > 0.0f ? (static_cast<float>(i) / gridLines) * maxTime : 0.0f;
            QRect labelRect(x - 25, lastPanelY + 2, 50, 14);
            painter.drawText(labelRect, Qt::AlignCenter, QString::number(static_cast<double>(t), 'f', 1));
        }
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
        if (maxTime <= 0.0f || series.data.empty())
            return;

        auto range = bounds.maxY - bounds.minY;
        if (range <= 0.0f)
            range = 1.0f;

        QPen pen(series.color, 2);
        painter.setPen(pen);

        QPointF prev;
        auto hasPrev = false;
        auto count = std::min(timeData.size(), series.data.size());

        for (std::size_t i = 0; i < count; ++i)
        {
            auto xRatio = timeData[i] / maxTime;
            auto yRatio = (series.data[i] - bounds.minY) / range;

            auto x = plotArea.left() + static_cast<int>(xRatio * plotArea.width());
            auto y = plotArea.bottom() - static_cast<int>(yRatio * plotArea.height());

            QPointF curr(x, y);
            if (hasPrev)
                painter.drawLine(prev, curr);
            prev = curr;
            hasPrev = true;
        }
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

    TimeSeriesChartWidget::PanelBounds TimeSeriesChartWidget::ComputeBounds(const ChartPanel& panel) const
    {
        PanelBounds bounds{ std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest() };

        bool hasData = false;
        for (const auto& series : panel.series)
        {
            for (auto v : series.data)
            {
                bounds.maxY = std::max(bounds.maxY, v);
                bounds.minY = std::min(bounds.minY, v);
                hasData = true;
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
}
