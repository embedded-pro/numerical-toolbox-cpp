#include "simulator/widgets/FrequencyChartWidget.hpp"
#include <QPainter>
#include <QPen>
#include <algorithm>
#include <cmath>
#include <limits>

namespace simulator::widgets
{
    FrequencyChartWidget::FrequencyChartWidget(QWidget* parent)
        : QWidget(parent)
    {
        setMinimumSize(400, 300);
        setBackgroundRole(QPalette::Base);
        setAutoFillBackground(true);
        setMouseTracking(true);
    }

    void FrequencyChartWidget::SetFrequencyAxis(std::span<const float> frequenciesHz)
    {
        freqData.assign(frequenciesHz.begin(), frequenciesHz.end());
        UpdateLogRange();
        update();
    }

    void FrequencyChartWidget::SetFrequencyAxis(std::vector<float> frequenciesHz)
    {
        freqData = std::move(frequenciesHz);
        UpdateLogRange();
        update();
    }

    void FrequencyChartWidget::UpdateLogRange()
    {
        if (!freqData.empty())
        {
            minFreq = *std::min_element(freqData.begin(), freqData.end());
            maxFreq = *std::max_element(freqData.begin(), freqData.end());

            if (minFreq <= 0.0f)
                minFreq = 1.0f;

            logMinFreq = std::floor(std::log10(minFreq));
            logMaxFreq = std::ceil(std::log10(maxFreq));

            interaction.SetDataRange(logMinFreq, logMaxFreq);
        }
    }

    void FrequencyChartWidget::SetPanels(std::vector<ChartPanel> panels)
    {
        chartPanels = std::move(panels);
        update();
    }

    void FrequencyChartWidget::Clear()
    {
        freqData.clear();
        chartPanels.clear();
        minFreq = 0.0f;
        maxFreq = 0.0f;
        logMinFreq = 0.0f;
        logMaxFreq = 0.0f;
        interaction.SetDataRange(0.0f, 0.0f);
        update();
    }

    float FrequencyChartWidget::FreqToX(float freq, int plotLeft, int plotWidth) const
    {
        auto viewSpan = interaction.viewMax - interaction.viewMin;
        if (viewSpan <= 0.0f)
            return static_cast<float>(plotLeft);

        auto logFreq = std::log10(std::max(freq, std::pow(10.0f, interaction.viewMin)));
        auto ratio = (logFreq - interaction.viewMin) / viewSpan;
        return static_cast<float>(plotLeft) + ratio * plotWidth;
    }

    std::vector<FrequencyChartWidget::PanelLayout> FrequencyChartWidget::ComputeLayouts() const
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

    void FrequencyChartWidget::paintEvent(QPaintEvent* event)
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

        for (const auto& layout : cachedLayouts)
            DrawPanel(painter, layout.plotArea, chartPanels[layout.panelIndex], layout.bounds);

        QFont labelFont = painter.font();
        labelFont.setPointSize(9);
        painter.setFont(labelFont);
        painter.setPen(Qt::black);

        auto lastPanelBottom = cachedLayouts.back().plotArea.bottom();
        painter.drawText(leftMargin + plotWidth / 2 - 30, lastPanelBottom + 25, "Frequency (Hz)");

        labelFont.setPointSize(8);
        painter.setFont(labelFont);

        auto viewSpan = interaction.viewMax - interaction.viewMin;
        if (viewSpan > 0.0f)
        {
            auto startDecade = static_cast<int>(std::floor(interaction.viewMin));
            auto endDecade = static_cast<int>(std::ceil(interaction.viewMax));

            for (int decade = startDecade; decade <= endDecade; ++decade)
            {
                auto freq = std::pow(10.0f, static_cast<float>(decade));
                auto x = static_cast<int>(FreqToX(freq, leftMargin, plotWidth));

                if (x >= leftMargin && x <= leftMargin + plotWidth)
                {
                    QString label;
                    if (freq >= 1000.0f)
                        label = QString::number(static_cast<double>(freq / 1000.0f), 'f', 0) + "k";
                    else
                        label = QString::number(static_cast<double>(freq), 'f', 0);

                    QRect labelRect(x - 25, lastPanelBottom + 2, 50, 14);
                    painter.drawText(labelRect, Qt::AlignCenter, label);
                }
            }
        }

        DrawCrosshair(painter);
    }

    void FrequencyChartWidget::DrawPanel(QPainter& painter, const QRect& plotArea, const ChartPanel& panel, const PanelBounds& bounds)
    {
        DrawAxes(painter, plotArea);
        DrawLogGridLines(painter, plotArea);

        QFont titleFont = painter.font();
        titleFont.setPointSize(10);
        titleFont.setBold(true);
        painter.setFont(titleFont);
        painter.setPen(Qt::black);
        painter.drawText(plotArea.left(), plotArea.top() - 3, panel.title);

        DrawYLabels(painter, plotArea, bounds);

        if (!freqData.empty())
        {
            for (const auto& series : panel.series)
                DrawSeries(painter, plotArea, series, bounds);
        }

        DrawLegend(painter, plotArea, panel);
    }

    void FrequencyChartWidget::DrawSeries(QPainter& painter, const QRect& plotArea, const Series& series, const PanelBounds& bounds)
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

        QPointF prev;
        auto hasPrev = false;
        auto count = std::min(freqData.size(), series.data.size());
        auto stride = std::max<std::size_t>(1, count / static_cast<std::size_t>(plotArea.width() * 2));

        for (std::size_t i = 0; i < count; i += stride)
        {
            if (freqData[i] <= 0.0f)
                continue;

            auto x = FreqToX(freqData[i], plotArea.left(), plotArea.width());
            auto yRatio = (series.data[i] - bounds.minY) / range;
            auto y = static_cast<float>(plotArea.bottom()) - yRatio * plotArea.height();

            QPointF curr(x, y);
            if (hasPrev)
                painter.drawLine(prev, curr);
            prev = curr;
            hasPrev = true;
        }

        auto lastIdx = count - 1;
        if (hasPrev && lastIdx % stride != 0 && freqData[lastIdx] > 0.0f)
        {
            auto x = FreqToX(freqData[lastIdx], plotArea.left(), plotArea.width());
            auto yRatio = (series.data[lastIdx] - bounds.minY) / range;
            auto y = static_cast<float>(plotArea.bottom()) - yRatio * plotArea.height();
            painter.drawLine(prev, QPointF(x, y));
        }

        painter.setClipping(false);
    }

    void FrequencyChartWidget::DrawAxes(QPainter& painter, const QRect& plotArea)
    {
        QPen axisPen(Qt::darkGray, 1);
        painter.setPen(axisPen);
        painter.drawLine(plotArea.bottomLeft(), plotArea.bottomRight());
        painter.drawLine(plotArea.bottomLeft(), plotArea.topLeft());
    }

    void FrequencyChartWidget::DrawLogGridLines(QPainter& painter, const QRect& plotArea)
    {
        QPen gridPen(QColor(220, 220, 220), 1, Qt::DashLine);
        painter.setPen(gridPen);

        for (auto i = 1; i <= yGridLines; ++i)
        {
            auto y = plotArea.bottom() - (i * plotArea.height() / yGridLines);
            painter.drawLine(plotArea.left(), y, plotArea.right(), y);
        }

        auto viewSpan = interaction.viewMax - interaction.viewMin;
        if (viewSpan <= 0.0f)
            return;

        auto startDecade = static_cast<int>(std::floor(interaction.viewMin));
        auto endDecade = static_cast<int>(std::ceil(interaction.viewMax));

        for (int decade = startDecade; decade <= endDecade; ++decade)
        {
            for (int sub = 1; sub <= 9; ++sub)
            {
                auto freq = static_cast<float>(sub) * std::pow(10.0f, static_cast<float>(decade));
                auto x = static_cast<int>(FreqToX(freq, plotArea.left(), plotArea.width()));

                if (x > plotArea.left() && x < plotArea.right())
                {
                    auto pen = (sub == 1) ? QPen(QColor(200, 200, 200), 1, Qt::DashLine)
                                          : QPen(QColor(235, 235, 235), 1, Qt::DotLine);
                    painter.setPen(pen);
                    painter.drawLine(x, plotArea.top(), x, plotArea.bottom());
                }
            }
        }
    }

    void FrequencyChartWidget::DrawYLabels(QPainter& painter, const QRect& plotArea, const PanelBounds& bounds)
    {
        QFont labelFont = painter.font();
        labelFont.setPointSize(8);
        labelFont.setBold(false);
        painter.setFont(labelFont);
        painter.setPen(Qt::black);

        for (auto i = 0; i <= yGridLines; ++i)
        {
            auto y = plotArea.bottom() - (i * plotArea.height() / yGridLines);
            auto val = bounds.minY + (static_cast<float>(i) / yGridLines) * (bounds.maxY - bounds.minY);
            painter.drawText(plotArea.left() - 60, y + 4, QString::number(static_cast<double>(val), 'f', 1));
        }
    }

    void FrequencyChartWidget::DrawLegend(QPainter& painter, const QRect& plotArea, const ChartPanel& panel)
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

    void FrequencyChartWidget::DrawCrosshair(QPainter& painter)
    {
        if (!interaction.showCrosshair || cachedLayouts.empty() || freqData.empty())
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
        auto logFreqAtCursor = interaction.viewMin + xRatio * viewSpan;
        auto freqAtCursor = std::pow(10.0f, logFreqAtCursor);

        QStringList lines;
        if (freqAtCursor >= 1000.0f)
            lines << QString("f = %1 kHz").arg(static_cast<double>(freqAtCursor / 1000.0f), 0, 'f', 2);
        else
            lines << QString("f = %1 Hz").arg(static_cast<double>(freqAtCursor), 0, 'f', 1);

        for (const auto& series : panel.series)
        {
            auto count = std::min(freqData.size(), series.data.size());
            if (count == 0)
                continue;

            std::size_t idx = 0;
            auto bestDist = std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < count; ++i)
            {
                if (freqData[i] <= 0.0f)
                    continue;
                auto dist = std::abs(std::log10(freqData[i]) - logFreqAtCursor);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    idx = i;
                }
            }

            lines << QString("%1 = %2").arg(series.name).arg(static_cast<double>(series.data[idx]), 0, 'f', 2);
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

    FrequencyChartWidget::PanelBounds FrequencyChartWidget::ComputeBounds(const ChartPanel& panel) const
    {
        PanelBounds bounds{ std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest() };

        bool hasData = false;
        for (const auto& series : panel.series)
        {
            auto count = std::min(freqData.size(), series.data.size());
            for (std::size_t i = 0; i < count; ++i)
            {
                if (freqData[i] <= 0.0f)
                    continue;
                auto logF = std::log10(freqData[i]);
                if (logF >= interaction.viewMin && logF <= interaction.viewMax)
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

    void FrequencyChartWidget::wheelEvent(QWheelEvent* event)
    {
        auto plotWidth = width() - leftMargin - rightMargin;
        if (plotWidth <= 0)
            return;

        auto cursorXRatio = static_cast<float>(event->position().x() - leftMargin) / plotWidth;
        cursorXRatio = std::clamp(cursorXRatio, 0.0f, 1.0f);

        interaction.Zoom(static_cast<float>(event->angleDelta().y()), cursorXRatio);
        update();
    }

    void FrequencyChartWidget::mousePressEvent(QMouseEvent* event)
    {
        if (event->button() == Qt::LeftButton)
        {
            auto plotWidth = width() - leftMargin - rightMargin;
            interaction.StartPan(event->pos(), leftMargin, plotWidth);
            setCursor(Qt::ClosedHandCursor);
        }
    }

    void FrequencyChartWidget::mouseMoveEvent(QMouseEvent* event)
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

    void FrequencyChartWidget::mouseReleaseEvent(QMouseEvent* event)
    {
        if (event->button() == Qt::LeftButton)
        {
            interaction.EndPan();
            setCursor(Qt::ArrowCursor);
        }
    }

    void FrequencyChartWidget::mouseDoubleClickEvent(QMouseEvent* event)
    {
        Q_UNUSED(event);
        interaction.ResetView();
        update();
    }

    void FrequencyChartWidget::leaveEvent(QEvent* event)
    {
        Q_UNUSED(event);
        interaction.showCrosshair = false;
        update();
    }
}
