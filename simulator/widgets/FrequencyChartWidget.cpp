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
    }

    void FrequencyChartWidget::SetFrequencyAxis(std::span<const float> frequenciesHz)
    {
        freqData.assign(frequenciesHz.begin(), frequenciesHz.end());

        if (!freqData.empty())
        {
            minFreq = *std::min_element(freqData.begin(), freqData.end());
            maxFreq = *std::max_element(freqData.begin(), freqData.end());

            if (minFreq <= 0.0f)
                minFreq = 1.0f;

            logMinFreq = std::log10(minFreq);
            logMaxFreq = std::log10(maxFreq);
        }

        update();
    }

    void FrequencyChartWidget::SetFrequencyAxis(std::vector<float> frequenciesHz)
    {
        freqData = std::move(frequenciesHz);

        if (!freqData.empty())
        {
            minFreq = *std::min_element(freqData.begin(), freqData.end());
            maxFreq = *std::max_element(freqData.begin(), freqData.end());

            if (minFreq <= 0.0f)
                minFreq = 1.0f;

            logMinFreq = std::log10(minFreq);
            logMaxFreq = std::log10(maxFreq);
        }

        update();
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
        update();
    }

    void FrequencyChartWidget::paintEvent(QPaintEvent* event)
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
        painter.drawText(leftMargin + plotWidth / 2 - 30, lastPanelBottom + 25, "Frequency (Hz)");

        labelFont.setPointSize(8);
        painter.setFont(labelFont);

        if (maxFreq > minFreq && minFreq > 0.0f)
        {
            auto lastPanelY = currentY - panelSpacing;
            auto startDecade = static_cast<int>(std::floor(logMinFreq));
            auto endDecade = static_cast<int>(std::ceil(logMaxFreq));

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

                    QRect labelRect(x - 25, lastPanelY + 2, 50, 14);
                    painter.drawText(labelRect, Qt::AlignCenter, label);
                }
            }
        }
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
        if (maxFreq <= minFreq || series.data.empty())
            return;

        auto range = bounds.maxY - bounds.minY;
        if (range <= 0.0f)
            range = 1.0f;

        QPen pen(series.color, 2);
        painter.setPen(pen);

        QPointF prev;
        auto hasPrev = false;
        auto count = std::min(freqData.size(), series.data.size());
        auto stride = std::max<std::size_t>(1, count / static_cast<std::size_t>(plotArea.width()));

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
        // Y grid lines (linear)
        QPen gridPen(QColor(220, 220, 220), 1, Qt::DashLine);
        painter.setPen(gridPen);

        for (auto i = 1; i <= yGridLines; ++i)
        {
            auto y = plotArea.bottom() - (i * plotArea.height() / yGridLines);
            painter.drawLine(plotArea.left(), y, plotArea.right(), y);
        }

        // X grid lines (logarithmic decades)
        if (maxFreq <= minFreq || minFreq <= 0.0f)
            return;

        auto startDecade = static_cast<int>(std::floor(logMinFreq));
        auto endDecade = static_cast<int>(std::ceil(logMaxFreq));

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

    FrequencyChartWidget::PanelBounds FrequencyChartWidget::ComputeBounds(const ChartPanel& panel) const
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

    float FrequencyChartWidget::FreqToX(float freq, int plotLeft, int plotWidth) const
    {
        if (logMaxFreq <= logMinFreq)
            return static_cast<float>(plotLeft);

        auto logFreq = std::log10(std::max(freq, minFreq));
        auto ratio = (logFreq - logMinFreq) / (logMaxFreq - logMinFreq);
        return static_cast<float>(plotLeft) + ratio * plotWidth;
    }
}
