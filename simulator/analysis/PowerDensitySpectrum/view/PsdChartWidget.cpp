#include "simulator/analysis/PowerDensitySpectrum/view/PsdChartWidget.hpp"
#include <QPainter>
#include <QPen>
#include <algorithm>
#include <cmath>

namespace simulator::analysis::psd::view
{
    PsdChartWidget::PsdChartWidget(QWidget* parent)
        : QWidget(parent)
    {
        setMinimumSize(400, 500);
        setBackgroundRole(QPalette::Base);
        setAutoFillBackground(true);
    }

    void PsdChartWidget::SetData(const PsdResult& result)
    {
        time = result.time;
        signal = result.signal;
        frequencies = result.frequencies;
        powerDensityDb = result.powerDensityDb;

        maxTime = 0.0f;
        maxSignalAmplitude = 0.0f;
        minPowerDb = 0.0f;
        maxPowerDb = -100.0f;
        maxFrequency = 0.0f;

        if (!time.empty())
            maxTime = time.back();

        if (!signal.empty())
        {
            for (auto s : signal)
                maxSignalAmplitude = std::max(maxSignalAmplitude, std::abs(s));
        }

        if (maxSignalAmplitude == 0.0f)
            maxSignalAmplitude = 1.0f;

        if (!powerDensityDb.empty())
        {
            minPowerDb = *std::min_element(powerDensityDb.begin(), powerDensityDb.end());
            maxPowerDb = *std::max_element(powerDensityDb.begin(), powerDensityDb.end());

            if (maxPowerDb - minPowerDb < 1.0f)
            {
                minPowerDb -= 5.0f;
                maxPowerDb += 5.0f;
            }
        }

        if (!frequencies.empty())
            maxFrequency = frequencies.back();

        update();
    }

    void PsdChartWidget::Clear()
    {
        time.clear();
        signal.clear();
        frequencies.clear();
        powerDensityDb.clear();
        maxTime = 0.0f;
        maxSignalAmplitude = 0.0f;
        minPowerDb = 0.0f;
        maxPowerDb = 0.0f;
        maxFrequency = 0.0f;
        update();
    }

    void PsdChartWidget::paintEvent(QPaintEvent* event)
    {
        Q_UNUSED(event);

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        constexpr int leftMargin = 60;
        constexpr int rightMargin = 20;
        constexpr int topMargin = 15;
        constexpr int bottomMargin = 45;
        constexpr int chartSpacing = 55;

        int availableHeight = height() - topMargin - bottomMargin - chartSpacing;
        int timeDomainHeight = availableHeight * 2 / 5;
        int psdHeight = availableHeight - timeDomainHeight;
        int plotWidth = width() - leftMargin - rightMargin;

        if (plotWidth <= 0 || timeDomainHeight <= 0 || psdHeight <= 0)
            return;

        QRect timePlot(leftMargin, topMargin, plotWidth, timeDomainHeight);
        QRect psdPlot(leftMargin, topMargin + timeDomainHeight + chartSpacing, plotWidth, psdHeight);

        // Time domain chart
        DrawAxes(painter, timePlot);
        DrawGridLines(painter, timePlot);

        QFont titleFont = painter.font();
        titleFont.setPointSize(10);
        titleFont.setBold(true);
        painter.setFont(titleFont);
        painter.setPen(Qt::black);
        painter.drawText(timePlot.left(), timePlot.top() - 3, "Time Domain (Signal + Noise)");

        if (!time.empty() && !signal.empty())
            DrawTimeDomain(painter, timePlot);

        // Time domain labels
        QFont labelFont = painter.font();
        labelFont.setPointSize(8);
        labelFont.setBold(false);
        painter.setFont(labelFont);
        painter.setPen(Qt::black);

        constexpr int gridLines = 5;
        for (int i = 0; i <= gridLines; ++i)
        {
            int x = timePlot.left() + (i * timePlot.width() / gridLines);
            float t = maxTime > 0.0f ? (static_cast<float>(i) / gridLines) * maxTime * 1000.0f : 0.0f;
            painter.drawText(x - 15, timePlot.bottom() + 14, QString::number(static_cast<double>(t), 'f', 2));
        }

        for (int i = 0; i <= gridLines; ++i)
        {
            int y = timePlot.bottom() - (i * timePlot.height() / gridLines);
            float amp = -maxSignalAmplitude + (static_cast<float>(i) / gridLines) * 2.0f * maxSignalAmplitude;
            painter.drawText(timePlot.left() - 55, y + 4, QString::number(static_cast<double>(amp), 'f', 2));
        }

        labelFont.setPointSize(9);
        painter.setFont(labelFont);
        painter.drawText(timePlot.center().x() - 20, timePlot.bottom() + 30, "Time (ms)");

        painter.save();
        painter.translate(12, timePlot.center().y() + 30);
        painter.rotate(-90);
        painter.drawText(0, 0, "Amplitude");
        painter.restore();

        // PSD chart
        DrawAxes(painter, psdPlot);
        DrawGridLines(painter, psdPlot);

        titleFont.setPointSize(10);
        titleFont.setBold(true);
        painter.setFont(titleFont);
        painter.setPen(Qt::black);
        painter.drawText(psdPlot.left(), psdPlot.top() - 3, "Power Spectral Density");

        if (!frequencies.empty() && !powerDensityDb.empty())
            DrawPowerSpectralDensity(painter, psdPlot);

        // PSD labels
        labelFont.setPointSize(8);
        labelFont.setBold(false);
        painter.setFont(labelFont);
        painter.setPen(Qt::black);

        for (int i = 0; i <= gridLines; ++i)
        {
            int x = psdPlot.left() + (i * psdPlot.width() / gridLines);
            float freq = maxFrequency > 0.0f ? (static_cast<float>(i) / gridLines) * maxFrequency : 0.0f;
            painter.drawText(x - 20, psdPlot.bottom() + 14, QString::number(static_cast<int>(freq)) + " Hz");
        }

        float powerRange = maxPowerDb - minPowerDb;
        for (int i = 0; i <= gridLines; ++i)
        {
            int y = psdPlot.bottom() - (i * psdPlot.height() / gridLines);
            float db = minPowerDb + (static_cast<float>(i) / gridLines) * powerRange;
            painter.drawText(psdPlot.left() - 55, y + 4, QString::number(static_cast<double>(db), 'f', 1));
        }

        labelFont.setPointSize(9);
        painter.setFont(labelFont);
        painter.drawText(psdPlot.center().x() - 40, psdPlot.bottom() + 30, "Frequency (Hz)");

        painter.save();
        painter.translate(12, psdPlot.center().y() + 40);
        painter.rotate(-90);
        painter.drawText(0, 0, "Power (dB/Hz)");
        painter.restore();
    }

    void PsdChartWidget::DrawAxes(QPainter& painter, const QRect& plotArea)
    {
        QPen axisPen(Qt::darkGray, 1);
        painter.setPen(axisPen);

        painter.drawLine(plotArea.bottomLeft(), plotArea.bottomRight());
        painter.drawLine(plotArea.bottomLeft(), plotArea.topLeft());
    }

    void PsdChartWidget::DrawGridLines(QPainter& painter, const QRect& plotArea)
    {
        QPen gridPen(QColor(220, 220, 220), 1, Qt::DashLine);
        painter.setPen(gridPen);

        constexpr int gridLines = 5;
        for (int i = 1; i <= gridLines; ++i)
        {
            int y = plotArea.bottom() - (i * plotArea.height() / gridLines);
            painter.drawLine(plotArea.left(), y, plotArea.right(), y);
        }

        for (int i = 1; i <= gridLines; ++i)
        {
            int x = plotArea.left() + (i * plotArea.width() / gridLines);
            painter.drawLine(x, plotArea.top(), x, plotArea.bottom());
        }
    }

    void PsdChartWidget::DrawTimeDomain(QPainter& painter, const QRect& plotArea)
    {
        if (maxTime <= 0.0f)
            return;

        QPen pen(QColor(41, 128, 185), 1);
        painter.setPen(pen);

        QPointF previousPoint;
        bool hasPrevious = false;

        const std::size_t count = std::min(time.size(), signal.size());
        for (std::size_t i = 0; i < count; ++i)
        {
            float xRatio = time[i] / maxTime;
            float yRatio = (signal[i] + maxSignalAmplitude) / (2.0f * maxSignalAmplitude);

            int x = plotArea.left() + static_cast<int>(xRatio * plotArea.width());
            int y = plotArea.bottom() - static_cast<int>(yRatio * plotArea.height());

            QPointF currentPoint(x, y);

            if (hasPrevious)
                painter.drawLine(previousPoint, currentPoint);

            previousPoint = currentPoint;
            hasPrevious = true;
        }
    }

    void PsdChartWidget::DrawPowerSpectralDensity(QPainter& painter, const QRect& plotArea)
    {
        if (maxFrequency <= 0.0f)
            return;

        float powerRange = maxPowerDb - minPowerDb;
        if (powerRange <= 0.0f)
            return;

        QPen spectrumPen(QColor(192, 57, 43), 2);
        painter.setPen(spectrumPen);

        QPointF previousPoint;
        bool hasPrevious = false;

        const std::size_t count = std::min(frequencies.size(), powerDensityDb.size());
        for (std::size_t i = 0; i < count; ++i)
        {
            float xRatio = frequencies[i] / maxFrequency;
            float yRatio = (powerDensityDb[i] - minPowerDb) / powerRange;

            int x = plotArea.left() + static_cast<int>(xRatio * plotArea.width());
            int y = plotArea.bottom() - static_cast<int>(yRatio * plotArea.height());

            QPointF currentPoint(x, y);

            if (hasPrevious)
                painter.drawLine(previousPoint, currentPoint);

            previousPoint = currentPoint;
            hasPrevious = true;
        }
    }
}
