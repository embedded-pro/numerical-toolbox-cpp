#include "simulator/analysis/FastFourierTransform/view/FftChartWidget.hpp"
#include <QPainter>
#include <QPen>
#include <algorithm>
#include <cmath>

namespace simulator::analysis::view
{
    FftChartWidget::FftChartWidget(QWidget* parent)
        : QWidget(parent)
    {
        setMinimumSize(400, 500);
        setBackgroundRole(QPalette::Base);
        setAutoFillBackground(true);
    }

    void FftChartWidget::SetData(const FftResult& result)
    {
        time = result.time;
        signal = result.signal;
        windowedSignal = result.windowedSignal;
        frequencies = result.frequencies;
        magnitudes = result.magnitudes;

        maxTime = 0.0f;
        maxSignalAmplitude = 0.0f;
        maxMagnitude = 0.0f;
        maxFrequency = 0.0f;

        if (!time.empty())
            maxTime = time.back();

        if (!signal.empty())
        {
            for (auto s : signal)
                maxSignalAmplitude = std::max(maxSignalAmplitude, std::abs(s));
        }

        if (!windowedSignal.empty())
        {
            for (auto s : windowedSignal)
                maxSignalAmplitude = std::max(maxSignalAmplitude, std::abs(s));
        }

        if (maxSignalAmplitude == 0.0f)
            maxSignalAmplitude = 1.0f;

        if (!magnitudes.empty())
            maxMagnitude = *std::max_element(magnitudes.begin(), magnitudes.end());

        maxFrequency = result.sampleRateHz / 2.0f;

        update();
    }

    void FftChartWidget::Clear()
    {
        time.clear();
        signal.clear();
        windowedSignal.clear();
        frequencies.clear();
        magnitudes.clear();
        maxTime = 0.0f;
        maxSignalAmplitude = 0.0f;
        maxMagnitude = 0.0f;
        maxFrequency = 0.0f;
        update();
    }

    void FftChartWidget::paintEvent(QPaintEvent* event)
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
        int freqDomainHeight = availableHeight - timeDomainHeight;
        int plotWidth = width() - leftMargin - rightMargin;

        if (plotWidth <= 0 || timeDomainHeight <= 0 || freqDomainHeight <= 0)
            return;

        QRect timePlot(leftMargin, topMargin, plotWidth, timeDomainHeight);
        QRect freqPlot(leftMargin, topMargin + timeDomainHeight + chartSpacing, plotWidth, freqDomainHeight);

        // Time domain chart
        DrawAxes(painter, timePlot);
        DrawGridLines(painter, timePlot);

        QFont titleFont = painter.font();
        titleFont.setPointSize(10);
        titleFont.setBold(true);
        painter.setFont(titleFont);
        painter.setPen(Qt::black);
        painter.drawText(timePlot.left(), timePlot.top() - 3, "Time Domain");

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
            QString label = QString::number(static_cast<double>(t), 'f', 2);
            QRect labelRect(x - 30, timePlot.bottom() + 4, 60, 14);
            painter.drawText(labelRect, Qt::AlignCenter, label);
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

        // Legend for time domain
        if (!signal.empty())
        {
            labelFont.setPointSize(8);
            painter.setFont(labelFont);

            int legendX = timePlot.right() - 170;
            int legendY = timePlot.top() + 15;

            painter.setPen(QPen(QColor(41, 128, 185), 2));
            painter.drawLine(legendX, legendY, legendX + 20, legendY);
            painter.setPen(Qt::black);
            painter.drawText(legendX + 25, legendY + 4, "Original");

            if (!windowedSignal.empty())
            {
                painter.setPen(QPen(QColor(231, 76, 60), 2));
                painter.drawLine(legendX, legendY + 18, legendX + 20, legendY + 18);
                painter.setPen(Qt::black);
                painter.drawText(legendX + 25, legendY + 22, "Windowed");
            }
        }

        // Frequency domain chart
        DrawAxes(painter, freqPlot);
        DrawGridLines(painter, freqPlot);

        titleFont.setPointSize(10);
        titleFont.setBold(true);
        painter.setFont(titleFont);
        painter.setPen(Qt::black);
        painter.drawText(freqPlot.left(), freqPlot.top() - 3, "Frequency Domain (FFT)");

        if (!frequencies.empty() && !magnitudes.empty())
            DrawFrequencyDomain(painter, freqPlot);

        // Frequency domain labels
        labelFont.setPointSize(8);
        labelFont.setBold(false);
        painter.setFont(labelFont);
        painter.setPen(Qt::black);

        for (int i = 0; i <= gridLines; ++i)
        {
            int x = freqPlot.left() + (i * freqPlot.width() / gridLines);
            float freq = maxFrequency > 0.0f ? (static_cast<float>(i) / gridLines) * maxFrequency : 0.0f;
            QString label;
            if (freq >= 1000.0f)
                label = QString::number(static_cast<double>(freq / 1000.0f), 'f', 1) + " kHz";
            else
                label = QString::number(static_cast<int>(freq)) + " Hz";
            QRect labelRect(x - 35, freqPlot.bottom() + 4, 70, 14);
            painter.drawText(labelRect, Qt::AlignCenter, label);
        }

        for (int i = 0; i <= gridLines; ++i)
        {
            int y = freqPlot.bottom() - (i * freqPlot.height() / gridLines);
            float mag = maxMagnitude > 0.0f ? (static_cast<float>(i) / gridLines) * maxMagnitude : 0.0f;
            painter.drawText(freqPlot.left() - 55, y + 4, QString::number(static_cast<double>(mag), 'f', 3));
        }

        labelFont.setPointSize(9);
        painter.setFont(labelFont);
        painter.drawText(freqPlot.center().x() - 40, freqPlot.bottom() + 30, "Frequency (Hz)");

        painter.save();
        painter.translate(12, freqPlot.center().y() + 30);
        painter.rotate(-90);
        painter.drawText(0, 0, "Magnitude");
        painter.restore();
    }

    void FftChartWidget::DrawAxes(QPainter& painter, const QRect& plotArea)
    {
        QPen axisPen(Qt::darkGray, 1);
        painter.setPen(axisPen);

        painter.drawLine(plotArea.bottomLeft(), plotArea.bottomRight());
        painter.drawLine(plotArea.bottomLeft(), plotArea.topLeft());
    }

    void FftChartWidget::DrawGridLines(QPainter& painter, const QRect& plotArea)
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

    void FftChartWidget::DrawTimeDomain(QPainter& painter, const QRect& plotArea)
    {
        if (maxTime <= 0.0f)
            return;

        auto drawSignalLine = [&](const std::vector<float>& sig, const QColor& color)
        {
            QPen pen(color, 1);
            painter.setPen(pen);

            QPointF previousPoint;
            bool hasPrevious = false;

            const std::size_t count = std::min(time.size(), sig.size());
            for (std::size_t i = 0; i < count; ++i)
            {
                float xRatio = time[i] / maxTime;
                float yRatio = (sig[i] + maxSignalAmplitude) / (2.0f * maxSignalAmplitude);

                int x = plotArea.left() + static_cast<int>(xRatio * plotArea.width());
                int y = plotArea.bottom() - static_cast<int>(yRatio * plotArea.height());

                QPointF currentPoint(x, y);

                if (hasPrevious)
                    painter.drawLine(previousPoint, currentPoint);

                previousPoint = currentPoint;
                hasPrevious = true;
            }
        };

        drawSignalLine(signal, QColor(41, 128, 185));

        if (!windowedSignal.empty())
            drawSignalLine(windowedSignal, QColor(231, 76, 60));
    }

    void FftChartWidget::DrawFrequencyDomain(QPainter& painter, const QRect& plotArea)
    {
        if (maxMagnitude <= 0.0f || maxFrequency <= 0.0f)
            return;

        QPen spectrumPen(QColor(41, 128, 185), 2);
        painter.setPen(spectrumPen);

        QPointF previousPoint;
        bool hasPrevious = false;

        const std::size_t count = std::min(frequencies.size(), magnitudes.size());
        for (std::size_t i = 0; i < count; ++i)
        {
            float xRatio = frequencies[i] / maxFrequency;
            float yRatio = magnitudes[i] / maxMagnitude;

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
