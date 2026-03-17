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
        setMinimumSize(400, 300);
        setBackgroundRole(QPalette::Base);
        setAutoFillBackground(true);
    }

    void FftChartWidget::SetData(const FftResult& result)
    {
        frequencies = result.frequencies;
        magnitudes = result.magnitudes;

        maxMagnitude = 0.0f;
        maxFrequency = 0.0f;

        if (!magnitudes.empty())
            maxMagnitude = *std::max_element(magnitudes.begin(), magnitudes.end());

        if (!frequencies.empty())
            maxFrequency = frequencies.back();

        update();
    }

    void FftChartWidget::Clear()
    {
        frequencies.clear();
        magnitudes.clear();
        maxMagnitude = 0.0f;
        maxFrequency = 0.0f;
        update();
    }

    void FftChartWidget::paintEvent(QPaintEvent* event)
    {
        Q_UNUSED(event);

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        const int margin = 60;
        QRect plotArea(margin, 20, width() - margin - 20, height() - margin - 20);

        if (plotArea.width() <= 0 || plotArea.height() <= 0)
            return;

        DrawAxes(painter, plotArea);
        DrawLabels(painter, plotArea);

        if (!frequencies.empty() && !magnitudes.empty())
            DrawSpectrum(painter, plotArea);
    }

    void FftChartWidget::DrawAxes(QPainter& painter, const QRect& plotArea)
    {
        QPen axisPen(Qt::darkGray, 1);
        painter.setPen(axisPen);

        painter.drawLine(plotArea.bottomLeft(), plotArea.bottomRight());
        painter.drawLine(plotArea.bottomLeft(), plotArea.topLeft());

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

    void FftChartWidget::DrawSpectrum(QPainter& painter, const QRect& plotArea)
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

    void FftChartWidget::DrawLabels(QPainter& painter, const QRect& plotArea)
    {
        QPen textPen(Qt::black);
        painter.setPen(textPen);
        QFont font = painter.font();
        font.setPointSize(8);
        painter.setFont(font);

        constexpr int gridLines = 5;

        for (int i = 0; i <= gridLines; ++i)
        {
            int x = plotArea.left() + (i * plotArea.width() / gridLines);
            float freq = maxFrequency > 0.0f ? (static_cast<float>(i) / gridLines) * maxFrequency : 0.0f;
            QString label = QString::number(static_cast<int>(freq)) + " Hz";
            painter.drawText(x - 20, plotArea.bottom() + 15, label);
        }

        for (int i = 0; i <= gridLines; ++i)
        {
            int y = plotArea.bottom() - (i * plotArea.height() / gridLines);
            float mag = maxMagnitude > 0.0f ? (static_cast<float>(i) / gridLines) * maxMagnitude : 0.0f;
            QString label = QString::number(static_cast<double>(mag), 'f', 3);
            painter.drawText(plotArea.left() - 55, y + 4, label);
        }

        font.setPointSize(10);
        painter.setFont(font);
        painter.drawText(plotArea.center().x() - 40, plotArea.bottom() + 35, "Frequency (Hz)");

        painter.save();
        painter.translate(15, plotArea.center().y() + 30);
        painter.rotate(-90);
        painter.drawText(0, 0, "Magnitude");
        painter.restore();
    }
}
