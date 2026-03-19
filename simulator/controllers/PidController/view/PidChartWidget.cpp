#include "simulator/controllers/PidController/view/PidChartWidget.hpp"
#include <QPainter>
#include <QPen>
#include <algorithm>
#include <cmath>

namespace simulator::controllers::view
{
    // ==================== PidTimeResponseWidget ====================

    PidTimeResponseWidget::PidTimeResponseWidget(const QString& title, QWidget* parent)
        : QWidget(parent)
        , chartTitle(title)
    {
        setMinimumSize(400, 500);
        setBackgroundRole(QPalette::Base);
        setAutoFillBackground(true);
    }

    void PidTimeResponseWidget::SetData(const TimeResponse& result)
    {
        time = result.time;
        reference = result.reference;
        output = result.output;
        controlSignal = result.controlSignal;
        error = result.error;

        maxTime = time.empty() ? 0.0f : time.back();
        maxOutput = 0.0f;
        minOutput = 0.0f;
        maxControl = 0.0f;
        minControl = 0.0f;
        maxError = 0.0f;
        minError = 0.0f;

        for (std::size_t i = 0; i < output.size(); ++i)
        {
            maxOutput = std::max(maxOutput, std::max(output[i], reference[i]));
            minOutput = std::min(minOutput, std::min(output[i], reference[i]));
        }

        for (auto v : controlSignal)
        {
            maxControl = std::max(maxControl, v);
            minControl = std::min(minControl, v);
        }

        for (auto v : error)
        {
            maxError = std::max(maxError, v);
            minError = std::min(minError, v);
        }

        float outPad = (maxOutput - minOutput) * 0.1f;
        if (outPad < 0.1f)
            outPad = 0.1f;
        maxOutput += outPad;
        minOutput -= outPad;

        float ctrlPad = (maxControl - minControl) * 0.1f;
        if (ctrlPad < 0.1f)
            ctrlPad = 0.1f;
        maxControl += ctrlPad;
        minControl -= ctrlPad;

        float errPad = (maxError - minError) * 0.1f;
        if (errPad < 0.1f)
            errPad = 0.1f;
        maxError += errPad;
        minError -= errPad;

        update();
    }

    void PidTimeResponseWidget::paintEvent(QPaintEvent* event)
    {
        Q_UNUSED(event);

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        constexpr int leftMargin = 65;
        constexpr int rightMargin = 20;
        constexpr int topMargin = 15;
        constexpr int bottomMargin = 35;
        constexpr int chartSpacing = 45;

        int availableHeight = height() - topMargin - bottomMargin - 2 * chartSpacing;
        int outputHeight = availableHeight * 2 / 5;
        int controlHeight = availableHeight * 3 / 10;
        int errorHeight = availableHeight - outputHeight - controlHeight;
        int plotWidth = width() - leftMargin - rightMargin;

        if (plotWidth <= 0 || outputHeight <= 0)
            return;

        QRect outputPlot(leftMargin, topMargin, plotWidth, outputHeight);
        QRect controlPlot(leftMargin, topMargin + outputHeight + chartSpacing, plotWidth, controlHeight);
        QRect errorPlot(leftMargin, topMargin + outputHeight + chartSpacing + controlHeight + chartSpacing, plotWidth, errorHeight);

        QFont titleFont = painter.font();
        titleFont.setPointSize(10);
        titleFont.setBold(true);

        QFont labelFont = painter.font();
        labelFont.setPointSize(8);
        labelFont.setBold(false);

        // Output plot
        DrawAxes(painter, outputPlot);
        DrawGridLines(painter, outputPlot);
        painter.setFont(titleFont);
        painter.setPen(Qt::black);
        painter.drawText(outputPlot.left(), outputPlot.top() - 3, chartTitle + " - Output vs Reference");
        if (!time.empty())
            DrawOutputChart(painter, outputPlot);

        // Y-axis labels for output
        painter.setFont(labelFont);
        painter.setPen(Qt::black);
        constexpr int gridLines = 5;
        for (int i = 0; i <= gridLines; ++i)
        {
            int y = outputPlot.bottom() - (i * outputPlot.height() / gridLines);
            float val = minOutput + (static_cast<float>(i) / gridLines) * (maxOutput - minOutput);
            painter.drawText(outputPlot.left() - 60, y + 4, QString::number(static_cast<double>(val), 'f', 2));
        }

        // X-axis labels
        for (int i = 0; i <= gridLines; ++i)
        {
            int x = outputPlot.left() + (i * outputPlot.width() / gridLines);
            float t = maxTime > 0.0f ? (static_cast<float>(i) / gridLines) * maxTime : 0.0f;
            QRect labelRect(x - 25, outputPlot.bottom() + 2, 50, 14);
            painter.drawText(labelRect, Qt::AlignCenter, QString::number(static_cast<double>(t), 'f', 1));
        }

        // Legend
        int legendX = outputPlot.right() - 170;
        int legendY = outputPlot.top() + 15;
        painter.setPen(QPen(QColor(231, 76, 60), 2));
        painter.drawLine(legendX, legendY, legendX + 20, legendY);
        painter.setPen(Qt::black);
        painter.drawText(legendX + 25, legendY + 4, "Reference");
        painter.setPen(QPen(QColor(41, 128, 185), 2));
        painter.drawLine(legendX, legendY + 16, legendX + 20, legendY + 16);
        painter.setPen(Qt::black);
        painter.drawText(legendX + 25, legendY + 20, "Output");

        // Control signal plot
        DrawAxes(painter, controlPlot);
        DrawGridLines(painter, controlPlot);
        painter.setFont(titleFont);
        painter.setPen(Qt::black);
        painter.drawText(controlPlot.left(), controlPlot.top() - 3, "Control Signal");
        if (!time.empty())
            DrawControlChart(painter, controlPlot);

        painter.setFont(labelFont);
        painter.setPen(Qt::black);
        for (int i = 0; i <= gridLines; ++i)
        {
            int y = controlPlot.bottom() - (i * controlPlot.height() / gridLines);
            float val = minControl + (static_cast<float>(i) / gridLines) * (maxControl - minControl);
            painter.drawText(controlPlot.left() - 60, y + 4, QString::number(static_cast<double>(val), 'f', 1));
        }

        // Error plot
        DrawAxes(painter, errorPlot);
        DrawGridLines(painter, errorPlot);
        painter.setFont(titleFont);
        painter.setPen(Qt::black);
        painter.drawText(errorPlot.left(), errorPlot.top() - 3, "Error");
        if (!time.empty())
            DrawErrorChart(painter, errorPlot);

        painter.setFont(labelFont);
        painter.setPen(Qt::black);
        for (int i = 0; i <= gridLines; ++i)
        {
            int y = errorPlot.bottom() - (i * errorPlot.height() / gridLines);
            float val = minError + (static_cast<float>(i) / gridLines) * (maxError - minError);
            painter.drawText(errorPlot.left() - 60, y + 4, QString::number(static_cast<double>(val), 'f', 2));
        }

        // Common x-axis label
        labelFont.setPointSize(9);
        painter.setFont(labelFont);
        painter.drawText(errorPlot.center().x() - 20, errorPlot.bottom() + 25, "Time (s)");
    }

    void PidTimeResponseWidget::DrawAxes(QPainter& painter, const QRect& plotArea)
    {
        QPen axisPen(Qt::darkGray, 1);
        painter.setPen(axisPen);
        painter.drawLine(plotArea.bottomLeft(), plotArea.bottomRight());
        painter.drawLine(plotArea.bottomLeft(), plotArea.topLeft());
    }

    void PidTimeResponseWidget::DrawGridLines(QPainter& painter, const QRect& plotArea)
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

    void PidTimeResponseWidget::DrawOutputChart(QPainter& painter, const QRect& plotArea)
    {
        if (maxTime <= 0.0f)
            return;

        float range = maxOutput - minOutput;
        if (range <= 0.0f)
            range = 1.0f;

        auto drawLine = [&](const std::vector<float>& data, const QColor& color)
        {
            QPen pen(color, 2);
            painter.setPen(pen);

            QPointF prev;
            bool hasPrev = false;
            std::size_t count = std::min(time.size(), data.size());
            for (std::size_t i = 0; i < count; ++i)
            {
                float xRatio = time[i] / maxTime;
                float yRatio = (data[i] - minOutput) / range;

                int x = plotArea.left() + static_cast<int>(xRatio * plotArea.width());
                int y = plotArea.bottom() - static_cast<int>(yRatio * plotArea.height());

                QPointF curr(x, y);
                if (hasPrev)
                    painter.drawLine(prev, curr);
                prev = curr;
                hasPrev = true;
            }
        };

        drawLine(reference, QColor(231, 76, 60));
        drawLine(output, QColor(41, 128, 185));
    }

    void PidTimeResponseWidget::DrawControlChart(QPainter& painter, const QRect& plotArea)
    {
        if (maxTime <= 0.0f)
            return;

        float range = maxControl - minControl;
        if (range <= 0.0f)
            range = 1.0f;

        QPen pen(QColor(39, 174, 96), 2);
        painter.setPen(pen);

        QPointF prev;
        bool hasPrev = false;
        std::size_t count = std::min(time.size(), controlSignal.size());
        for (std::size_t i = 0; i < count; ++i)
        {
            float xRatio = time[i] / maxTime;
            float yRatio = (controlSignal[i] - minControl) / range;

            int x = plotArea.left() + static_cast<int>(xRatio * plotArea.width());
            int y = plotArea.bottom() - static_cast<int>(yRatio * plotArea.height());

            QPointF curr(x, y);
            if (hasPrev)
                painter.drawLine(prev, curr);
            prev = curr;
            hasPrev = true;
        }
    }

    void PidTimeResponseWidget::DrawErrorChart(QPainter& painter, const QRect& plotArea)
    {
        if (maxTime <= 0.0f)
            return;

        float range = maxError - minError;
        if (range <= 0.0f)
            range = 1.0f;

        QPen pen(QColor(142, 68, 173), 2);
        painter.setPen(pen);

        QPointF prev;
        bool hasPrev = false;
        std::size_t count = std::min(time.size(), error.size());
        for (std::size_t i = 0; i < count; ++i)
        {
            float xRatio = time[i] / maxTime;
            float yRatio = (error[i] - minError) / range;

            int x = plotArea.left() + static_cast<int>(xRatio * plotArea.width());
            int y = plotArea.bottom() - static_cast<int>(yRatio * plotArea.height());

            QPointF curr(x, y);
            if (hasPrev)
                painter.drawLine(prev, curr);
            prev = curr;
            hasPrev = true;
        }
    }

    // ==================== PidBodeWidget ====================

    PidBodeWidget::PidBodeWidget(QWidget* parent)
        : QWidget(parent)
    {
        setMinimumSize(400, 500);
        setBackgroundRole(QPalette::Base);
        setAutoFillBackground(true);
    }

    void PidBodeWidget::SetData(const BodeResult& result)
    {
        frequencies = result.frequencies;
        magnitudeDb = result.magnitudeDb;
        phaseDeg = result.phaseDeg;
        gainMarginDb = result.gainMarginDb;
        phaseMarginDeg = result.phaseMarginDeg;
        gainCrossoverHz = result.gainCrossoverHz;
        phaseCrossoverHz = result.phaseCrossoverHz;
        update();
    }

    void PidBodeWidget::paintEvent(QPaintEvent* event)
    {
        Q_UNUSED(event);

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        constexpr int leftMargin = 65;
        constexpr int rightMargin = 20;
        constexpr int topMargin = 15;
        constexpr int bottomMargin = 40;
        constexpr int chartSpacing = 55;

        int availableHeight = height() - topMargin - bottomMargin - chartSpacing;
        int magHeight = availableHeight / 2;
        int phaseHeight = availableHeight - magHeight;
        int plotWidth = width() - leftMargin - rightMargin;

        if (plotWidth <= 0 || magHeight <= 0 || phaseHeight <= 0)
            return;

        QRect magPlot(leftMargin, topMargin, plotWidth, magHeight);
        QRect phasePlot(leftMargin, topMargin + magHeight + chartSpacing, plotWidth, phaseHeight);

        float fMin = 0.001f;
        float fMax = 200.0f;
        if (!frequencies.empty())
        {
            fMin = *std::min_element(frequencies.begin(), frequencies.end());
            fMax = *std::max_element(frequencies.begin(), frequencies.end());
            if (fMin <= 0.0f)
                fMin = 0.001f;
        }

        QFont titleFont = painter.font();
        titleFont.setPointSize(10);
        titleFont.setBold(true);

        QFont labelFont = painter.font();
        labelFont.setPointSize(8);
        labelFont.setBold(false);

        // Magnitude plot
        DrawAxes(painter, magPlot);
        DrawLogGridLines(painter, magPlot, fMin, fMax);
        painter.setFont(titleFont);
        painter.setPen(Qt::black);
        painter.drawText(magPlot.left(), magPlot.top() - 3, "Bode Plot - Magnitude");
        if (!frequencies.empty())
            DrawMagnitudePlot(painter, magPlot);

        // Magnitude axis labels
        painter.setFont(labelFont);
        painter.setPen(Qt::black);

        float magMin = -60.0f;
        float magMax = 60.0f;
        if (!magnitudeDb.empty())
        {
            magMin = *std::min_element(magnitudeDb.begin(), magnitudeDb.end());
            magMax = *std::max_element(magnitudeDb.begin(), magnitudeDb.end());
            float pad = (magMax - magMin) * 0.1f;
            if (pad < 5.0f)
                pad = 5.0f;
            magMin -= pad;
            magMax += pad;
        }

        constexpr int gridLines = 5;
        for (int i = 0; i <= gridLines; ++i)
        {
            int y = magPlot.bottom() - (i * magPlot.height() / gridLines);
            float val = magMin + (static_cast<float>(i) / gridLines) * (magMax - magMin);
            painter.drawText(magPlot.left() - 60, y + 4, QString::number(static_cast<double>(val), 'f', 0) + " dB");
        }

        // Draw 0 dB line
        if (magMin < 0.0f && magMax > 0.0f)
        {
            float yRatio = (0.0f - magMin) / (magMax - magMin);
            int y0 = magPlot.bottom() - static_cast<int>(yRatio * magPlot.height());
            QPen zeroPen(Qt::red, 1, Qt::DashLine);
            painter.setPen(zeroPen);
            painter.drawLine(magPlot.left(), y0, magPlot.right(), y0);
        }

        // Gain margin annotation
        if (gainMarginDb != 0.0f)
        {
            painter.setFont(labelFont);
            painter.setPen(QColor(231, 76, 60));
            painter.drawText(magPlot.right() - 200, magPlot.bottom() - 10,
                QString("GM: %1 dB @ %2 Hz")
                    .arg(static_cast<double>(gainMarginDb), 0, 'f', 1)
                    .arg(static_cast<double>(phaseCrossoverHz), 0, 'f', 2));
        }

        // Frequency labels for magnitude plot (log scale)
        float logFMin = std::log10(fMin);
        float logFMax = std::log10(fMax);
        for (int i = 0; i <= gridLines; ++i)
        {
            int x = magPlot.left() + (i * magPlot.width() / gridLines);
            float logF = logFMin + (static_cast<float>(i) / gridLines) * (logFMax - logFMin);
            float freq = std::pow(10.0f, logF);
            QString label;
            if (freq >= 1000.0f)
                label = QString::number(static_cast<double>(freq / 1000.0f), 'f', 1) + "k";
            else if (freq >= 1.0f)
                label = QString::number(static_cast<double>(freq), 'f', 1);
            else
                label = QString::number(static_cast<double>(freq), 'f', 3);
            QRect labelRect(x - 30, magPlot.bottom() + 2, 60, 14);
            painter.setPen(Qt::black);
            painter.drawText(labelRect, Qt::AlignCenter, label);
        }

        // Phase plot
        DrawAxes(painter, phasePlot);
        DrawLogGridLines(painter, phasePlot, fMin, fMax);
        painter.setFont(titleFont);
        painter.setPen(Qt::black);
        painter.drawText(phasePlot.left(), phasePlot.top() - 3, "Bode Plot - Phase");
        if (!frequencies.empty())
            DrawPhasePlot(painter, phasePlot);

        // Phase axis labels
        painter.setFont(labelFont);
        painter.setPen(Qt::black);

        float phMin = -270.0f;
        float phMax = 90.0f;
        if (!phaseDeg.empty())
        {
            phMin = *std::min_element(phaseDeg.begin(), phaseDeg.end());
            phMax = *std::max_element(phaseDeg.begin(), phaseDeg.end());
            float pad = (phMax - phMin) * 0.1f;
            if (pad < 10.0f)
                pad = 10.0f;
            phMin -= pad;
            phMax += pad;
        }

        for (int i = 0; i <= gridLines; ++i)
        {
            int y = phasePlot.bottom() - (i * phasePlot.height() / gridLines);
            float val = phMin + (static_cast<float>(i) / gridLines) * (phMax - phMin);
            painter.drawText(phasePlot.left() - 55, y + 4, QString::number(static_cast<double>(val), 'f', 0) + QString::fromUtf8("\xC2\xB0"));
        }

        // Draw -180 line
        if (phMin < -180.0f && phMax > -180.0f)
        {
            float yRatio = (-180.0f - phMin) / (phMax - phMin);
            int y180 = phasePlot.bottom() - static_cast<int>(yRatio * phasePlot.height());
            QPen linePen(Qt::red, 1, Qt::DashLine);
            painter.setPen(linePen);
            painter.drawLine(phasePlot.left(), y180, phasePlot.right(), y180);
        }

        // Phase margin annotation
        if (phaseMarginDeg != 0.0f)
        {
            painter.setFont(labelFont);
            painter.setPen(QColor(41, 128, 185));
            painter.drawText(phasePlot.right() - 200, phasePlot.bottom() - 10,
                QString("PM: %1%2 @ %3 Hz")
                    .arg(static_cast<double>(phaseMarginDeg), 0, 'f', 1)
                    .arg(QString::fromUtf8("\xC2\xB0"))
                    .arg(static_cast<double>(gainCrossoverHz), 0, 'f', 2));
        }

        // Frequency labels for phase plot
        for (int i = 0; i <= gridLines; ++i)
        {
            int x = phasePlot.left() + (i * phasePlot.width() / gridLines);
            float logF = logFMin + (static_cast<float>(i) / gridLines) * (logFMax - logFMin);
            float freq = std::pow(10.0f, logF);
            QString label;
            if (freq >= 1000.0f)
                label = QString::number(static_cast<double>(freq / 1000.0f), 'f', 1) + "k";
            else if (freq >= 1.0f)
                label = QString::number(static_cast<double>(freq), 'f', 1);
            else
                label = QString::number(static_cast<double>(freq), 'f', 3);
            QRect labelRect(x - 30, phasePlot.bottom() + 2, 60, 14);
            painter.setPen(Qt::black);
            painter.drawText(labelRect, Qt::AlignCenter, label);
        }

        labelFont.setPointSize(9);
        painter.setFont(labelFont);
        painter.setPen(Qt::black);
        painter.drawText(phasePlot.center().x() - 30, phasePlot.bottom() + 30, "Frequency (Hz)");
    }

    void PidBodeWidget::DrawAxes(QPainter& painter, const QRect& plotArea)
    {
        QPen axisPen(Qt::darkGray, 1);
        painter.setPen(axisPen);
        painter.drawLine(plotArea.bottomLeft(), plotArea.bottomRight());
        painter.drawLine(plotArea.bottomLeft(), plotArea.topLeft());
    }

    void PidBodeWidget::DrawLogGridLines(QPainter& painter, const QRect& plotArea, float fMin, float fMax)
    {
        QPen gridPen(QColor(220, 220, 220), 1, Qt::DashLine);
        painter.setPen(gridPen);

        constexpr int gridLines = 5;
        for (int i = 1; i <= gridLines; ++i)
        {
            int y = plotArea.bottom() - (i * plotArea.height() / gridLines);
            painter.drawLine(plotArea.left(), y, plotArea.right(), y);
        }

        // Logarithmic vertical grid
        float logMin = std::log10(fMin);
        float logMax = std::log10(fMax);
        int decadeStart = static_cast<int>(std::floor(logMin));
        int decadeEnd = static_cast<int>(std::ceil(logMax));

        for (int decade = decadeStart; decade <= decadeEnd; ++decade)
        {
            for (int sub = 1; sub < 10; ++sub)
            {
                float freq = std::pow(10.0f, static_cast<float>(decade)) * sub;
                if (freq < fMin || freq > fMax)
                    continue;

                float logRatio = (std::log10(freq) - logMin) / (logMax - logMin);
                int x = plotArea.left() + static_cast<int>(logRatio * plotArea.width());

                if (sub == 1)
                    painter.setPen(QPen(QColor(180, 180, 180), 1, Qt::DashLine));
                else
                    painter.setPen(QPen(QColor(230, 230, 230), 1, Qt::DotLine));

                painter.drawLine(x, plotArea.top(), x, plotArea.bottom());
            }
        }
    }

    void PidBodeWidget::DrawMagnitudePlot(QPainter& painter, const QRect& plotArea)
    {
        if (frequencies.empty())
            return;

        float fMin = *std::min_element(frequencies.begin(), frequencies.end());
        float fMax = *std::max_element(frequencies.begin(), frequencies.end());
        if (fMin <= 0.0f)
            fMin = 0.001f;

        float magMin = *std::min_element(magnitudeDb.begin(), magnitudeDb.end());
        float magMax = *std::max_element(magnitudeDb.begin(), magnitudeDb.end());
        float pad = (magMax - magMin) * 0.1f;
        if (pad < 5.0f)
            pad = 5.0f;
        magMin -= pad;
        magMax += pad;

        float logFMin = std::log10(fMin);
        float logFMax = std::log10(fMax);
        float magRange = magMax - magMin;
        if (magRange <= 0.0f)
            magRange = 1.0f;

        QPen pen(QColor(41, 128, 185), 2);
        painter.setPen(pen);

        QPointF prev;
        bool hasPrev = false;
        for (std::size_t i = 0; i < frequencies.size(); ++i)
        {
            if (frequencies[i] <= 0.0f)
                continue;

            float xRatio = (std::log10(frequencies[i]) - logFMin) / (logFMax - logFMin);
            float yRatio = (magnitudeDb[i] - magMin) / magRange;

            int x = plotArea.left() + static_cast<int>(xRatio * plotArea.width());
            int y = plotArea.bottom() - static_cast<int>(yRatio * plotArea.height());

            QPointF curr(x, y);
            if (hasPrev)
                painter.drawLine(prev, curr);
            prev = curr;
            hasPrev = true;
        }
    }

    void PidBodeWidget::DrawPhasePlot(QPainter& painter, const QRect& plotArea)
    {
        if (frequencies.empty())
            return;

        float fMin = *std::min_element(frequencies.begin(), frequencies.end());
        float fMax = *std::max_element(frequencies.begin(), frequencies.end());
        if (fMin <= 0.0f)
            fMin = 0.001f;

        float phMin = *std::min_element(phaseDeg.begin(), phaseDeg.end());
        float phMax = *std::max_element(phaseDeg.begin(), phaseDeg.end());
        float pad = (phMax - phMin) * 0.1f;
        if (pad < 10.0f)
            pad = 10.0f;
        phMin -= pad;
        phMax += pad;

        float logFMin = std::log10(fMin);
        float logFMax = std::log10(fMax);
        float phRange = phMax - phMin;
        if (phRange <= 0.0f)
            phRange = 1.0f;

        QPen pen(QColor(39, 174, 96), 2);
        painter.setPen(pen);

        QPointF prev;
        bool hasPrev = false;
        for (std::size_t i = 0; i < frequencies.size(); ++i)
        {
            if (frequencies[i] <= 0.0f)
                continue;

            float xRatio = (std::log10(frequencies[i]) - logFMin) / (logFMax - logFMin);
            float yRatio = (phaseDeg[i] - phMin) / phRange;

            int x = plotArea.left() + static_cast<int>(xRatio * plotArea.width());
            int y = plotArea.bottom() - static_cast<int>(yRatio * plotArea.height());

            QPointF curr(x, y);
            if (hasPrev)
                painter.drawLine(prev, curr);
            prev = curr;
            hasPrev = true;
        }
    }

    // ==================== PidRootLocusWidget ====================

    PidRootLocusWidget::PidRootLocusWidget(QWidget* parent)
        : QWidget(parent)
    {
        setMinimumSize(400, 400);
        setBackgroundRole(QPalette::Base);
        setAutoFillBackground(true);
        setMouseTracking(true);
        setCursor(Qt::CrossCursor);
    }

    void PidRootLocusWidget::SetData(const RootLocusResult& result)
    {
        loci = result.loci;
        gains = result.gains;
        openLoopPoles = result.openLoopPoles;
        openLoopZeros = result.openLoopZeros;
        closedLoopPoles = result.closedLoopPoles;
        currentGain = result.currentGain;

        // Auto-fit view bounds
        float rMin = 0.0f, rMax = 0.0f, iMin = 0.0f, iMax = 0.0f;

        auto expandBounds = [&](std::complex<float> c)
        {
            rMin = std::min(rMin, c.real());
            rMax = std::max(rMax, c.real());
            iMin = std::min(iMin, c.imag());
            iMax = std::max(iMax, c.imag());
        };

        for (const auto& pole : openLoopPoles)
            expandBounds(pole);
        for (const auto& zero : openLoopZeros)
            expandBounds(zero);
        for (const auto& locus : loci)
            for (const auto& point : locus)
                expandBounds(point);

        float rPad = (rMax - rMin) * 0.2f;
        float iPad = (iMax - iMin) * 0.2f;
        if (rPad < 1.0f)
            rPad = 1.0f;
        if (iPad < 1.0f)
            iPad = 1.0f;

        viewRealMin = rMin - rPad;
        viewRealMax = rMax + rPad;
        viewImagMin = iMin - iPad;
        viewImagMax = iMax + iPad;

        // Ensure symmetric about imaginary axis
        float absMax = std::max(std::abs(viewImagMin), std::abs(viewImagMax));
        viewImagMin = -absMax;
        viewImagMax = absMax;

        update();
    }

    QRect PidRootLocusWidget::GetPlotArea() const
    {
        constexpr int leftMargin = 65;
        constexpr int rightMargin = 20;
        constexpr int topMargin = 30;
        constexpr int bottomMargin = 45;

        return QRect(leftMargin, topMargin, width() - leftMargin - rightMargin, height() - topMargin - bottomMargin);
    }

    QPointF PidRootLocusWidget::ComplexToPixel(std::complex<float> c, const QRect& plotArea) const
    {
        float xRatio = (c.real() - viewRealMin) / (viewRealMax - viewRealMin);
        float yRatio = (c.imag() - viewImagMin) / (viewImagMax - viewImagMin);

        float x = plotArea.left() + xRatio * plotArea.width();
        float y = plotArea.bottom() - yRatio * plotArea.height();

        return QPointF(x, y);
    }

    std::complex<float> PidRootLocusWidget::PixelToComplex(QPointF pixel, const QRect& plotArea) const
    {
        float xRatio = static_cast<float>(pixel.x() - plotArea.left()) / plotArea.width();
        float yRatio = static_cast<float>(plotArea.bottom() - pixel.y()) / plotArea.height();

        float real = viewRealMin + xRatio * (viewRealMax - viewRealMin);
        float imag = viewImagMin + yRatio * (viewImagMax - viewImagMin);

        return std::complex<float>(real, imag);
    }

    float PidRootLocusWidget::FindNearestGain(std::complex<float> point) const
    {
        float bestGain = currentGain;
        float bestDist = 1e30f;

        for (const auto& locus : loci)
        {
            for (std::size_t i = 0; i < locus.size() && i < gains.size(); ++i)
            {
                float dist = std::abs(locus[i] - point);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestGain = gains[i];
                }
            }
        }

        return bestGain;
    }

    void PidRootLocusWidget::paintEvent(QPaintEvent* event)
    {
        Q_UNUSED(event);

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        QRect plotArea = GetPlotArea();
        if (plotArea.width() <= 0 || plotArea.height() <= 0)
            return;

        QFont titleFont = painter.font();
        titleFont.setPointSize(10);
        titleFont.setBold(true);

        QFont labelFont = painter.font();
        labelFont.setPointSize(8);
        labelFont.setBold(false);

        // Title
        painter.setFont(titleFont);
        painter.setPen(Qt::black);
        painter.drawText(plotArea.left(), plotArea.top() - 10, "Root Locus (drag closed-loop poles to adjust gain)");

        // Background rendering order
        DrawStabilityRegion(painter, plotArea);
        DrawGridLines(painter, plotArea);
        DrawAxes(painter, plotArea);
        DrawLoci(painter, plotArea);
        DrawZeros(painter, plotArea);
        DrawPoles(painter, plotArea);
        DrawClosedLoopPoles(painter, plotArea);

        // Axis labels
        painter.setFont(labelFont);
        painter.setPen(Qt::black);

        constexpr int gridLines = 5;
        for (int i = 0; i <= gridLines; ++i)
        {
            int x = plotArea.left() + (i * plotArea.width() / gridLines);
            float val = viewRealMin + (static_cast<float>(i) / gridLines) * (viewRealMax - viewRealMin);
            QRect labelRect(x - 25, plotArea.bottom() + 4, 50, 14);
            painter.drawText(labelRect, Qt::AlignCenter, QString::number(static_cast<double>(val), 'f', 1));
        }

        for (int i = 0; i <= gridLines; ++i)
        {
            int y = plotArea.bottom() - (i * plotArea.height() / gridLines);
            float val = viewImagMin + (static_cast<float>(i) / gridLines) * (viewImagMax - viewImagMin);
            painter.drawText(plotArea.left() - 60, y + 4, QString::number(static_cast<double>(val), 'f', 1) + "j");
        }

        labelFont.setPointSize(9);
        painter.setFont(labelFont);
        painter.drawText(plotArea.center().x() - 20, plotArea.bottom() + 35, "Real");

        painter.save();
        painter.translate(12, plotArea.center().y() + 25);
        painter.rotate(-90);
        painter.drawText(0, 0, "Imaginary");
        painter.restore();

        // Current gain display
        painter.setFont(labelFont);
        painter.setPen(QColor(41, 128, 185));
        painter.drawText(plotArea.right() - 150, plotArea.top() + 15,
            QString("K = %1").arg(static_cast<double>(currentGain), 0, 'f', 3));

        // Legend
        int legendX = plotArea.left() + 10;
        int legendY = plotArea.top() + 10;

        painter.setPen(QPen(Qt::blue, 2));
        painter.drawLine(legendX, legendY, legendX + 15, legendY);
        painter.setPen(Qt::black);
        painter.drawText(legendX + 20, legendY + 4, "Locus");

        painter.setPen(Qt::red);
        painter.drawText(legendX, legendY + 18, "x");
        painter.setPen(Qt::black);
        painter.drawText(legendX + 20, legendY + 22, "OL Poles");

        painter.setPen(QColor(39, 174, 96));
        painter.drawText(legendX, legendY + 36, "o");
        painter.setPen(Qt::black);
        painter.drawText(legendX + 20, legendY + 40, "OL Zeros");

        painter.setPen(QColor(142, 68, 173));
        painter.drawText(legendX, legendY + 54, QString::fromUtf8("\xE2\x97\x86"));
        painter.setPen(Qt::black);
        painter.drawText(legendX + 20, legendY + 58, "CL Poles");
    }

    void PidRootLocusWidget::DrawStabilityRegion(QPainter& painter, const QRect& plotArea)
    {
        // Shade the left-half plane (stable region)
        float zeroXRatio = (0.0f - viewRealMin) / (viewRealMax - viewRealMin);
        int zeroX = plotArea.left() + static_cast<int>(zeroXRatio * plotArea.width());

        if (zeroX > plotArea.left() && zeroX < plotArea.right())
        {
            QRect stableRegion(plotArea.left(), plotArea.top(), zeroX - plotArea.left(), plotArea.height());
            painter.fillRect(stableRegion, QColor(200, 255, 200, 40));

            QRect unstableRegion(zeroX, plotArea.top(), plotArea.right() - zeroX, plotArea.height());
            painter.fillRect(unstableRegion, QColor(255, 200, 200, 40));
        }
    }

    void PidRootLocusWidget::DrawAxes(QPainter& painter, const QRect& plotArea)
    {
        QPen axisPen(Qt::darkGray, 1);
        painter.setPen(axisPen);

        // Real axis
        float yRatio = (0.0f - viewImagMin) / (viewImagMax - viewImagMin);
        int yZero = plotArea.bottom() - static_cast<int>(yRatio * plotArea.height());
        if (yZero >= plotArea.top() && yZero <= plotArea.bottom())
            painter.drawLine(plotArea.left(), yZero, plotArea.right(), yZero);

        // Imaginary axis
        float xRatio = (0.0f - viewRealMin) / (viewRealMax - viewRealMin);
        int xZero = plotArea.left() + static_cast<int>(xRatio * plotArea.width());
        if (xZero >= plotArea.left() && xZero <= plotArea.right())
            painter.drawLine(xZero, plotArea.top(), xZero, plotArea.bottom());

        // Border
        painter.drawRect(plotArea);
    }

    void PidRootLocusWidget::DrawGridLines(QPainter& painter, const QRect& plotArea)
    {
        QPen gridPen(QColor(220, 220, 220), 1, Qt::DotLine);
        painter.setPen(gridPen);

        constexpr int gridLines = 10;
        for (int i = 1; i < gridLines; ++i)
        {
            int x = plotArea.left() + (i * plotArea.width() / gridLines);
            painter.drawLine(x, plotArea.top(), x, plotArea.bottom());

            int y = plotArea.bottom() - (i * plotArea.height() / gridLines);
            painter.drawLine(plotArea.left(), y, plotArea.right(), y);
        }
    }

    void PidRootLocusWidget::DrawLoci(QPainter& painter, const QRect& plotArea)
    {
        // Color palette for different branches
        QColor colors[] = {
            QColor(41, 128, 185),
            QColor(231, 76, 60),
            QColor(39, 174, 96),
            QColor(142, 68, 173),
            QColor(243, 156, 18)
        };

        for (std::size_t branch = 0; branch < loci.size(); ++branch)
        {
            QPen pen(colors[branch % 5], 2);
            painter.setPen(pen);

            const auto& locus = loci[branch];
            QPointF prev;
            bool hasPrev = false;

            for (const auto& point : locus)
            {
                QPointF pixel = ComplexToPixel(point, plotArea);

                if (pixel.x() < plotArea.left() || pixel.x() > plotArea.right() || pixel.y() < plotArea.top() || pixel.y() > plotArea.bottom())
                {
                    hasPrev = false;
                    continue;
                }

                if (hasPrev)
                {
                    // Skip very large jumps (branch discontinuities)
                    float dist = std::sqrt(std::pow(pixel.x() - prev.x(), 2) + std::pow(pixel.y() - prev.y(), 2));
                    if (dist < plotArea.width() / 2.0f)
                        painter.drawLine(prev, pixel);
                }

                prev = pixel;
                hasPrev = true;
            }
        }
    }

    void PidRootLocusWidget::DrawPoles(QPainter& painter, const QRect& plotArea)
    {
        QPen polePen(Qt::red, 2);
        painter.setPen(polePen);

        constexpr int markerSize = 6;
        for (const auto& pole : openLoopPoles)
        {
            QPointF p = ComplexToPixel(pole, plotArea);
            // Draw X
            painter.drawLine(QPointF(p.x() - markerSize, p.y() - markerSize), QPointF(p.x() + markerSize, p.y() + markerSize));
            painter.drawLine(QPointF(p.x() - markerSize, p.y() + markerSize), QPointF(p.x() + markerSize, p.y() - markerSize));
        }
    }

    void PidRootLocusWidget::DrawZeros(QPainter& painter, const QRect& plotArea)
    {
        QPen zeroPen(QColor(39, 174, 96), 2);
        painter.setPen(zeroPen);

        constexpr int markerSize = 6;
        for (const auto& zero : openLoopZeros)
        {
            QPointF p = ComplexToPixel(zero, plotArea);
            painter.drawEllipse(p, markerSize, markerSize);
        }
    }

    void PidRootLocusWidget::DrawClosedLoopPoles(QPainter& painter, const QRect& plotArea)
    {
        QPen clPen(QColor(142, 68, 173), 2);
        painter.setPen(clPen);
        painter.setBrush(QColor(142, 68, 173));

        constexpr int markerSize = 5;
        for (const auto& pole : closedLoopPoles)
        {
            QPointF p = ComplexToPixel(pole, plotArea);
            // Draw filled diamond
            QPolygonF diamond;
            diamond << QPointF(p.x(), p.y() - markerSize)
                    << QPointF(p.x() + markerSize, p.y())
                    << QPointF(p.x(), p.y() + markerSize)
                    << QPointF(p.x() - markerSize, p.y());
            painter.drawPolygon(diamond);
        }

        painter.setBrush(Qt::NoBrush);
    }

    void PidRootLocusWidget::mousePressEvent(QMouseEvent* event)
    {
        if (event->button() != Qt::LeftButton)
            return;

        QRect plotArea = GetPlotArea();
        QPointF pos = event->position();

        // Check if click is near a closed-loop pole
        constexpr float hitRadius = 15.0f;
        for (std::size_t i = 0; i < closedLoopPoles.size(); ++i)
        {
            QPointF polePixel = ComplexToPixel(closedLoopPoles[i], plotArea);
            float dist = std::sqrt(std::pow(pos.x() - polePixel.x(), 2) + std::pow(pos.y() - polePixel.y(), 2));
            if (dist < hitRadius)
            {
                isDragging = true;
                draggedPoleIndex = static_cast<int>(i);
                setCursor(Qt::ClosedHandCursor);
                return;
            }
        }
    }

    void PidRootLocusWidget::mouseMoveEvent(QMouseEvent* event)
    {
        if (!isDragging)
            return;

        QRect plotArea = GetPlotArea();
        std::complex<float> mouseComplex = PixelToComplex(event->position(), plotArea);
        float newGain = FindNearestGain(mouseComplex);

        if (newGain > 0.0f && newGain != currentGain)
            emit PoleGainChanged(newGain);
    }

    void PidRootLocusWidget::mouseReleaseEvent(QMouseEvent* event)
    {
        Q_UNUSED(event);
        if (isDragging)
        {
            isDragging = false;
            draggedPoleIndex = -1;
            setCursor(Qt::CrossCursor);
        }
    }
}
