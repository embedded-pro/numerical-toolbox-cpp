#include "simulator/controllers/PidController/view/PidRootLocusWidget.hpp"
#include <QPainter>
#include <QPolygonF>
#include <algorithm>
#include <cmath>
#include <limits>

namespace simulator::controllers::view
{
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
        float rMin = std::numeric_limits<float>::max();
        float rMax = std::numeric_limits<float>::lowest();
        float iMin = std::numeric_limits<float>::max();
        float iMax = std::numeric_limits<float>::lowest();

        bool hasData = false;
        auto expandBounds = [&](std::complex<float> c)
        {
            rMin = std::min(rMin, c.real());
            rMax = std::max(rMax, c.real());
            iMin = std::min(iMin, c.imag());
            iMax = std::max(iMax, c.imag());
            hasData = true;
        };

        for (const auto& pole : openLoopPoles)
            expandBounds(pole);
        for (const auto& zero : openLoopZeros)
            expandBounds(zero);
        for (const auto& locus : loci)
            for (const auto& point : locus)
                expandBounds(point);

        if (!hasData)
        {
            viewRealMin = -10.0f;
            viewRealMax = 5.0f;
            viewImagMin = -7.0f;
            viewImagMax = 7.0f;
            update();
            return;
        }

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

        auto searchBranch = [&](std::size_t branch)
        {
            const auto& locus = loci[branch];
            for (std::size_t i = 0; i < locus.size() && i < gains.size(); ++i)
            {
                float dist = std::abs(locus[i] - point);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestGain = gains[i];
                }
            }
        };

        if (draggedPoleIndex >= 0 && static_cast<std::size_t>(draggedPoleIndex) < loci.size())
        {
            searchBranch(static_cast<std::size_t>(draggedPoleIndex));
        }
        else
        {
            for (std::size_t branch = 0; branch < loci.size(); ++branch)
                searchBranch(branch);
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
