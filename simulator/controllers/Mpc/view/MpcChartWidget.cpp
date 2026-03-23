#include "simulator/controllers/Mpc/view/MpcChartWidget.hpp"
#include <QPainter>
#include <QPen>
#include <algorithm>
#include <cmath>

namespace simulator::controllers::view
{
    MpcTimeResponseWidget::MpcTimeResponseWidget(const QString& title, QWidget* parent)
        : QWidget(parent)
        , chartTitle(title)
    {
        setMinimumSize(400, 500);
        setBackgroundRole(QPalette::Base);
        setAutoFillBackground(true);
    }

    void MpcTimeResponseWidget::SetData(const MpcTimeResponse& result, float refPos)
    {
        time = result.time;
        states = result.states;
        control = result.control;
        cost = result.cost;
        referencePosition = refPos;

        maxTime = time.empty() ? 0.0f : time.back();
        maxState = 0.0f;
        minState = 0.0f;
        maxControl = 0.0f;
        minControl = 0.0f;
        maxCost = 0.0f;

        for (const auto& s : states)
            for (auto v : s)
            {
                maxState = std::max(maxState, v);
                minState = std::min(minState, v);
            }

        maxState = std::max(maxState, refPos);
        minState = std::min(minState, refPos);

        for (auto v : control)
        {
            maxControl = std::max(maxControl, v);
            minControl = std::min(minControl, v);
        }

        for (auto v : cost)
            maxCost = std::max(maxCost, v);

        float statePad = (maxState - minState) * 0.1f;
        if (statePad < 0.1f)
            statePad = 0.1f;
        maxState += statePad;
        minState -= statePad;

        float ctrlPad = (maxControl - minControl) * 0.1f;
        if (ctrlPad < 0.1f)
            ctrlPad = 0.1f;
        maxControl += ctrlPad;
        minControl -= ctrlPad;

        maxCost *= 1.1f;
        if (maxCost < 0.1f)
            maxCost = 0.1f;

        update();
    }

    void MpcTimeResponseWidget::paintEvent(QPaintEvent* /*event*/)
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        if (time.empty())
        {
            painter.drawText(rect(), Qt::AlignCenter, "No data - press Compute");
            return;
        }

        int margin = 50;
        int chartHeight = (height() - 4 * margin) / 3;
        if (chartHeight < 50)
            return;

        painter.setPen(QPen(Qt::black, 1));
        painter.drawText(QRect(0, 5, width(), 20), Qt::AlignCenter, chartTitle);

        QRect stateArea(margin, margin, width() - 2 * margin, chartHeight);
        QRect controlArea(margin, margin + chartHeight + margin / 2, width() - 2 * margin, chartHeight);
        QRect costArea(margin, margin + 2 * (chartHeight + margin / 2), width() - 2 * margin, chartHeight);

        DrawStateChart(painter, stateArea);
        DrawControlChart(painter, controlArea);
        DrawCostChart(painter, costArea);
    }

    void MpcTimeResponseWidget::DrawAxes(QPainter& painter, const QRect& plotArea)
    {
        painter.setPen(QPen(Qt::black, 1));
        painter.drawRect(plotArea);
    }

    void MpcTimeResponseWidget::DrawStateChart(QPainter& painter, const QRect& plotArea)
    {
        DrawAxes(painter, plotArea);
        painter.drawText(plotArea.topLeft() + QPointF(-45, plotArea.height() / 2.0), "State");

        if (time.size() < 2)
            return;

        float stateRange = maxState - minState;
        if (stateRange < 1e-6f)
            stateRange = 1.0f;

        // Draw reference line
        painter.setPen(QPen(Qt::gray, 1, Qt::DashLine));
        float refY = plotArea.bottom() - (referencePosition - minState) / stateRange * plotArea.height();
        painter.drawLine(QPointF(plotArea.left(), refY), QPointF(plotArea.right(), refY));

        // Draw states
        QColor colors[] = { Qt::blue, Qt::red, Qt::darkGreen, Qt::magenta };
        QString labels[] = { "Position", "Velocity", "State 3", "State 4" };

        for (std::size_t s = 0; s < states.size() && s < 4; ++s)
        {
            painter.setPen(QPen(colors[s], 2));
            for (std::size_t i = 1; i < time.size() && i < states[s].size(); ++i)
            {
                float x1 = plotArea.left() + time[i - 1] / maxTime * plotArea.width();
                float y1 = plotArea.bottom() - (states[s][i - 1] - minState) / stateRange * plotArea.height();
                float x2 = plotArea.left() + time[i] / maxTime * plotArea.width();
                float y2 = plotArea.bottom() - (states[s][i] - minState) / stateRange * plotArea.height();
                painter.drawLine(QPointF(x1, y1), QPointF(x2, y2));
            }

            painter.drawText(plotArea.right() - 80, plotArea.top() + 15 + static_cast<int>(s) * 15, labels[s]);
        }
    }

    void MpcTimeResponseWidget::DrawControlChart(QPainter& painter, const QRect& plotArea)
    {
        DrawAxes(painter, plotArea);
        painter.drawText(plotArea.topLeft() + QPointF(-45, plotArea.height() / 2.0), "Control");

        if (time.size() < 2)
            return;

        float ctrlRange = maxControl - minControl;
        if (ctrlRange < 1e-6f)
            ctrlRange = 1.0f;

        painter.setPen(QPen(Qt::darkCyan, 2));
        for (std::size_t i = 1; i < time.size() && i < control.size(); ++i)
        {
            float x1 = plotArea.left() + time[i - 1] / maxTime * plotArea.width();
            float y1 = plotArea.bottom() - (control[i - 1] - minControl) / ctrlRange * plotArea.height();
            float x2 = plotArea.left() + time[i] / maxTime * plotArea.width();
            float y2 = plotArea.bottom() - (control[i] - minControl) / ctrlRange * plotArea.height();
            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2));
        }
    }

    void MpcTimeResponseWidget::DrawCostChart(QPainter& painter, const QRect& plotArea)
    {
        DrawAxes(painter, plotArea);
        painter.drawText(plotArea.topLeft() + QPointF(-45, plotArea.height() / 2.0), "Cost");

        if (time.size() < 2)
            return;

        painter.setPen(QPen(Qt::darkRed, 2));
        for (std::size_t i = 1; i < time.size() && i < cost.size(); ++i)
        {
            float x1 = plotArea.left() + time[i - 1] / maxTime * plotArea.width();
            float y1 = plotArea.bottom() - cost[i - 1] / maxCost * plotArea.height();
            float x2 = plotArea.left() + time[i] / maxTime * plotArea.width();
            float y2 = plotArea.bottom() - cost[i] / maxCost * plotArea.height();
            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2));
        }
    }
}
