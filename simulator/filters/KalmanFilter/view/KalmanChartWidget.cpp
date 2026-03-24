#include "simulator/filters/KalmanFilter/view/KalmanChartWidget.hpp"
#include <QPainter>
#include <algorithm>
#include <cmath>
#include <limits>

namespace simulator::filters::view
{
    KalmanChartWidget::KalmanChartWidget(const QString& title, QWidget* parent)
        : QWidget(parent)
        , chartTitle(title)
    {
        setMinimumSize(400, 300);
    }

    void KalmanChartWidget::SetData(const SimulationResult& result)
    {
        data = result;

        if (!data.time.empty())
            maxTime = data.time.back();

        minValue = std::numeric_limits<float>::max();
        maxValue = std::numeric_limits<float>::lowest();

        auto updateRange = [this](const std::vector<float>& values)
        {
            for (float v : values)
            {
                minValue = std::min(minValue, v);
                maxValue = std::max(maxValue, v);
            }
        };

        if (chartTitle.contains("Angle"))
        {
            updateRange(data.trueTheta);
            updateRange(data.measuredTheta);
            updateRange(data.kf.theta);
            updateRange(data.ekf.theta);
            updateRange(data.ukf.theta);
        }
        else if (chartTitle.contains("Velocity"))
        {
            updateRange(data.trueThetaDot);
            updateRange(data.kf.thetaDot);
            updateRange(data.ekf.thetaDot);
            updateRange(data.ukf.thetaDot);
        }
        else if (chartTitle.contains("Covariance"))
        {
            updateRange(data.kf.covarianceTheta);
            updateRange(data.ekf.covarianceTheta);
            updateRange(data.ukf.covarianceTheta);
        }
        else if (chartTitle.contains("Error"))
        {
            kfError.clear();
            ekfError.clear();
            ukfError.clear();
            kfError.reserve(data.trueTheta.size());
            ekfError.reserve(data.trueTheta.size());
            ukfError.reserve(data.trueTheta.size());

            for (std::size_t i = 0; i < data.trueTheta.size(); ++i)
            {
                kfError.push_back(std::abs(data.kf.theta[i] - data.trueTheta[i]));
                ekfError.push_back(std::abs(data.ekf.theta[i] - data.trueTheta[i]));
                ukfError.push_back(std::abs(data.ukf.theta[i] - data.trueTheta[i]));
            }

            updateRange(kfError);
            updateRange(ekfError);
            updateRange(ukfError);
        }

        float margin = (maxValue - minValue) * 0.1f;
        if (margin < 0.01f)
            margin = 0.1f;
        minValue -= margin;
        maxValue += margin;

        update();
    }

    void KalmanChartWidget::paintEvent(QPaintEvent* /*event*/)
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), Qt::white);

        QRect plotArea(60, 30, width() - 80, height() - 70);

        DrawAxes(painter, plotArea);
        DrawGridLines(painter, plotArea);

        if (data.time.empty())
            return;

        if (chartTitle.contains("Angle"))
        {
            DrawTimeSeries(painter, plotArea, data.time, data.measuredTheta, QColor(200, 200, 200), 1);
            DrawTimeSeries(painter, plotArea, data.time, data.trueTheta, Qt::black, 2);
            DrawTimeSeries(painter, plotArea, data.time, data.kf.theta, Qt::blue, 2);
            DrawTimeSeries(painter, plotArea, data.time, data.ekf.theta, Qt::red, 2);
            DrawTimeSeries(painter, plotArea, data.time, data.ukf.theta, QColor(0, 150, 0), 2);
        }
        else if (chartTitle.contains("Velocity"))
        {
            DrawTimeSeries(painter, plotArea, data.time, data.trueThetaDot, Qt::black, 2);
            DrawTimeSeries(painter, plotArea, data.time, data.kf.thetaDot, Qt::blue, 2);
            DrawTimeSeries(painter, plotArea, data.time, data.ekf.thetaDot, Qt::red, 2);
            DrawTimeSeries(painter, plotArea, data.time, data.ukf.thetaDot, QColor(0, 150, 0), 2);
        }
        else if (chartTitle.contains("Covariance"))
        {
            DrawTimeSeries(painter, plotArea, data.time, data.kf.covarianceTheta, Qt::blue, 2);
            DrawTimeSeries(painter, plotArea, data.time, data.ekf.covarianceTheta, Qt::red, 2);
            DrawTimeSeries(painter, plotArea, data.time, data.ukf.covarianceTheta, QColor(0, 150, 0), 2);
        }
        else if (chartTitle.contains("Error"))
        {
            DrawTimeSeries(painter, plotArea, data.time, kfError, Qt::blue, 2);
            DrawTimeSeries(painter, plotArea, data.time, ekfError, Qt::red, 2);
            DrawTimeSeries(painter, plotArea, data.time, ukfError, QColor(0, 150, 0), 2);
        }

        DrawLegend(painter, plotArea);
    }

    void KalmanChartWidget::DrawAxes(QPainter& painter, const QRect& plotArea)
    {
        painter.setPen(QPen(Qt::black, 1));
        painter.drawRect(plotArea);

        painter.setFont(QFont("Sans", 10, QFont::Bold));
        painter.drawText(plotArea, Qt::AlignHCenter | Qt::AlignTop, chartTitle);

        painter.setFont(QFont("Sans", 8));

        // Y-axis labels
        for (int i = 0; i <= 4; ++i)
        {
            float val = minValue + (maxValue - minValue) * static_cast<float>(i) / 4.0f;
            int y = plotArea.bottom() - static_cast<int>((val - minValue) / (maxValue - minValue) * plotArea.height());
            painter.drawText(5, y + 4, QString::number(static_cast<double>(val), 'f', 2));
        }

        // X-axis labels
        for (int i = 0; i <= 5; ++i)
        {
            float t = maxTime * static_cast<float>(i) / 5.0f;
            int x = plotArea.left() + static_cast<int>(t / maxTime * plotArea.width());
            painter.drawText(x - 15, plotArea.bottom() + 15, QString::number(static_cast<double>(t), 'f', 1) + "s");
        }
    }

    void KalmanChartWidget::DrawGridLines(QPainter& painter, const QRect& plotArea)
    {
        painter.setPen(QPen(QColor(230, 230, 230), 1, Qt::DashLine));

        for (int i = 1; i <= 4; ++i)
        {
            int y = plotArea.top() + plotArea.height() * i / 5;
            painter.drawLine(plotArea.left(), y, plotArea.right(), y);
        }

        for (int i = 1; i <= 4; ++i)
        {
            int x = plotArea.left() + plotArea.width() * i / 5;
            painter.drawLine(x, plotArea.top(), x, plotArea.bottom());
        }
    }

    void KalmanChartWidget::DrawLegend(QPainter& painter, const QRect& plotArea)
    {
        painter.setFont(QFont("Sans", 8));

        struct LegendEntry
        {
            const char* label;
            QColor color;
        };

        std::vector<LegendEntry> entries;

        if (chartTitle.contains("Angle"))
        {
            entries = { { "Measurement", QColor(200, 200, 200) },
                { "True", Qt::black },
                { "KF", Qt::blue },
                { "EKF", Qt::red },
                { "UKF", QColor(0, 150, 0) } };
        }
        else if (chartTitle.contains("Velocity"))
        {
            entries = { { "True", Qt::black },
                { "KF", Qt::blue },
                { "EKF", Qt::red },
                { "UKF", QColor(0, 150, 0) } };
        }
        else
        {
            entries = { { "KF", Qt::blue },
                { "EKF", Qt::red },
                { "UKF", QColor(0, 150, 0) } };
        }

        int legendX = plotArea.right() - 120;
        int legendY = plotArea.top() + 20;

        painter.fillRect(legendX - 5, legendY - 5,
            125, static_cast<int>(entries.size()) * 18 + 10,
            QColor(255, 255, 255, 220));

        for (std::size_t i = 0; i < entries.size(); ++i)
        {
            int y = legendY + static_cast<int>(i) * 18;
            painter.setPen(QPen(entries[i].color, 2));
            painter.drawLine(legendX, y + 6, legendX + 20, y + 6);
            painter.setPen(Qt::black);
            painter.drawText(legendX + 25, y + 10, entries[i].label);
        }
    }

    void KalmanChartWidget::DrawTimeSeries(QPainter& painter, const QRect& plotArea,
        const std::vector<float>& time,
        const std::vector<float>& values,
        const QColor& color, int lineWidth)
    {
        if (time.size() < 2 || values.size() < 2)
            return;

        painter.setPen(QPen(color, lineWidth));

        float yRange = maxValue - minValue;
        if (yRange < 1e-6f)
            yRange = 1.0f;

        // Downsample for rendering if too many points
        std::size_t step = std::max(std::size_t(1), time.size() / static_cast<std::size_t>(plotArea.width()));

        QPoint prev;
        bool first = true;

        for (std::size_t i = 0; i < time.size(); i += step)
        {
            int x = plotArea.left() + static_cast<int>((time[i] / maxTime) * plotArea.width());
            int y = plotArea.bottom() - static_cast<int>(((values[i] - minValue) / yRange) * plotArea.height());

            QPoint current(x, y);
            if (!first)
                painter.drawLine(prev, current);

            prev = current;
            first = false;
        }
    }
}
