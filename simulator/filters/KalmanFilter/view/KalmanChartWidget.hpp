#pragma once

#include "simulator/filters/KalmanFilter/application/KalmanFilterSimulator.hpp"
#include <QWidget>

namespace simulator::filters::view
{
    class KalmanChartWidget
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit KalmanChartWidget(const QString& title, QWidget* parent = nullptr);

        void SetData(const SimulationResult& result);

    protected:
        void paintEvent(QPaintEvent* event) override;

    private:
        void DrawAxes(QPainter& painter, const QRect& plotArea);
        void DrawGridLines(QPainter& painter, const QRect& plotArea);
        void DrawLegend(QPainter& painter, const QRect& plotArea);
        void DrawTimeSeries(QPainter& painter, const QRect& plotArea,
            const std::vector<float>& time,
            const std::vector<float>& values,
            const QColor& color, int width = 1);

        QString chartTitle;
        SimulationResult data;
        std::vector<float> kfError;
        std::vector<float> ekfError;
        std::vector<float> ukfError;
        float minValue = 0.0f;
        float maxValue = 0.0f;
        float maxTime = 0.0f;
    };

    class ThetaChartWidget : public KalmanChartWidget
    {
    public:
        explicit ThetaChartWidget(QWidget* parent = nullptr)
            : KalmanChartWidget("Angle (θ) Estimation", parent)
        {}
    };

    class ThetaDotChartWidget : public KalmanChartWidget
    {
    public:
        explicit ThetaDotChartWidget(QWidget* parent = nullptr)
            : KalmanChartWidget("Angular Velocity (θ̇) Estimation", parent)
        {}
    };

    class CovarianceChartWidget : public KalmanChartWidget
    {
    public:
        explicit CovarianceChartWidget(QWidget* parent = nullptr)
            : KalmanChartWidget("Position Covariance P(θ,θ)", parent)
        {}
    };

    class ErrorChartWidget : public KalmanChartWidget
    {
    public:
        explicit ErrorChartWidget(QWidget* parent = nullptr)
            : KalmanChartWidget("Estimation Error |θ̂ - θ|", parent)
        {}
    };
}
