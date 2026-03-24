#pragma once

#include "simulator/controllers/LqrCartPole/application/LqrCartPoleSimulator.hpp"
#include "simulator/utils/TimeSeriesChartWidget.hpp"
#include <QLabel>
#include <QWidget>
#include <vector>

namespace simulator::controllers::lqr::view
{
    class LqrEvaluationWidget
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit LqrEvaluationWidget(QWidget* parent = nullptr);

        void SetConfig(const LqrCartPoleConfig& config);
        void Clear();

    public slots:
        void OnStateUpdated(float x, float xDot, float theta, float thetaDot, float force);

    private:
        void UpdateMetrics();
        void UpdateCharts();

        LqrCartPoleConfig config;

        std::vector<float> time;
        std::vector<float> xHistory;
        std::vector<float> xDotHistory;
        std::vector<float> thetaHistory;
        std::vector<float> thetaDotHistory;
        std::vector<float> thetaDegHistory;
        std::vector<float> thetaDotDegHistory;
        std::vector<float> forceHistory;
        std::vector<float> costHistory;

        float cumulativeCost = 0.0f;
        float peakTheta = 0.0f;
        float peakX = 0.0f;
        float totalEffort = 0.0f;
        float settlingTime = 0.0f;
        int sampleCount = 0;

        QLabel* costLabel;
        QLabel* settlingTimeLabel;
        QLabel* peakThetaLabel;
        QLabel* peakXLabel;
        QLabel* totalEffortLabel;

        utils::TimeSeriesChartWidget* stateChart;
        utils::TimeSeriesChartWidget* controlChart;

        static constexpr int updateInterval = 10;
        static constexpr float settlingThreshold = 0.02f;
    };
}
