#pragma once

#include "simulator/controllers/PidController/application/PidSimulator.hpp"
#include "simulator/controllers/PidController/view/PidConfigurationPanel.hpp"
#include <QMainWindow>
#include <QStatusBar>
#include <QTabWidget>
#include <QTimer>

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
    class FrequencyChartWidget;
}

namespace simulator::controllers::view
{
    class PidRootLocusWidget;

    class PidMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit PidMainWindow(QWidget* parent = nullptr);

    private:
        void OnComputeRequested();
        void OnPoleGainChanged(float gain);
        void RecomputeAndUpdateCharts();

        void DisplayTimeResponse(widgets::TimeSeriesChartWidget* chart, const TimeResponse& result);
        void DisplayBodeResponse(widgets::FrequencyChartWidget* chart, const BodeResult& result);

        PidSimulator pidSimulator;
        PidConfigurationPanel* configPanel;
        QTabWidget* tabWidget;
        widgets::TimeSeriesChartWidget* stepChart;
        widgets::TimeSeriesChartWidget* rampChart;
        widgets::FrequencyChartWidget* bodeChart;
        PidRootLocusWidget* rootLocusChart;

        QTimer* dragTimer;
        float pendingGain = 0.0f;
    };
}
