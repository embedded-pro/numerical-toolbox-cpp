#pragma once

#include "simulator/controllers/PidController/application/PidSimulator.hpp"
#include "simulator/controllers/PidController/view/PidChartWidget.hpp"
#include "simulator/controllers/PidController/view/PidConfigurationPanel.hpp"
#include <QMainWindow>
#include <QStatusBar>
#include <QTabWidget>
#include <QTimer>

namespace simulator::controllers::view
{
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

        PidSimulator pidSimulator;
        PidConfigurationPanel* configPanel;
        QTabWidget* tabWidget;
        PidStepResponseWidget* stepChart;
        PidRampResponseWidget* rampChart;
        PidBodeWidget* bodeChart;
        PidRootLocusWidget* rootLocusChart;

        QTimer* dragTimer;
        float pendingGain = 0.0f;
    };
}
