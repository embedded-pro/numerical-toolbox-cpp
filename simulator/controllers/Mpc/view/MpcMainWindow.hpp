#pragma once

#include "simulator/controllers/Mpc/application/MpcSimulator.hpp"
#include "simulator/controllers/Mpc/view/MpcConfigurationPanel.hpp"
#include <QMainWindow>
#include <QStatusBar>
#include <QTabWidget>

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
}

namespace simulator::controllers::view
{
    class MpcMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit MpcMainWindow(QWidget* parent = nullptr);

    private:
        void OnComputeRequested();
        void DisplayResponse(widgets::TimeSeriesChartWidget* chart, const MpcTimeResponse& result, float referencePosition);

        MpcSimulator mpcSimulator;
        MpcConfigurationPanel* configPanel;
        QTabWidget* tabWidget;
        widgets::TimeSeriesChartWidget* stepChart;
        widgets::TimeSeriesChartWidget* constrainedChart;
    };
}
