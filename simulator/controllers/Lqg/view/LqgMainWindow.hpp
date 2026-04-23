#pragma once

#include "simulator/controllers/Lqg/application/LqgSimulator.hpp"
#include "simulator/controllers/Lqg/view/LqgConfigurationPanel.hpp"
#include <QMainWindow>
#include <QStatusBar>

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
}

namespace simulator::controllers::lqg::view
{
    class LqgMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit LqgMainWindow(QWidget* parent = nullptr);

    private:
        void OnComputeRequested();
        void DisplayResponse(const LqgTimeResponse& result);

        LqgSimulator lqgSimulator;
        LqgConfigurationPanel* configPanel;
        widgets::TimeSeriesChartWidget* chart;
    };
}
