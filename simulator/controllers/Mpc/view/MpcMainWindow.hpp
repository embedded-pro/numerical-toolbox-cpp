#pragma once

#include "simulator/controllers/Mpc/application/MpcSimulator.hpp"
#include "simulator/controllers/Mpc/view/MpcChartWidget.hpp"
#include "simulator/controllers/Mpc/view/MpcConfigurationPanel.hpp"
#include <QMainWindow>
#include <QStatusBar>
#include <QTabWidget>

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

        MpcSimulator mpcSimulator;
        MpcConfigurationPanel* configPanel;
        QTabWidget* tabWidget;
        MpcStepResponseWidget* stepChart;
        MpcConstrainedResponseWidget* constrainedChart;
    };
}
