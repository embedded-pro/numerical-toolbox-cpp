#pragma once

#include "simulator/analysis/PowerDensitySpectrum/application/PsdSimulator.hpp"
#include "simulator/analysis/PowerDensitySpectrum/view/PsdConfigurationPanel.hpp"
#include <QMainWindow>
#include <QStatusBar>
#include <QTabWidget>

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
    class FrequencyChartWidget;
}

namespace simulator::analysis::psd::view
{
    class PsdMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit PsdMainWindow(QWidget* parent = nullptr);

    private:
        void OnComputeRequested();

        PsdSimulator psdSimulator;
        PsdConfigurationPanel* configPanel;
        QTabWidget* tabWidget;
        widgets::TimeSeriesChartWidget* timeDomainChart;
        widgets::FrequencyChartWidget* psdChart;
    };
}
