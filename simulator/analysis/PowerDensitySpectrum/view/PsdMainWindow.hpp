#pragma once

#include "simulator/analysis/PowerDensitySpectrum/application/PsdSimulator.hpp"
#include "simulator/analysis/PowerDensitySpectrum/view/PsdChartWidget.hpp"
#include "simulator/analysis/PowerDensitySpectrum/view/PsdConfigurationPanel.hpp"
#include <QMainWindow>
#include <QStatusBar>

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
        PsdChartWidget* chartWidget;
    };
}
