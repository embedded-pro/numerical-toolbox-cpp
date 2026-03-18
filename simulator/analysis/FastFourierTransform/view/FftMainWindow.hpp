#pragma once

#include "simulator/analysis/FastFourierTransform/application/FftSimulator.hpp"
#include "simulator/analysis/FastFourierTransform/view/FftChartWidget.hpp"
#include "simulator/analysis/FastFourierTransform/view/FftConfigurationPanel.hpp"
#include <QMainWindow>
#include <QStatusBar>

namespace simulator::analysis::view
{
    class FftMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit FftMainWindow(QWidget* parent = nullptr);

    private:
        void OnComputeRequested();

        FftSimulator fftSimulator;
        FftConfigurationPanel* configPanel;
        FftChartWidget* chartWidget;
    };
}
