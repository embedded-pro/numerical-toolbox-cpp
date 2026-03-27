#pragma once

#include "simulator/analysis/FastFourierTransform/application/FftSimulator.hpp"
#include "simulator/analysis/FastFourierTransform/view/FftConfigurationPanel.hpp"
#include <QMainWindow>
#include <QStatusBar>
#include <QTabWidget>

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
    class FrequencyChartWidget;
}

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
        QTabWidget* tabWidget;
        widgets::TimeSeriesChartWidget* timeDomainChart;
        widgets::FrequencyChartWidget* frequencyChart;
    };
}
