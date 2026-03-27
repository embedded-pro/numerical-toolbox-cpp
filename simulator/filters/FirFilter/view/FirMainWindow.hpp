#pragma once

#include <QMainWindow>

namespace simulator::filters::fir::view
{
    class FirConfigurationPanel;
}

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
    class FrequencyChartWidget;
}

namespace simulator::filters::fir::view
{
    class FirMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit FirMainWindow(QWidget* parent = nullptr);

    private slots:
        void OnComputeRequested();

    private:
        FirConfigurationPanel* configPanel;
        QTabWidget* tabWidget;
        widgets::TimeSeriesChartWidget* timeDomainChart;
        widgets::FrequencyChartWidget* frequencyChart;
        widgets::TimeSeriesChartWidget* impulseChart;
    };
}
