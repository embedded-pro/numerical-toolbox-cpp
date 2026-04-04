#pragma once

#include <QMainWindow>
#include <QTabWidget>

namespace simulator::filters::iir::view
{
    class IirConfigurationPanel;
}

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
    class FrequencyChartWidget;
}

namespace simulator::filters::iir::view
{
    class IirMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit IirMainWindow(QWidget* parent = nullptr);

    private slots:
        void OnComputeRequested();

    private:
        IirConfigurationPanel* configPanel;
        QTabWidget* tabWidget;
        widgets::TimeSeriesChartWidget* timeDomainChart;
        widgets::FrequencyChartWidget* frequencyChart;
        widgets::TimeSeriesChartWidget* impulseChart;
    };
}
