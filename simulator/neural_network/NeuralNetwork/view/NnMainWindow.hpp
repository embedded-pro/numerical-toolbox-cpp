#pragma once

#include <QMainWindow>

namespace simulator::neural_network::nn::view
{
    class NnConfigurationPanel;
}

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
}

namespace simulator::neural_network::nn::view
{
    class NnMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit NnMainWindow(QWidget* parent = nullptr);

    private slots:
        void OnComputeRequested();

    private:
        NnConfigurationPanel* configPanel;
        QTabWidget* tabWidget;
        widgets::TimeSeriesChartWidget* lossChart;
        widgets::TimeSeriesChartWidget* predictionChart;
    };
}
