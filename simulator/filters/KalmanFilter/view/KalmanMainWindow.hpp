#pragma once

#include "simulator/filters/KalmanFilter/application/KalmanFilterSimulator.hpp"
#include "simulator/filters/KalmanFilter/view/KalmanConfigPanel.hpp"
#include <QMainWindow>
#include <QTabWidget>

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
}

namespace simulator::filters::view
{
    class KalmanMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit KalmanMainWindow(QWidget* parent = nullptr);

    private:
        void OnComputeRequested();

        KalmanFilterSimulator simulator;
        KalmanConfigPanel* configPanel;
        QTabWidget* tabWidget;
        widgets::TimeSeriesChartWidget* thetaChart;
        widgets::TimeSeriesChartWidget* thetaDotChart;
        widgets::TimeSeriesChartWidget* covarianceChart;
        widgets::TimeSeriesChartWidget* errorChart;
    };
}
