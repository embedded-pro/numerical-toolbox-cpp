#pragma once

#include "simulator/filters/KalmanFilter/application/KalmanFilterSimulator.hpp"
#include "simulator/filters/KalmanFilter/view/KalmanChartWidget.hpp"
#include "simulator/filters/KalmanFilter/view/KalmanConfigPanel.hpp"
#include <QMainWindow>
#include <QTabWidget>

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
        ThetaChartWidget* thetaChart;
        ThetaDotChartWidget* thetaDotChart;
        CovarianceChartWidget* covarianceChart;
        ErrorChartWidget* errorChart;
    };
}
