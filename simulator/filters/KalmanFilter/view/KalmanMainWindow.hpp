#pragma once

#include "simulator/filters/KalmanFilter/application/KalmanForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include <QMainWindow>

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
        kalman::KalmanForm form;
        ui::backend::qt::QtFormView* formView;
        ui::backend::qt::QtAppShell shell;

        widgets::TimeSeriesChartWidget* thetaChart;
        widgets::TimeSeriesChartWidget* thetaDotChart;
        widgets::TimeSeriesChartWidget* covarianceChart;
        widgets::TimeSeriesChartWidget* errorChart;
    };
}
