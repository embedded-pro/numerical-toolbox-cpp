#pragma once

#include "simulator/filters/KalmanFilter/application/KalmanForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include "ui/backend/qt/QtPaintedWidget.hpp"
#include "ui/charts/ChartCore.hpp"
#include "ui/charts/LinearAxis.hpp"
#include <QMainWindow>

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

        ui::charts::LinearAxis timeAxis{ ui::charts::LinearAxis::Time() };
        ui::charts::ChartCore thetaChart{ timeAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore thetaDotChart{ timeAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore covarianceChart{ timeAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore errorChart{ timeAxis, ui::charts::ChartConfig{} };

        ui::backend::qt::QtPaintedWidget* thetaView;
        ui::backend::qt::QtPaintedWidget* thetaDotView;
        ui::backend::qt::QtPaintedWidget* covarianceView;
        ui::backend::qt::QtPaintedWidget* errorView;
    };
}
