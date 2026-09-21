#pragma once

#include "simulator/estimators/RecursiveLeastSquares/application/RlsForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include <QMainWindow>

namespace simulator::widgets
{
    class TimeSeriesChartWidget;
}

namespace simulator::estimators::rls::view
{
    class RlsMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit RlsMainWindow(QWidget* parent = nullptr);

    private:
        void OnComputeRequested();

        RlsForm form;
        ui::backend::qt::QtFormView* formView;
        ui::backend::qt::QtAppShell shell;

        widgets::TimeSeriesChartWidget* outputChart;
        widgets::TimeSeriesChartWidget* coefficientChart;
        widgets::TimeSeriesChartWidget* metricsChart;
    };
}
