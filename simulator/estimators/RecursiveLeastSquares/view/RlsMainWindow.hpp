#pragma once

#include "simulator/estimators/RecursiveLeastSquares/application/RlsForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include "ui/backend/qt/QtPaintedWidget.hpp"
#include "ui/charts/ChartCore.hpp"
#include "ui/charts/LinearAxis.hpp"
#include <QMainWindow>

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

        ui::charts::LinearAxis sampleAxis{ "Sample", 5, 0, "n = ", "" };
        ui::charts::ChartCore outputChart{ sampleAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore coefficientChart{ sampleAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore metricsChart{ sampleAxis, ui::charts::ChartConfig{} };

        ui::backend::qt::QtPaintedWidget* outputView;
        ui::backend::qt::QtPaintedWidget* coefficientView;
        ui::backend::qt::QtPaintedWidget* metricsView;
    };
}
