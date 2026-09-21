#pragma once

#include "simulator/controllers/BayesianMpcCalibration/application/BayesianMpcForm.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include "ui/backend/qt/QtPaintedWidget.hpp"
#include "ui/charts/ChartCore.hpp"
#include "ui/charts/LinearAxis.hpp"
#include <QMainWindow>

namespace simulator::controllers::view
{
    class BayesianMpcCalibrationView
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit BayesianMpcCalibrationView(QWidget* parent = nullptr);

    private:
        void OnRunRequested();
        void DisplayResults(const CalibrationSimulationResults& results);

        bayesian::BayesianMpcForm form;
        ui::backend::qt::QtFormView* formView;
        ui::backend::qt::QtAppShell shell;

        ui::charts::LinearAxis timeAxis{ ui::charts::LinearAxis::Time() };
        ui::charts::LinearAxis iterationAxis{ "Iteration", 5, 0, "i = ", "" };
        ui::charts::LinearAxis evaluationAxis{ "Evaluation", 5, 0, "i = ", "" };
        ui::charts::ChartCore observationsChart{ timeAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore emConvergenceChart{ iterationAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore boConvergenceChart{ evaluationAxis, ui::charts::ChartConfig{} };
        ui::charts::ChartCore stepResponseChart{ timeAxis, ui::charts::ChartConfig{} };

        ui::backend::qt::QtPaintedWidget* observationsView;
        ui::backend::qt::QtPaintedWidget* emConvergenceView;
        ui::backend::qt::QtPaintedWidget* boConvergenceView;
        ui::backend::qt::QtPaintedWidget* stepResponseView;
    };
}
