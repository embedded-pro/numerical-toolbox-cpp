#pragma once

#include "simulator/controllers/BayesianMpcCalibration/application/BayesianMpcForm.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
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

        widgets::TimeSeriesChartWidget* observationsChart;
        widgets::TimeSeriesChartWidget* emConvergenceChart;
        widgets::TimeSeriesChartWidget* boConvergenceChart;
        widgets::TimeSeriesChartWidget* stepResponseChart;
    };
}
