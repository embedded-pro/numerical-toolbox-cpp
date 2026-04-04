#pragma once

#include "simulator/controllers/BayesianMpcCalibration/application/BayesianMpcCalibrationSimulator.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include <QDoubleSpinBox>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>

namespace simulator::controllers::view
{
    class BayesianMpcCalibrationView : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit BayesianMpcCalibrationView(QWidget* parent = nullptr);

    private:
        void OnRunRequested();
        void SetupUi();
        void DisplayResults(const CalibrationSimulationResults& results);

        widgets::TimeSeriesChartWidget* observationsChart_;
        widgets::TimeSeriesChartWidget* emConvergenceChart_;
        widgets::TimeSeriesChartWidget* boConvergenceChart_;
        widgets::TimeSeriesChartWidget* stepResponseChart_;

        QTabWidget* tabWidget_;
        QDoubleSpinBox* dtSpinBox_;
        QDoubleSpinBox* sigmaQSpinBox_;
        QDoubleSpinBox* sigmaRSpinBox_;
        QSpinBox* emIterationsSpinBox_;
        QSpinBox* boIterationsSpinBox_;
        QPushButton* runButton_;
        QLabel* statusLabel_;
    };
}
