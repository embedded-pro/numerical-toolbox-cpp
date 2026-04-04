#pragma once

#include "simulator/filters/KalmanFilter/application/KalmanFilterSimulator.hpp"
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

namespace simulator::filters::view
{
    class KalmanConfigPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit KalmanConfigPanel(QWidget* parent = nullptr);

        SimulationConfig GetConfig() const;

    signals:
        void ComputeRequested();

    private:
        QDoubleSpinBox* CreateSpinBox(double min, double max, double value, double step, int decimals);

        QDoubleSpinBox* durationSpin;
        QDoubleSpinBox* dtSpin;
        QDoubleSpinBox* thetaSpin;
        QDoubleSpinBox* thetaDotSpin;
        QDoubleSpinBox* measNoiseSpin;
        QDoubleSpinBox* procNoiseSpin;
        QDoubleSpinBox* lengthSpin;
        QDoubleSpinBox* massSpin;
        QDoubleSpinBox* dampingSpin;
    };
}
