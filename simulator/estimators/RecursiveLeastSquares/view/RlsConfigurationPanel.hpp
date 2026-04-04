#pragma once

#include "simulator/estimators/RecursiveLeastSquares/application/RlsSimulator.hpp"
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QWidget>

namespace simulator::estimators::rls::view
{
    class RlsConfigurationPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit RlsConfigurationPanel(QWidget* parent = nullptr);

        RlsSimulator::Configuration GetConfiguration() const;

    signals:
        void ComputeRequested();

    private:
        QDoubleSpinBox* forgettingFactorSpinBox;
        QDoubleSpinBox* covarianceSpinBox;
        QSpinBox* numSamplesSpinBox;
        QDoubleSpinBox* noiseSpinBox;
        QDoubleSpinBox* theta0SpinBox;
        QDoubleSpinBox* theta1SpinBox;
        QDoubleSpinBox* theta2SpinBox;
    };
}
