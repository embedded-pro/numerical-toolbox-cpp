#pragma once

#include "simulator/controllers/LqrCartPole/application/LqrCartPoleSimulator.hpp"
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QWidget>

namespace simulator::controllers::lqr::view
{
    class LqrConfigurationPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit LqrConfigurationPanel(QWidget* parent = nullptr);

        LqrCartPoleConfig GetConfiguration() const;

    signals:
        void ConfigureRequested();
        void StartRequested();
        void StopRequested();
        void ResetRequested();
        void DisturbRequested();

    public Q_SLOTS:
        void OnStateUpdated(float x, float xDot, float theta, float thetaDot, float force);

    private:
        void SetupUi();

        // Plant parameters
        QDoubleSpinBox* cartMassSpinBox;
        QDoubleSpinBox* poleMassSpinBox;
        QDoubleSpinBox* poleLengthSpinBox;
        QDoubleSpinBox* gravitySpinBox;
        QDoubleSpinBox* frictionSpinBox;
        QDoubleSpinBox* trackLimitSpinBox;

        // LQR weights
        QDoubleSpinBox* qXSpinBox;
        QDoubleSpinBox* qXDotSpinBox;
        QDoubleSpinBox* qThetaSpinBox;
        QDoubleSpinBox* qThetaDotSpinBox;
        QDoubleSpinBox* rForceSpinBox;

        // Simulation
        QDoubleSpinBox* dtSpinBox;
        QDoubleSpinBox* forceLimitSpinBox;

        // State display
        QLabel* stateXLabel;
        QLabel* stateThetaLabel;
        QLabel* stateForceLabel;

        // Buttons
        QPushButton* configureButton;
        QPushButton* startButton;
        QPushButton* stopButton;
        QPushButton* resetButton;
        QPushButton* disturbButton;
    };
}
