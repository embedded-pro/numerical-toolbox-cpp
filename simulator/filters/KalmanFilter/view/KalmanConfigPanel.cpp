#include "simulator/filters/KalmanFilter/view/KalmanConfigPanel.hpp"
#include <QFormLayout>
#include <QLabel>

namespace simulator::filters::view
{
    KalmanConfigPanel::KalmanConfigPanel(QWidget* parent)
        : QWidget(parent)
    {
        auto* mainLayout = new QVBoxLayout(this);

        // Simulation group
        auto* simGroup = new QGroupBox("Simulation", this);
        auto* simLayout = new QFormLayout(simGroup);

        durationSpin = CreateSpinBox(1.0, 60.0, 10.0, 1.0, 1);
        dtSpin = CreateSpinBox(0.001, 0.1, 0.01, 0.001, 3);
        thetaSpin = CreateSpinBox(-3.14, 3.14, 0.5, 0.1, 2);
        thetaDotSpin = CreateSpinBox(-10.0, 10.0, 0.0, 0.1, 2);

        simLayout->addRow("Duration (s):", durationSpin);
        simLayout->addRow("Time Step (s):", dtSpin);
        simLayout->addRow("Initial θ (rad):", thetaSpin);
        simLayout->addRow("Initial θ̇ (rad/s):", thetaDotSpin);

        mainLayout->addWidget(simGroup);

        // Noise group
        auto* noiseGroup = new QGroupBox("Noise", this);
        auto* noiseLayout = new QFormLayout(noiseGroup);

        measNoiseSpin = CreateSpinBox(0.0, 1.0, 0.1, 0.01, 3);
        procNoiseSpin = CreateSpinBox(0.0, 1.0, 0.01, 0.001, 3);

        noiseLayout->addRow("Measurement σ:", measNoiseSpin);
        noiseLayout->addRow("Process σ:", procNoiseSpin);

        mainLayout->addWidget(noiseGroup);

        // Pendulum group
        auto* pendGroup = new QGroupBox("Pendulum", this);
        auto* pendLayout = new QFormLayout(pendGroup);

        lengthSpin = CreateSpinBox(0.1, 10.0, 1.0, 0.1, 2);
        massSpin = CreateSpinBox(0.1, 10.0, 1.0, 0.1, 2);
        dampingSpin = CreateSpinBox(0.0, 5.0, 0.1, 0.01, 3);

        pendLayout->addRow("Length (m):", lengthSpin);
        pendLayout->addRow("Mass (kg):", massSpin);
        pendLayout->addRow("Damping:", dampingSpin);

        mainLayout->addWidget(pendGroup);

        // Compute button
        auto* computeBtn = new QPushButton("Compute", this);
        computeBtn->setMinimumHeight(40);
        connect(computeBtn, &QPushButton::clicked, this, &KalmanConfigPanel::ComputeRequested);
        mainLayout->addWidget(computeBtn);

        mainLayout->addStretch();
    }

    SimulationConfig KalmanConfigPanel::GetConfig() const
    {
        SimulationConfig config;
        config.duration = static_cast<float>(durationSpin->value());
        config.dt = static_cast<float>(dtSpin->value());
        config.initialTheta = static_cast<float>(thetaSpin->value());
        config.initialThetaDot = static_cast<float>(thetaDotSpin->value());
        config.measurementNoiseStdDev = static_cast<float>(measNoiseSpin->value());
        config.processNoiseStdDev = static_cast<float>(procNoiseSpin->value());
        config.pendulum.length = static_cast<float>(lengthSpin->value());
        config.pendulum.mass = static_cast<float>(massSpin->value());
        config.pendulum.damping = static_cast<float>(dampingSpin->value());
        return config;
    }

    QDoubleSpinBox* KalmanConfigPanel::CreateSpinBox(double min, double max, double value, double step, int decimals)
    {
        auto* spinBox = new QDoubleSpinBox(this);
        spinBox->setRange(min, max);
        spinBox->setValue(value);
        spinBox->setSingleStep(step);
        spinBox->setDecimals(decimals);
        return spinBox;
    }
}
