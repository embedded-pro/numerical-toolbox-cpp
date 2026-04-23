#include "simulator/controllers/Lqg/view/LqgConfigurationPanel.hpp"
#include <QFormLayout>
#include <QGroupBox>
#include <QVBoxLayout>

namespace simulator::controllers::lqg::view
{
    LqgConfigurationPanel::LqgConfigurationPanel(QWidget* parent)
        : QWidget(parent)
    {
        SetupUi();
    }

    void LqgConfigurationPanel::SetupUi()
    {
        auto* mainLayout = new QVBoxLayout(this);

        auto* weightsGroup = new QGroupBox("LQR Weights", this);
        auto* weightsLayout = new QFormLayout(weightsGroup);
        stateWeightSpinBox = new QDoubleSpinBox(this);
        stateWeightSpinBox->setRange(0.01, 1000.0);
        stateWeightSpinBox->setValue(10.0);
        controlWeightSpinBox = new QDoubleSpinBox(this);
        controlWeightSpinBox->setRange(0.001, 100.0);
        controlWeightSpinBox->setSingleStep(0.01);
        controlWeightSpinBox->setValue(0.1);
        weightsLayout->addRow("State Weight Q:", stateWeightSpinBox);
        weightsLayout->addRow("Control Weight R:", controlWeightSpinBox);

        auto* noiseGroup = new QGroupBox("Kalman Noise", this);
        auto* noiseLayout = new QFormLayout(noiseGroup);
        processNoiseSpinBox = new QDoubleSpinBox(this);
        processNoiseSpinBox->setRange(0.001, 10.0);
        processNoiseSpinBox->setSingleStep(0.01);
        processNoiseSpinBox->setValue(0.1);
        measurementNoiseSpinBox = new QDoubleSpinBox(this);
        measurementNoiseSpinBox->setRange(0.001, 10.0);
        measurementNoiseSpinBox->setSingleStep(0.1);
        measurementNoiseSpinBox->setValue(1.0);
        noiseLayout->addRow("Process Noise:", processNoiseSpinBox);
        noiseLayout->addRow("Measurement Noise:", measurementNoiseSpinBox);

        auto* simGroup = new QGroupBox("Simulation", this);
        auto* simLayout = new QFormLayout(simGroup);
        sampleTimeSpinBox = new QDoubleSpinBox(this);
        sampleTimeSpinBox->setRange(0.001, 1.0);
        sampleTimeSpinBox->setSingleStep(0.01);
        sampleTimeSpinBox->setValue(0.1);
        durationSpinBox = new QDoubleSpinBox(this);
        durationSpinBox->setRange(1.0, 100.0);
        durationSpinBox->setValue(10.0);
        initialPositionSpinBox = new QDoubleSpinBox(this);
        initialPositionSpinBox->setRange(-10.0, 10.0);
        initialPositionSpinBox->setValue(1.0);
        simLayout->addRow("Sample Time (s):", sampleTimeSpinBox);
        simLayout->addRow("Duration (s):", durationSpinBox);
        simLayout->addRow("Initial Position:", initialPositionSpinBox);

        computeButton = new QPushButton("Compute", this);
        connect(computeButton, &QPushButton::clicked, this, &LqgConfigurationPanel::ComputeRequested);

        mainLayout->addWidget(weightsGroup);
        mainLayout->addWidget(noiseGroup);
        mainLayout->addWidget(simGroup);
        mainLayout->addWidget(computeButton);
        mainLayout->addStretch();
    }

    LqgSimulatorConfig LqgConfigurationPanel::GetConfiguration() const
    {
        LqgSimulatorConfig config;
        config.weights.stateWeight = static_cast<float>(stateWeightSpinBox->value());
        config.weights.controlWeight = static_cast<float>(controlWeightSpinBox->value());
        config.noise.processNoise = static_cast<float>(processNoiseSpinBox->value());
        config.noise.measurementNoise = static_cast<float>(measurementNoiseSpinBox->value());
        config.simulation.sampleTime = static_cast<float>(sampleTimeSpinBox->value());
        config.simulation.duration = static_cast<float>(durationSpinBox->value());
        config.initialPosition = static_cast<float>(initialPositionSpinBox->value());
        return config;
    }

    LqgPlant LqgConfigurationPanel::CreatePlant() const
    {
        float dt = static_cast<float>(sampleTimeSpinBox->value());
        math::SquareMatrix<float, lqgStateSize> A{
            { 1.0f, dt },
            { 0.0f, 1.0f }
        };
        math::Matrix<float, lqgStateSize, lqgInputSize> B{
            { 0.0f },
            { dt }
        };
        math::Matrix<float, lqgMeasurementSize, lqgStateSize> C{
            { 1.0f, 0.0f }
        };
        math::Matrix<float, lqgMeasurementSize, lqgInputSize> D{};
        return LqgPlant{ A, B, C, D };
    }
}
