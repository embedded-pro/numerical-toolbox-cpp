#include "simulator/estimators/RecursiveLeastSquares/view/RlsConfigurationPanel.hpp"
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace simulator::estimators::rls::view
{
    RlsConfigurationPanel::RlsConfigurationPanel(QWidget* parent)
        : QWidget(parent)
    {
        auto* layout = new QVBoxLayout(this);

        auto* rlsGroup = new QGroupBox("RLS Parameters", this);
        auto* rlsLayout = new QVBoxLayout(rlsGroup);

        rlsLayout->addWidget(new QLabel("Forgetting Factor (λ):"));
        forgettingFactorSpinBox = new QDoubleSpinBox(rlsGroup);
        forgettingFactorSpinBox->setRange(0.9, 1.0);
        forgettingFactorSpinBox->setDecimals(3);
        forgettingFactorSpinBox->setSingleStep(0.005);
        forgettingFactorSpinBox->setValue(0.99);
        rlsLayout->addWidget(forgettingFactorSpinBox);

        rlsLayout->addWidget(new QLabel("Initial Covariance:"));
        covarianceSpinBox = new QDoubleSpinBox(rlsGroup);
        covarianceSpinBox->setRange(1.0, 10000.0);
        covarianceSpinBox->setDecimals(0);
        covarianceSpinBox->setSingleStep(100);
        covarianceSpinBox->setValue(1000.0);
        rlsLayout->addWidget(covarianceSpinBox);

        rlsLayout->addWidget(new QLabel("Number of Samples:"));
        numSamplesSpinBox = new QSpinBox(rlsGroup);
        numSamplesSpinBox->setRange(50, 2000);
        numSamplesSpinBox->setSingleStep(50);
        numSamplesSpinBox->setValue(200);
        rlsLayout->addWidget(numSamplesSpinBox);

        rlsLayout->addWidget(new QLabel("Noise Amplitude:"));
        noiseSpinBox = new QDoubleSpinBox(rlsGroup);
        noiseSpinBox->setRange(0.0, 5.0);
        noiseSpinBox->setDecimals(2);
        noiseSpinBox->setSingleStep(0.05);
        noiseSpinBox->setValue(0.1);
        rlsLayout->addWidget(noiseSpinBox);

        layout->addWidget(rlsGroup);

        auto* coeffGroup = new QGroupBox("True Coefficients (θ₀ + θ₁·x₁ + θ₂·x₂)", this);
        auto* coeffLayout = new QVBoxLayout(coeffGroup);

        coeffLayout->addWidget(new QLabel("θ₀ (bias):"));
        theta0SpinBox = new QDoubleSpinBox(coeffGroup);
        theta0SpinBox->setRange(-10.0, 10.0);
        theta0SpinBox->setDecimals(2);
        theta0SpinBox->setSingleStep(0.1);
        theta0SpinBox->setValue(2.0);
        coeffLayout->addWidget(theta0SpinBox);

        coeffLayout->addWidget(new QLabel("θ₁:"));
        theta1SpinBox = new QDoubleSpinBox(coeffGroup);
        theta1SpinBox->setRange(-10.0, 10.0);
        theta1SpinBox->setDecimals(2);
        theta1SpinBox->setSingleStep(0.1);
        theta1SpinBox->setValue(-1.5);
        coeffLayout->addWidget(theta1SpinBox);

        coeffLayout->addWidget(new QLabel("θ₂:"));
        theta2SpinBox = new QDoubleSpinBox(coeffGroup);
        theta2SpinBox->setRange(-10.0, 10.0);
        theta2SpinBox->setDecimals(2);
        theta2SpinBox->setSingleStep(0.1);
        theta2SpinBox->setValue(0.8);
        coeffLayout->addWidget(theta2SpinBox);

        layout->addWidget(coeffGroup);

        auto* computeButton = new QPushButton("Compute", this);
        layout->addWidget(computeButton);

        layout->addStretch();

        connect(computeButton, &QPushButton::clicked, this, &RlsConfigurationPanel::ComputeRequested);
    }

    RlsSimulator::Configuration RlsConfigurationPanel::GetConfiguration() const
    {
        RlsSimulator::Configuration config;
        config.rls.forgettingFactor = static_cast<float>(forgettingFactorSpinBox->value());
        config.rls.initialCovariance = static_cast<float>(covarianceSpinBox->value());
        config.rls.numSamples = static_cast<std::size_t>(numSamplesSpinBox->value());
        config.rls.noiseAmplitude = static_cast<float>(noiseSpinBox->value());
        config.rls.trueCoefficients = {
            static_cast<float>(theta0SpinBox->value()),
            static_cast<float>(theta1SpinBox->value()),
            static_cast<float>(theta2SpinBox->value()),
        };
        return config;
    }
}
