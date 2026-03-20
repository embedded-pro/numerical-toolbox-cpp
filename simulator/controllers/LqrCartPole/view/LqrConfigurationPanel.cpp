#include "simulator/controllers/LqrCartPole/view/LqrConfigurationPanel.hpp"
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <numbers>

namespace simulator::controllers::lqr::view
{
    LqrConfigurationPanel::LqrConfigurationPanel(QWidget* parent)
        : QWidget(parent)
    {
        SetupUi();
    }

    void LqrConfigurationPanel::SetupUi()
    {
        auto* mainLayout = new QVBoxLayout(this);

        auto addSpinRow = [&](QVBoxLayout* layout, const QString& label, QDoubleSpinBox*& spinBox,
                              double min, double max, double value, int decimals, const QString& suffix = "")
        {
            auto* row = new QHBoxLayout();
            row->addWidget(new QLabel(label, this));
            spinBox = new QDoubleSpinBox(this);
            spinBox->setRange(min, max);
            spinBox->setValue(value);
            spinBox->setDecimals(decimals);
            if (!suffix.isEmpty())
                spinBox->setSuffix(suffix);
            row->addWidget(spinBox);
            layout->addLayout(row);
        };

        // Plant parameters group
        auto* plantGroup = new QGroupBox("Cart-Pole Plant", this);
        auto* plantLayout = new QVBoxLayout(plantGroup);
        addSpinRow(plantLayout, "Cart Mass:", cartMassSpinBox, 0.1, 50.0, 1.0, 2, " kg");
        addSpinRow(plantLayout, "Pole Mass:", poleMassSpinBox, 0.01, 10.0, 0.1, 3, " kg");
        addSpinRow(plantLayout, "Pole Length:", poleLengthSpinBox, 0.1, 5.0, 0.5, 2, " m");
        addSpinRow(plantLayout, "Gravity:", gravitySpinBox, 1.0, 20.0, 9.81, 2, " m/s²");
        addSpinRow(plantLayout, "Friction:", frictionSpinBox, 0.0, 5.0, 0.1, 2);
        addSpinRow(plantLayout, "Track Limit:", trackLimitSpinBox, 0.5, 10.0, 2.4, 1, " m");
        mainLayout->addWidget(plantGroup);

        // LQR weights group
        auto* lqrGroup = new QGroupBox("LQR Weights (Q, R)", this);
        auto* lqrLayout = new QVBoxLayout(lqrGroup);
        addSpinRow(lqrLayout, "Q(x):", qXSpinBox, 0.001, 1000.0, 1.0, 3);
        addSpinRow(lqrLayout, "Q(ẋ):", qXDotSpinBox, 0.001, 1000.0, 1.0, 3);
        addSpinRow(lqrLayout, "Q(θ):", qThetaSpinBox, 0.001, 10000.0, 100.0, 1);
        addSpinRow(lqrLayout, "Q(θ̇):", qThetaDotSpinBox, 0.001, 1000.0, 10.0, 2);
        addSpinRow(lqrLayout, "R(F):", rForceSpinBox, 0.0001, 100.0, 0.01, 4);
        mainLayout->addWidget(lqrGroup);

        // Simulation group
        auto* simGroup = new QGroupBox("Simulation", this);
        auto* simLayout = new QVBoxLayout(simGroup);
        addSpinRow(simLayout, "Time Step:", dtSpinBox, 0.001, 0.1, 0.01, 3, " s");
        addSpinRow(simLayout, "Force Limit:", forceLimitSpinBox, 1.0, 500.0, 50.0, 1, " N");
        mainLayout->addWidget(simGroup);

        // State display group
        auto* stateGroup = new QGroupBox("State", this);
        auto* stateLayout = new QVBoxLayout(stateGroup);
        stateXLabel = new QLabel("x: 0.000 m, v: 0.000 m/s", this);
        stateThetaLabel = new QLabel("θ: 0.00°, ω: 0.00°/s", this);
        stateForceLabel = new QLabel("Control: 0.00 N", this);
        QFont monoFont("Monospace");
        monoFont.setStyleHint(QFont::TypeWriter);
        monoFont.setPointSize(9);
        stateXLabel->setFont(monoFont);
        stateThetaLabel->setFont(monoFont);
        stateForceLabel->setFont(monoFont);
        stateLayout->addWidget(stateXLabel);
        stateLayout->addWidget(stateThetaLabel);
        stateLayout->addWidget(stateForceLabel);
        mainLayout->addWidget(stateGroup);

        // Buttons
        configureButton = new QPushButton("Apply Configuration", this);
        configureButton->setMinimumHeight(35);
        mainLayout->addWidget(configureButton);

        auto* controlLayout = new QHBoxLayout();
        startButton = new QPushButton("Start", this);
        startButton->setMinimumHeight(35);
        stopButton = new QPushButton("Stop", this);
        stopButton->setMinimumHeight(35);
        controlLayout->addWidget(startButton);
        controlLayout->addWidget(stopButton);
        mainLayout->addLayout(controlLayout);

        auto* actionLayout = new QHBoxLayout();
        resetButton = new QPushButton("Reset", this);
        resetButton->setMinimumHeight(35);
        disturbButton = new QPushButton("Disturb", this);
        disturbButton->setMinimumHeight(35);
        actionLayout->addWidget(resetButton);
        actionLayout->addWidget(disturbButton);
        mainLayout->addLayout(actionLayout);

        mainLayout->addStretch();

        connect(configureButton, &QPushButton::clicked, this, &LqrConfigurationPanel::ConfigureRequested);
        connect(startButton, &QPushButton::clicked, this, &LqrConfigurationPanel::StartRequested);
        connect(stopButton, &QPushButton::clicked, this, &LqrConfigurationPanel::StopRequested);
        connect(resetButton, &QPushButton::clicked, this, &LqrConfigurationPanel::ResetRequested);
        connect(disturbButton, &QPushButton::clicked, this, &LqrConfigurationPanel::DisturbRequested);
    }

    LqrCartPoleConfig LqrConfigurationPanel::GetConfiguration() const
    {
        LqrCartPoleConfig config;

        config.plantParams.cartMass = static_cast<float>(cartMassSpinBox->value());
        config.plantParams.poleMass = static_cast<float>(poleMassSpinBox->value());
        config.plantParams.poleLength = static_cast<float>(poleLengthSpinBox->value());
        config.plantParams.gravity = static_cast<float>(gravitySpinBox->value());
        config.plantParams.cartFriction = static_cast<float>(frictionSpinBox->value());
        config.plantParams.trackLimit = static_cast<float>(trackLimitSpinBox->value());

        config.qX = static_cast<float>(qXSpinBox->value());
        config.qXDot = static_cast<float>(qXDotSpinBox->value());
        config.qTheta = static_cast<float>(qThetaSpinBox->value());
        config.qThetaDot = static_cast<float>(qThetaDotSpinBox->value());
        config.rForce = static_cast<float>(rForceSpinBox->value());

        config.dt = static_cast<float>(dtSpinBox->value());
        config.forceLimit = static_cast<float>(forceLimitSpinBox->value());

        return config;
    }

    void LqrConfigurationPanel::OnStateUpdated(float x, float xDot, float theta, float thetaDot, float force)
    {
        static constexpr auto radToDeg = 180.0f / std::numbers::pi_v<float>;

        auto thetaDeg = theta * radToDeg;
        auto thetaDotDeg = thetaDot * radToDeg;

        stateXLabel->setText(QString("x: %1 m, v: %2 m/s")
                                 .arg(static_cast<double>(x), 7, 'f', 3)
                                 .arg(static_cast<double>(xDot), 7, 'f', 3));
        stateThetaLabel->setText(QString("θ: %1°, ω: %2°/s")
                                     .arg(static_cast<double>(thetaDeg), 7, 'f', 2)
                                     .arg(static_cast<double>(thetaDotDeg), 7, 'f', 2));
        stateForceLabel->setText(QString("Control: %1 N")
                                     .arg(static_cast<double>(force), 7, 'f', 2));
    }
}
