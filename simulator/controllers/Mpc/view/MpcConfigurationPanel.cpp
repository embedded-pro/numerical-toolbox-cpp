#include "simulator/controllers/Mpc/view/MpcConfigurationPanel.hpp"
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

namespace simulator::controllers::view
{
    MpcConfigurationPanel::MpcConfigurationPanel(QWidget* parent)
        : QWidget(parent)
    {
        SetupUi();
    }

    void MpcConfigurationPanel::SetupUi()
    {
        auto* mainLayout = new QVBoxLayout(this);

        auto addSpinRow = [&](QVBoxLayout* layout, const QString& label, QDoubleSpinBox*& spinBox, double min, double max, double value, int decimals, const QString& suffix = "")
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

        auto* weightsGroup = new QGroupBox("MPC Weights", this);
        auto* weightsLayout = new QVBoxLayout(weightsGroup);
        addSpinRow(weightsLayout, "Q (state):", stateWeightSpinBox, 0.01, 1000.0, 10.0, 2);
        addSpinRow(weightsLayout, "R (control):", controlWeightSpinBox, 0.001, 100.0, 0.1, 3);
        mainLayout->addWidget(weightsGroup);

        auto* constraintGroup = new QGroupBox("Constraints", this);
        auto* constraintLayout = new QVBoxLayout(constraintGroup);
        constraintsEnabled = new QCheckBox("Enable control constraints", this);
        constraintLayout->addWidget(constraintsEnabled);
        addSpinRow(constraintLayout, "u_min:", uMinSpinBox, -100.0, 0.0, -2.0, 2);
        addSpinRow(constraintLayout, "u_max:", uMaxSpinBox, 0.0, 100.0, 2.0, 2);
        mainLayout->addWidget(constraintGroup);

        auto* plantGroup = new QGroupBox("Plant Model", this);
        auto* plantLayout = new QVBoxLayout(plantGroup);
        auto* plantRow = new QHBoxLayout();
        plantRow->addWidget(new QLabel("Type:", this));
        plantCombo = new QComboBox(this);
        plantCombo->addItem("Double Integrator");
        plantCombo->addItem("1st Order + Integrator");
        plantRow->addWidget(plantCombo);
        plantLayout->addLayout(plantRow);

        firstOrderGroup = new QGroupBox("Plant Parameters", this);
        auto* firstOrderLayout = new QVBoxLayout(firstOrderGroup);
        addSpinRow(firstOrderLayout, "Gain:", plantGainSpinBox, 0.01, 100.0, 1.0, 2);
        addSpinRow(firstOrderLayout, "Time Const:", plantTimeConstantSpinBox, 0.01, 100.0, 1.0, 2, " s");
        firstOrderGroup->setVisible(false);
        plantLayout->addWidget(firstOrderGroup);

        mainLayout->addWidget(plantGroup);

        auto* simGroup = new QGroupBox("Simulation", this);
        auto* simLayout = new QVBoxLayout(simGroup);
        addSpinRow(simLayout, "Duration:", durationSpinBox, 0.1, 100.0, 10.0, 1, " s");
        addSpinRow(simLayout, "Sample Time:", sampleTimeSpinBox, 0.001, 1.0, 0.1, 3, " s");
        addSpinRow(simLayout, "Reference:", referenceSpinBox, -100.0, 100.0, 1.0, 2);
        mainLayout->addWidget(simGroup);

        computeButton = new QPushButton("Compute", this);
        computeButton->setMinimumHeight(40);
        mainLayout->addWidget(computeButton);

        mainLayout->addStretch();

        connect(plantCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index)
            {
                firstOrderGroup->setVisible(index == 1);
            });
        connect(computeButton, &QPushButton::clicked, this, &MpcConfigurationPanel::ComputeRequested);
    }

    MpcSimulator::Configuration MpcConfigurationPanel::GetConfiguration() const
    {
        MpcSimulator::Configuration config;

        config.weights.stateWeight = static_cast<float>(stateWeightSpinBox->value());
        config.weights.controlWeight = static_cast<float>(controlWeightSpinBox->value());

        config.constraints.enabled = constraintsEnabled->isChecked();
        config.constraints.uMin = static_cast<float>(uMinSpinBox->value());
        config.constraints.uMax = static_cast<float>(uMaxSpinBox->value());

        config.simulation.duration = static_cast<float>(durationSpinBox->value());
        config.simulation.sampleTime = static_cast<float>(sampleTimeSpinBox->value());

        config.referencePosition = static_cast<float>(referenceSpinBox->value());

        return config;
    }

    math::LinearTimeInvariant<float, 2, 1> MpcConfigurationPanel::CreatePlant() const
    {
        float dt = static_cast<float>(sampleTimeSpinBox->value());

        if (plantCombo->currentIndex() == 0)
            return MakeDoubleIntegrator(dt);

        return MakeFirstOrderWithIntegrator(
            static_cast<float>(plantGainSpinBox->value()),
            static_cast<float>(plantTimeConstantSpinBox->value()),
            dt);
    }

    QString MpcConfigurationPanel::GetPlantDescription() const
    {
        return plantCombo->currentText();
    }
}
