#include "simulator/controllers/PidController/view/PidConfigurationPanel.hpp"
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

namespace simulator::controllers::view
{
    PidConfigurationPanel::PidConfigurationPanel(QWidget* parent)
        : QWidget(parent)
    {
        SetupUi();
    }

    void PidConfigurationPanel::SetupUi()
    {
        auto* mainLayout = new QVBoxLayout(this);

        auto* pidGroup = new QGroupBox("PID Tuning", this);
        auto* pidLayout = new QVBoxLayout(pidGroup);

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

        addSpinRow(pidLayout, "Kp:", kpSpinBox, 0.0, 100.0, 1.0, 3);
        addSpinRow(pidLayout, "Ki:", kiSpinBox, 0.0, 100.0, 0.1, 3);
        addSpinRow(pidLayout, "Kd:", kdSpinBox, 0.0, 100.0, 0.05, 3);

        mainLayout->addWidget(pidGroup);

        auto* plantGroup = new QGroupBox("Plant Model", this);
        auto* plantLayout = new QVBoxLayout(plantGroup);

        auto* orderRow = new QHBoxLayout();
        orderRow->addWidget(new QLabel("Order:", this));
        plantOrderCombo = new QComboBox(this);
        plantOrderCombo->addItem("1st Order");
        plantOrderCombo->addItem("2nd Order");
        orderRow->addWidget(plantOrderCombo);
        plantLayout->addLayout(orderRow);

        firstOrderGroup = new QGroupBox("K / (τs + 1)", this);
        auto* firstOrderLayout = new QVBoxLayout(firstOrderGroup);
        addSpinRow(firstOrderLayout, "Gain (K):", gainSpinBox, 0.01, 1000.0, 1.0, 3);
        addSpinRow(firstOrderLayout, "Time Const (τ):", timeConstantSpinBox, 0.01, 100.0, 1.0, 3, " s");
        firstOrderGroup->setVisible(false);
        plantLayout->addWidget(firstOrderGroup);

        secondOrderGroup = new QGroupBox("ωn² / (s² + 2ζωns + ωn²)", this);
        auto* secondOrderLayout = new QVBoxLayout(secondOrderGroup);
        addSpinRow(secondOrderLayout, "Natural Freq (ωn):", naturalFreqSpinBox, 0.01, 100.0, 1.0, 2, " rad/s");
        addSpinRow(secondOrderLayout, "Damping Ratio (ζ):", dampingRatioSpinBox, 0.0, 5.0, 0.5, 3);
        plantLayout->addWidget(secondOrderGroup);

        plantOrderCombo->setCurrentIndex(1);

        mainLayout->addWidget(plantGroup);

        auto* simGroup = new QGroupBox("Simulation", this);
        auto* simLayout = new QVBoxLayout(simGroup);

        addSpinRow(simLayout, "Duration:", durationSpinBox, 0.1, 1000.0, 20.0, 1, " s");
        addSpinRow(simLayout, "Sample Time:", sampleTimeSpinBox, 0.0001, 1.0, 0.01, 4, " s");
        addSpinRow(simLayout, "Output Min:", outputMinSpinBox, -10000.0, 0.0, -100.0, 1);
        addSpinRow(simLayout, "Output Max:", outputMaxSpinBox, 0.0, 10000.0, 100.0, 1);

        mainLayout->addWidget(simGroup);

        computeButton = new QPushButton("Compute", this);
        computeButton->setMinimumHeight(40);
        mainLayout->addWidget(computeButton);

        mainLayout->addStretch();

        connect(plantOrderCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &PidConfigurationPanel::OnPlantOrderChanged);
        connect(computeButton, &QPushButton::clicked, this, &PidConfigurationPanel::ComputeRequested);
    }

    PidSimulator::Configuration PidConfigurationPanel::GetConfiguration() const
    {
        PidSimulator::Configuration config;

        config.tunings.kp = static_cast<float>(kpSpinBox->value());
        config.tunings.ki = static_cast<float>(kiSpinBox->value());
        config.tunings.kd = static_cast<float>(kdSpinBox->value());

        config.limits.min = static_cast<float>(outputMinSpinBox->value());
        config.limits.max = static_cast<float>(outputMaxSpinBox->value());

        config.simulation.duration = static_cast<float>(durationSpinBox->value());
        config.simulation.sampleTime = static_cast<float>(sampleTimeSpinBox->value());

        return config;
    }

    std::unique_ptr<Plant> PidConfigurationPanel::CreatePlant() const
    {
        if (plantOrderCombo->currentIndex() == 0)
            return std::make_unique<FirstOrderPlant>(
                static_cast<float>(gainSpinBox->value()),
                static_cast<float>(timeConstantSpinBox->value()));

        return std::make_unique<SecondOrderPlant>(
            static_cast<float>(naturalFreqSpinBox->value()),
            static_cast<float>(dampingRatioSpinBox->value()));
    }

    QString PidConfigurationPanel::GetPlantDescription() const
    {
        return plantOrderCombo->currentText();
    }

    void PidConfigurationPanel::SetKp(float kp)
    {
        kpSpinBox->setValue(static_cast<double>(kp));
    }

    void PidConfigurationPanel::OnPlantOrderChanged(int index)
    {
        firstOrderGroup->setVisible(index == 0);
        secondOrderGroup->setVisible(index == 1);
    }
}
