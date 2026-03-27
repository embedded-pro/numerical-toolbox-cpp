#include "simulator/filters/IirFilter/view/IirConfigurationPanel.hpp"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace simulator::filters::iir::view
{
    IirConfigurationPanel::IirConfigurationPanel(QWidget* parent)
        : QWidget(parent)
    {
        auto* mainLayout = new QVBoxLayout(this);

        auto* filterGroup = new QGroupBox("Filter Design", this);
        auto* filterLayout = new QVBoxLayout(filterGroup);

        auto* typeLayout = new QHBoxLayout();
        typeLayout->addWidget(new QLabel("Type:", this));
        filterTypeCombo = new QComboBox(this);
        filterTypeCombo->addItem("Low-Pass", static_cast<int>(FilterType::LowPass));
        filterTypeCombo->addItem("High-Pass", static_cast<int>(FilterType::HighPass));
        filterTypeCombo->addItem("Band-Pass", static_cast<int>(FilterType::BandPass));
        typeLayout->addWidget(filterTypeCombo);
        filterLayout->addLayout(typeLayout);

        auto* cutoffLayout = new QHBoxLayout();
        cutoffLayout->addWidget(new QLabel("Cutoff (Hz):", this));
        cutoffSpinBox = new QDoubleSpinBox(this);
        cutoffSpinBox->setRange(1.0, 22050.0);
        cutoffSpinBox->setValue(1000.0);
        cutoffSpinBox->setDecimals(1);
        cutoffSpinBox->setSuffix(" Hz");
        cutoffLayout->addWidget(cutoffSpinBox);
        filterLayout->addLayout(cutoffLayout);

        auto* qualityLayout = new QHBoxLayout();
        qualityLayout->addWidget(new QLabel("Q Factor:", this));
        qualitySpinBox = new QDoubleSpinBox(this);
        qualitySpinBox->setRange(0.1, 20.0);
        qualitySpinBox->setValue(0.707);
        qualitySpinBox->setDecimals(3);
        qualitySpinBox->setSingleStep(0.1);
        qualityLayout->addWidget(qualitySpinBox);
        filterLayout->addLayout(qualityLayout);

        mainLayout->addWidget(filterGroup);

        auto* simGroup = new QGroupBox("Simulation", this);
        auto* simLayout = new QVBoxLayout(simGroup);

        auto* rateLayout = new QHBoxLayout();
        rateLayout->addWidget(new QLabel("Sample Rate (Hz):", this));
        sampleRateSpinBox = new QDoubleSpinBox(this);
        sampleRateSpinBox->setRange(100.0, 192000.0);
        sampleRateSpinBox->setValue(8000.0);
        sampleRateSpinBox->setDecimals(1);
        sampleRateSpinBox->setSuffix(" Hz");
        rateLayout->addWidget(sampleRateSpinBox);
        simLayout->addLayout(rateLayout);

        auto* durationLayout = new QHBoxLayout();
        durationLayout->addWidget(new QLabel("Duration (s):", this));
        durationSpinBox = new QDoubleSpinBox(this);
        durationSpinBox->setRange(0.001, 1.0);
        durationSpinBox->setValue(0.05);
        durationSpinBox->setDecimals(3);
        durationSpinBox->setSingleStep(0.01);
        durationSpinBox->setSuffix(" s");
        durationLayout->addWidget(durationSpinBox);
        simLayout->addLayout(durationLayout);

        mainLayout->addWidget(simGroup);

        auto* signalGroup = new QGroupBox("Signal Components", this);
        auto* signalLayout = new QVBoxLayout(signalGroup);
        signalConfig = new widgets::SignalConfigWidget(this);
        signalConfig->SetDefaultComponents({ { 200.0f, 1.0f }, { 2000.0f, 0.5f } });
        signalLayout->addWidget(signalConfig);
        mainLayout->addWidget(signalGroup);

        auto* computeButton = new QPushButton("Compute", this);
        computeButton->setMinimumHeight(40);
        mainLayout->addWidget(computeButton);

        mainLayout->addStretch();

        connect(computeButton, &QPushButton::clicked, this, &IirConfigurationPanel::ComputeRequested);
    }

    IirFilterSimulator::Configuration IirConfigurationPanel::GetConfiguration() const
    {
        IirFilterSimulator::Configuration config;

        config.filter.type = static_cast<FilterType>(filterTypeCombo->currentData().toInt());
        config.filter.cutoffHz = static_cast<float>(cutoffSpinBox->value());
        config.filter.qualityFactor = static_cast<float>(qualitySpinBox->value());
        config.filter.sampleRateHz = static_cast<float>(sampleRateSpinBox->value());

        config.simulation.sampleRateHz = config.filter.sampleRateHz;
        config.simulation.duration = static_cast<float>(durationSpinBox->value());
        config.simulation.signalComponents = signalConfig->GetComponents();

        return config;
    }
}
