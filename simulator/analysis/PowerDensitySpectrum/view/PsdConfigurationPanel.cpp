#include "simulator/analysis/PowerDensitySpectrum/view/PsdConfigurationPanel.hpp"
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

namespace simulator::analysis::psd::view
{
    PsdConfigurationPanel::PsdConfigurationPanel(QWidget* parent)
        : QWidget(parent)
    {
        SetupUi();
    }

    void PsdConfigurationPanel::SetupUi()
    {
        auto* mainLayout = new QVBoxLayout(this);

        auto* psdGroup = new QGroupBox("PSD Configuration", this);
        auto* psdLayout = new QVBoxLayout(psdGroup);

        auto* inputSizeLayout = new QHBoxLayout();
        inputSizeLayout->addWidget(new QLabel("Input Size:", this));
        inputSizeSpinBox = new QDoubleSpinBox(this);
        inputSizeSpinBox->setRange(64, 4096);
        inputSizeSpinBox->setValue(1024);
        inputSizeSpinBox->setDecimals(0);
        inputSizeLayout->addWidget(inputSizeSpinBox);
        psdLayout->addLayout(inputSizeLayout);

        auto* segmentLayout = new QHBoxLayout();
        segmentLayout->addWidget(new QLabel("Segment Size:", this));
        segmentSizeCombo = new QComboBox(this);
        for (auto size : PsdSimulator::SupportedSegmentSizes())
            segmentSizeCombo->addItem(QString::number(size), QVariant::fromValue(static_cast<qulonglong>(size)));

        constexpr qulonglong defaultSegmentSize = 256u;
        const int defaultIndex = segmentSizeCombo->findData(QVariant::fromValue(defaultSegmentSize));
        if (defaultIndex != -1)
            segmentSizeCombo->setCurrentIndex(defaultIndex);
        else if (segmentSizeCombo->count() > 0)
            segmentSizeCombo->setCurrentIndex(0);

        segmentLayout->addWidget(segmentSizeCombo);
        psdLayout->addLayout(segmentLayout);

        auto* overlapLayout = new QHBoxLayout();
        overlapLayout->addWidget(new QLabel("Overlap (%):", this));
        overlapCombo = new QComboBox(this);
        for (auto overlap : PsdSimulator::SupportedOverlapValues())
            overlapCombo->addItem(QString::number(overlap) + "%", QVariant::fromValue(static_cast<qulonglong>(overlap)));

        constexpr qulonglong defaultOverlap = 50u;
        const int overlapIndex = overlapCombo->findData(QVariant::fromValue(defaultOverlap));
        if (overlapIndex != -1)
            overlapCombo->setCurrentIndex(overlapIndex);

        overlapLayout->addWidget(overlapCombo);
        psdLayout->addLayout(overlapLayout);

        auto* rateLayout = new QHBoxLayout();
        rateLayout->addWidget(new QLabel("Sample Rate (Hz):", this));
        sampleRateSpinBox = new QDoubleSpinBox(this);
        sampleRateSpinBox->setRange(100.0, 192000.0);
        sampleRateSpinBox->setValue(44100.0);
        sampleRateSpinBox->setDecimals(1);
        sampleRateSpinBox->setSuffix(" Hz");
        rateLayout->addWidget(sampleRateSpinBox);
        psdLayout->addLayout(rateLayout);

        auto* windowLayout = new QHBoxLayout();
        windowLayout->addWidget(new QLabel("Window:", this));
        windowTypeCombo = new QComboBox(this);
        windowTypeCombo->addItem("Rectangular", static_cast<int>(WindowType::Rectangular));
        windowTypeCombo->addItem("Hamming", static_cast<int>(WindowType::Hamming));
        windowTypeCombo->addItem("Hanning", static_cast<int>(WindowType::Hanning));
        windowTypeCombo->addItem("Blackman", static_cast<int>(WindowType::Blackman));
        windowTypeCombo->setCurrentIndex(1);
        windowLayout->addWidget(windowTypeCombo);
        psdLayout->addLayout(windowLayout);

        mainLayout->addWidget(psdGroup);

        auto* noiseGroup = new QGroupBox("Noise Configuration", this);
        auto* noiseLayout = new QVBoxLayout(noiseGroup);

        auto* noiseTypeLayout = new QHBoxLayout();
        noiseTypeLayout->addWidget(new QLabel("Noise Type:", this));
        noiseTypeCombo = new QComboBox(this);
        noiseTypeCombo->addItem("None", static_cast<int>(NoiseType::None));
        noiseTypeCombo->addItem("White Gaussian", static_cast<int>(NoiseType::WhiteGaussian));
        noiseTypeCombo->setCurrentIndex(1);
        noiseTypeLayout->addWidget(noiseTypeCombo);
        noiseLayout->addLayout(noiseTypeLayout);

        auto* noiseAmpLayout = new QHBoxLayout();
        noiseAmpLayout->addWidget(new QLabel("Noise Amplitude:", this));
        noiseAmplitudeSpinBox = new QDoubleSpinBox(this);
        noiseAmplitudeSpinBox->setRange(0.0, 10.0);
        noiseAmplitudeSpinBox->setValue(0.1);
        noiseAmplitudeSpinBox->setDecimals(4);
        noiseAmplitudeSpinBox->setSingleStep(0.01);
        noiseAmpLayout->addWidget(noiseAmplitudeSpinBox);
        noiseLayout->addLayout(noiseAmpLayout);

        connect(noiseTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int)
            {
                auto type = static_cast<NoiseType>(noiseTypeCombo->currentData().toInt());
                noiseAmplitudeSpinBox->setEnabled(type != NoiseType::None);
            });

        mainLayout->addWidget(noiseGroup);

        signalConfig = new widgets::SignalConfigWidget(this);
        signalConfig->SetDefaultComponents({
            { 1000.0f, 0.15f },
            { 5000.0f, 0.5f },
            { 12000.0f, 0.25f },
        });
        mainLayout->addWidget(signalConfig);

        computeButton = new QPushButton("Compute PSD", this);
        computeButton->setMinimumHeight(40);
        mainLayout->addWidget(computeButton);

        mainLayout->addStretch();

        connect(computeButton, &QPushButton::clicked, this, &PsdConfigurationPanel::ComputeRequested);
    }

    PsdSimulator::Configuration PsdConfigurationPanel::GetConfiguration() const
    {
        PsdSimulator::Configuration config;
        config.inputSize = static_cast<std::size_t>(inputSizeSpinBox->value());
        config.segmentSize = segmentSizeCombo->currentData().toULongLong();
        config.sampleRateHz = static_cast<float>(sampleRateSpinBox->value());
        config.windowType = static_cast<WindowType>(windowTypeCombo->currentData().toInt());
        config.overlapPercent = overlapCombo->currentData().toULongLong();
        config.noise.type = static_cast<NoiseType>(noiseTypeCombo->currentData().toInt());
        config.noise.amplitude = static_cast<float>(noiseAmplitudeSpinBox->value());
        config.signalComponents = signalConfig->GetComponents();
        return config;
    }
}
