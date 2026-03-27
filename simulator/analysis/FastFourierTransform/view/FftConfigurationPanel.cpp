#include "simulator/analysis/FastFourierTransform/view/FftConfigurationPanel.hpp"
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

namespace simulator::analysis::view
{
    FftConfigurationPanel::FftConfigurationPanel(QWidget* parent)
        : QWidget(parent)
    {
        SetupUi();
    }

    void FftConfigurationPanel::SetupUi()
    {
        auto* mainLayout = new QVBoxLayout(this);

        auto* fftGroup = new QGroupBox("FFT Configuration", this);
        auto* fftLayout = new QVBoxLayout(fftGroup);

        auto* sizeLayout = new QHBoxLayout();
        sizeLayout->addWidget(new QLabel("FFT Size:", this));
        fftSizeCombo = new QComboBox(this);
        for (auto size : FftSimulator::SupportedFftSizes())
            fftSizeCombo->addItem(QString::number(size), QVariant::fromValue(static_cast<qulonglong>(size)));

        constexpr qulonglong defaultFftSize = 1024u;
        const int defaultIndex = fftSizeCombo->findData(QVariant::fromValue(defaultFftSize));
        if (defaultIndex != -1)
            fftSizeCombo->setCurrentIndex(defaultIndex);
        else if (fftSizeCombo->count() > 0)
            fftSizeCombo->setCurrentIndex(0);

        sizeLayout->addWidget(fftSizeCombo);
        fftLayout->addLayout(sizeLayout);

        auto* rateLayout = new QHBoxLayout();
        rateLayout->addWidget(new QLabel("Sample Rate (Hz):", this));
        sampleRateSpinBox = new QDoubleSpinBox(this);
        sampleRateSpinBox->setRange(100.0, 192000.0);
        sampleRateSpinBox->setValue(44100.0);
        sampleRateSpinBox->setDecimals(1);
        sampleRateSpinBox->setSuffix(" Hz");
        rateLayout->addWidget(sampleRateSpinBox);
        fftLayout->addLayout(rateLayout);

        auto* windowLayout = new QHBoxLayout();
        windowLayout->addWidget(new QLabel("Window:", this));
        windowTypeCombo = new QComboBox(this);
        windowTypeCombo->addItem("Rectangular", static_cast<int>(WindowType::Rectangular));
        windowTypeCombo->addItem("Hamming", static_cast<int>(WindowType::Hamming));
        windowTypeCombo->addItem("Hanning", static_cast<int>(WindowType::Hanning));
        windowTypeCombo->addItem("Blackman", static_cast<int>(WindowType::Blackman));
        windowLayout->addWidget(windowTypeCombo);
        fftLayout->addLayout(windowLayout);

        mainLayout->addWidget(fftGroup);

        signalConfig = new widgets::SignalConfigWidget(this);
        signalConfig->SetDefaultComponents({
            { 1000.0f, 0.15f },
            { 5000.0f, 0.5f },
            { 12000.0f, 0.25f },
        });
        mainLayout->addWidget(signalConfig);

        computeButton = new QPushButton("Compute FFT", this);
        computeButton->setMinimumHeight(40);
        mainLayout->addWidget(computeButton);

        mainLayout->addStretch();

        connect(computeButton, &QPushButton::clicked, this, &FftConfigurationPanel::ComputeRequested);
    }

    FftSimulator::Configuration FftConfigurationPanel::GetConfiguration() const
    {
        FftSimulator::Configuration config;
        config.fftSize = fftSizeCombo->currentData().toULongLong();
        config.sampleRateHz = static_cast<float>(sampleRateSpinBox->value());
        config.windowType = static_cast<WindowType>(windowTypeCombo->currentData().toInt());
        config.signalComponents = signalConfig->GetComponents();
        return config;
    }
}
