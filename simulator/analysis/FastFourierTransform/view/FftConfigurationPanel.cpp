#include "simulator/analysis/FastFourierTransform/view/FftConfigurationPanel.hpp"
#include <QHBoxLayout>
#include <QHeaderView>
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
        fftSizeCombo->setCurrentIndex(4);
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

        mainLayout->addWidget(fftGroup);

        auto* signalGroup = new QGroupBox("Signal Components", this);
        auto* signalLayout = new QVBoxLayout(signalGroup);

        componentsTable = new QTableWidget(0, 2, this);
        componentsTable->setHorizontalHeaderLabels({ "Frequency (Hz)", "Amplitude" });
        componentsTable->horizontalHeader()->setStretchLastSection(true);
        componentsTable->setSelectionBehavior(QAbstractItemView::SelectRows);
        signalLayout->addWidget(componentsTable);

        AddSignalComponent();
        componentsTable->item(0, 0)->setText("1000");
        componentsTable->item(0, 1)->setText("0.15");

        AddSignalComponent();
        componentsTable->item(1, 0)->setText("5000");
        componentsTable->item(1, 1)->setText("0.5");

        AddSignalComponent();
        componentsTable->item(2, 0)->setText("12000");
        componentsTable->item(2, 1)->setText("0.25");

        auto* buttonLayout = new QHBoxLayout();
        addComponentButton = new QPushButton("Add", this);
        removeComponentButton = new QPushButton("Remove", this);
        buttonLayout->addWidget(addComponentButton);
        buttonLayout->addWidget(removeComponentButton);
        signalLayout->addLayout(buttonLayout);

        mainLayout->addWidget(signalGroup);

        computeButton = new QPushButton("Compute FFT", this);
        computeButton->setMinimumHeight(40);
        mainLayout->addWidget(computeButton);

        mainLayout->addStretch();

        connect(addComponentButton, &QPushButton::clicked, this, &FftConfigurationPanel::AddSignalComponent);
        connect(removeComponentButton, &QPushButton::clicked, this, &FftConfigurationPanel::RemoveSignalComponent);
        connect(computeButton, &QPushButton::clicked, this, &FftConfigurationPanel::ComputeRequested);
    }

    FftSimulator::Configuration FftConfigurationPanel::GetConfiguration() const
    {
        FftSimulator::Configuration config;
        config.fftSize = fftSizeCombo->currentData().toULongLong();
        config.sampleRateHz = static_cast<float>(sampleRateSpinBox->value());

        config.signalComponents.clear();
        for (int row = 0; row < componentsTable->rowCount(); ++row)
        {
            auto* freqItem = componentsTable->item(row, 0);
            auto* ampItem = componentsTable->item(row, 1);

            if (freqItem && ampItem)
            {
                SignalComponent component;
                component.frequencyHz = freqItem->text().toFloat();
                component.amplitude = ampItem->text().toFloat();
                config.signalComponents.push_back(component);
            }
        }

        return config;
    }

    void FftConfigurationPanel::AddSignalComponent()
    {
        int row = componentsTable->rowCount();
        componentsTable->insertRow(row);
        componentsTable->setItem(row, 0, new QTableWidgetItem("1000"));
        componentsTable->setItem(row, 1, new QTableWidgetItem("0.5"));
    }

    void FftConfigurationPanel::RemoveSignalComponent()
    {
        auto selectedRows = componentsTable->selectionModel()->selectedRows();
        if (!selectedRows.isEmpty())
            componentsTable->removeRow(selectedRows.first().row());
        else if (componentsTable->rowCount() > 0)
            componentsTable->removeRow(componentsTable->rowCount() - 1);
    }
}
