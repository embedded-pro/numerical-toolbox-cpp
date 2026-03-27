#pragma once

#include "simulator/analysis/FastFourierTransform/application/FftSimulator.hpp"
#include "simulator/widgets/SignalConfigWidget.hpp"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QPushButton>
#include <QSpinBox>
#include <QWidget>

namespace simulator::analysis::view
{
    class FftConfigurationPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit FftConfigurationPanel(QWidget* parent = nullptr);

        FftSimulator::Configuration GetConfiguration() const;

    signals:
        void ComputeRequested();

    private:
        void SetupUi();

        QComboBox* fftSizeCombo;
        QComboBox* windowTypeCombo;
        QDoubleSpinBox* sampleRateSpinBox;
        widgets::SignalConfigWidget* signalConfig;
        QPushButton* computeButton;
    };
}
