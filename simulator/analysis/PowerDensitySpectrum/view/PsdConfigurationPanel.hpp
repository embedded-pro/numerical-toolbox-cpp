#pragma once

#include "simulator/analysis/PowerDensitySpectrum/application/PsdSimulator.hpp"
#include "simulator/widgets/SignalConfigWidget.hpp"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QPushButton>
#include <QWidget>

namespace simulator::analysis::psd::view
{
    class PsdConfigurationPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit PsdConfigurationPanel(QWidget* parent = nullptr);

        PsdSimulator::Configuration GetConfiguration() const;

    signals:
        void ComputeRequested();

    private:
        void SetupUi();

        QComboBox* segmentSizeCombo;
        QComboBox* windowTypeCombo;
        QComboBox* overlapCombo;
        QDoubleSpinBox* sampleRateSpinBox;
        QDoubleSpinBox* inputSizeSpinBox;
        QComboBox* noiseTypeCombo;
        QDoubleSpinBox* noiseAmplitudeSpinBox;
        widgets::SignalConfigWidget* signalConfig;
        QPushButton* computeButton;
    };
}
