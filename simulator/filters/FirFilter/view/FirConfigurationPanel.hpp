#pragma once

#include "simulator/filters/FirFilter/application/FirFilterSimulator.hpp"
#include "simulator/widgets/SignalConfigWidget.hpp"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QWidget>

namespace simulator::filters::fir::view
{
    class FirConfigurationPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit FirConfigurationPanel(QWidget* parent = nullptr);

        FirFilterSimulator::Configuration GetConfiguration() const;

    signals:
        void ComputeRequested();

    private:
        QComboBox* filterTypeCombo;
        QDoubleSpinBox* cutoffSpinBox;
        QDoubleSpinBox* cutoffHighSpinBox;
        QSpinBox* orderSpinBox;
        QDoubleSpinBox* sampleRateSpinBox;
        QDoubleSpinBox* durationSpinBox;
        widgets::SignalConfigWidget* signalConfig;
    };
}
