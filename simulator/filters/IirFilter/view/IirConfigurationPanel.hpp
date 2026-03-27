#pragma once

#include "simulator/filters/IirFilter/application/IirFilterSimulator.hpp"
#include "simulator/widgets/SignalConfigWidget.hpp"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QWidget>

namespace simulator::filters::iir::view
{
    class IirConfigurationPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit IirConfigurationPanel(QWidget* parent = nullptr);

        IirFilterSimulator::Configuration GetConfiguration() const;

    signals:
        void ComputeRequested();

    private:
        QComboBox* filterTypeCombo;
        QDoubleSpinBox* cutoffSpinBox;
        QDoubleSpinBox* qualitySpinBox;
        QDoubleSpinBox* sampleRateSpinBox;
        QDoubleSpinBox* durationSpinBox;
        widgets::SignalConfigWidget* signalConfig;
    };
}
