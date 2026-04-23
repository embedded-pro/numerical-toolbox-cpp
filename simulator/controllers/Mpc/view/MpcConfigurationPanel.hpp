#pragma once

#include "simulator/controllers/Mpc/application/MpcSimulator.hpp"
#include "simulator/controllers/Mpc/application/StateSpacePlant.hpp"
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QPushButton>
#include <QWidget>

namespace simulator::controllers::view
{
    class MpcConfigurationPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit MpcConfigurationPanel(QWidget* parent = nullptr);

        MpcSimulator::Configuration GetConfiguration() const;
        math::LinearTimeInvariant<float, 2, 1> CreatePlant() const;
        QString GetPlantDescription() const;

    signals:
        void ComputeRequested();

    private:
        void SetupUi();

        QDoubleSpinBox* stateWeightSpinBox;
        QDoubleSpinBox* controlWeightSpinBox;

        QCheckBox* constraintsEnabled;
        QDoubleSpinBox* uMinSpinBox;
        QDoubleSpinBox* uMaxSpinBox;

        QComboBox* plantCombo;
        QDoubleSpinBox* sampleTimeSpinBox;
        QDoubleSpinBox* durationSpinBox;
        QDoubleSpinBox* referenceSpinBox;

        QDoubleSpinBox* plantGainSpinBox;
        QDoubleSpinBox* plantTimeConstantSpinBox;
        QGroupBox* firstOrderGroup;

        QPushButton* computeButton;
    };
}
