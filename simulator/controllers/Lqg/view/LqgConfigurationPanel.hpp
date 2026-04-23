#pragma once

#include "simulator/controllers/Lqg/application/LqgSimulator.hpp"
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QWidget>

namespace simulator::controllers::lqg::view
{
    class LqgConfigurationPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit LqgConfigurationPanel(QWidget* parent = nullptr);

        LqgSimulatorConfig GetConfiguration() const;
        LqgPlant CreatePlant() const;

    signals:
        void ComputeRequested();

    private:
        void SetupUi();

        QDoubleSpinBox* stateWeightSpinBox;
        QDoubleSpinBox* controlWeightSpinBox;
        QDoubleSpinBox* processNoiseSpinBox;
        QDoubleSpinBox* measurementNoiseSpinBox;
        QDoubleSpinBox* sampleTimeSpinBox;
        QDoubleSpinBox* durationSpinBox;
        QDoubleSpinBox* initialPositionSpinBox;
        QPushButton* computeButton;
    };
}
