#pragma once

#include "simulator/neural_network/NeuralNetwork/application/NnSimulator.hpp"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QWidget>

namespace simulator::neural_network::nn::view
{
    class NnConfigurationPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit NnConfigurationPanel(QWidget* parent = nullptr);

        NnSimulator::Configuration GetConfiguration() const;

    signals:
        void ComputeRequested();

    private:
        QComboBox* demoCombo;
        QSpinBox* hiddenSizeSpinBox;
        QDoubleSpinBox* learningRateSpinBox;
        QSpinBox* epochsSpinBox;
        QSpinBox* sineSamplesSpinBox;
    };
}
