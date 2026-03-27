#include "simulator/neural_network/NeuralNetwork/view/NnConfigurationPanel.hpp"
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace simulator::neural_network::nn::view
{
    NnConfigurationPanel::NnConfigurationPanel(QWidget* parent)
        : QWidget(parent)
    {
        auto* layout = new QVBoxLayout(this);

        auto* demoGroup = new QGroupBox("Demo Selection", this);
        auto* demoLayout = new QVBoxLayout(demoGroup);

        demoLayout->addWidget(new QLabel("Problem:"));
        demoCombo = new QComboBox(demoGroup);
        demoCombo->addItem("XOR Classification", static_cast<int>(DemoType::Xor));
        demoCombo->addItem("Sine Approximation", static_cast<int>(DemoType::SineApproximation));
        demoLayout->addWidget(demoCombo);

        layout->addWidget(demoGroup);

        auto* networkGroup = new QGroupBox("Network Parameters", this);
        auto* networkLayout = new QVBoxLayout(networkGroup);

        networkLayout->addWidget(new QLabel("Hidden Layer Size:"));
        hiddenSizeSpinBox = new QSpinBox(networkGroup);
        hiddenSizeSpinBox->setRange(2, 64);
        hiddenSizeSpinBox->setSingleStep(2);
        hiddenSizeSpinBox->setValue(8);
        networkLayout->addWidget(hiddenSizeSpinBox);

        networkLayout->addWidget(new QLabel("Learning Rate:"));
        learningRateSpinBox = new QDoubleSpinBox(networkGroup);
        learningRateSpinBox->setRange(0.001, 10.0);
        learningRateSpinBox->setDecimals(3);
        learningRateSpinBox->setSingleStep(0.01);
        learningRateSpinBox->setValue(0.5);
        networkLayout->addWidget(learningRateSpinBox);

        networkLayout->addWidget(new QLabel("Epochs:"));
        epochsSpinBox = new QSpinBox(networkGroup);
        epochsSpinBox->setRange(10, 10000);
        epochsSpinBox->setSingleStep(100);
        epochsSpinBox->setValue(1000);
        networkLayout->addWidget(epochsSpinBox);

        networkLayout->addWidget(new QLabel("Sine Samples:"));
        sineSamplesSpinBox = new QSpinBox(networkGroup);
        sineSamplesSpinBox->setRange(10, 500);
        sineSamplesSpinBox->setSingleStep(10);
        sineSamplesSpinBox->setValue(50);
        networkLayout->addWidget(sineSamplesSpinBox);

        layout->addWidget(networkGroup);

        auto* computeButton = new QPushButton("Train", this);
        layout->addWidget(computeButton);

        layout->addStretch();

        connect(computeButton, &QPushButton::clicked, this, &NnConfigurationPanel::ComputeRequested);
    }

    NnSimulator::Configuration NnConfigurationPanel::GetConfiguration() const
    {
        NnSimulator::Configuration config;
        config.nn.demo = static_cast<DemoType>(demoCombo->currentData().toInt());
        config.nn.hiddenSize = static_cast<std::size_t>(hiddenSizeSpinBox->value());
        config.nn.learningRate = static_cast<float>(learningRateSpinBox->value());
        config.nn.epochs = static_cast<std::size_t>(epochsSpinBox->value());
        config.nn.sineSamples = static_cast<std::size_t>(sineSamplesSpinBox->value());
        return config;
    }
}
