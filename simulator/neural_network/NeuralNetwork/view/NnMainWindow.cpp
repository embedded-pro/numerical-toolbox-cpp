#include "simulator/neural_network/NeuralNetwork/view/NnMainWindow.hpp"
#include "simulator/neural_network/NeuralNetwork/view/NnConfigurationPanel.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>

namespace simulator::neural_network::nn::view
{
    NnMainWindow::NnMainWindow(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("Neural Network Simulator");
        resize(1200, 700);

        auto* splitter = new QSplitter(Qt::Horizontal, this);

        configPanel = new NnConfigurationPanel(splitter);
        configPanel->setMaximumWidth(350);

        tabWidget = new QTabWidget(splitter);

        lossChart = new widgets::TimeSeriesChartWidget(tabWidget);
        predictionChart = new widgets::TimeSeriesChartWidget(tabWidget);

        tabWidget->addTab(lossChart, "Training Loss");
        tabWidget->addTab(predictionChart, "Predictions");

        splitter->addWidget(configPanel);
        splitter->addWidget(tabWidget);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);
        statusBar()->showMessage("Configure network parameters and press Train");

        connect(configPanel, &NnConfigurationPanel::ComputeRequested, this, &NnMainWindow::OnComputeRequested);
    }

    void NnMainWindow::OnComputeRequested()
    {
        auto config = configPanel->GetConfiguration();

        NnSimulator simulator;
        simulator.Configure(config);
        auto result = simulator.Run();

        lossChart->SetTimeAxis(result.epochIndex);
        lossChart->SetPanels({
            {
                "Training Loss (MSE)",
                "Loss",
                {
                    { "MSE", QColor(231, 76, 60), result.lossHistory },
                },
                1,
            },
        });

        predictionChart->SetTimeAxis(result.predictionInputs);
        predictionChart->SetPanels({
            {
                "Target vs Prediction",
                "Value",
                {
                    { "Target", QColor(41, 128, 185), result.predictionTargets },
                    { "Prediction", QColor(231, 76, 60), result.predictionOutputs },
                },
                1,
            },
        });

        auto finalLoss = result.lossHistory.empty() ? 0.0f : result.lossHistory.back();
        statusBar()->showMessage(QString("Training complete: %1 epochs, final loss = %2")
                .arg(config.nn.epochs)
                .arg(static_cast<double>(finalLoss), 0, 'g', 6));
    }
}
