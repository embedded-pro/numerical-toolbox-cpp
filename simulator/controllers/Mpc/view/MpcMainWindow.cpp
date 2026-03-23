#include "simulator/controllers/Mpc/view/MpcMainWindow.hpp"
#include <QHBoxLayout>
#include <QMessageBox>
#include <QSplitter>
#include <stdexcept>

namespace simulator::controllers::view
{
    MpcMainWindow::MpcMainWindow(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("MPC Controller Simulator");
        resize(1280, 900);

        auto* splitter = new QSplitter(Qt::Horizontal, this);

        configPanel = new MpcConfigurationPanel(splitter);
        configPanel->setMaximumWidth(350);

        tabWidget = new QTabWidget(splitter);

        stepChart = new MpcStepResponseWidget(tabWidget);
        constrainedChart = new MpcConstrainedResponseWidget(tabWidget);

        tabWidget->addTab(stepChart, "Step Response");
        tabWidget->addTab(constrainedChart, "Constrained Response");

        splitter->addWidget(configPanel);
        splitter->addWidget(tabWidget);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);

        statusBar()->showMessage("Configure MPC parameters and press Compute");

        connect(configPanel, &MpcConfigurationPanel::ComputeRequested, this, &MpcMainWindow::OnComputeRequested);
    }

    void MpcMainWindow::OnComputeRequested()
    {
        try
        {
            auto config = configPanel->GetConfiguration();
            auto plant = configPanel->CreatePlant();
            mpcSimulator.Configure(plant, config);

            auto stepResult = mpcSimulator.ComputeStepResponse();
            stepChart->SetData(stepResult, config.referencePosition);

            auto constrainedResult = mpcSimulator.ComputeConstrainedResponse();
            constrainedChart->SetData(constrainedResult, config.referencePosition);

            statusBar()->showMessage(
                QString("MPC: Q=%1 R=%2 | Plant: %3")
                    .arg(static_cast<double>(config.weights.stateWeight), 0, 'f', 2)
                    .arg(static_cast<double>(config.weights.controlWeight), 0, 'f', 3)
                    .arg(configPanel->GetPlantDescription()));
        }
        catch (const std::exception& e)
        {
            QMessageBox::warning(this, "Computation Error", e.what());
            statusBar()->showMessage("Computation failed");
        }
    }
}
