#include "simulator/controllers/PidController/view/PidMainWindow.hpp"
#include <QHBoxLayout>
#include <QMessageBox>
#include <QSplitter>
#include <stdexcept>

namespace simulator::controllers::view
{
    PidMainWindow::PidMainWindow(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("PID Controller Simulator");
        resize(1280, 900);

        auto* splitter = new QSplitter(Qt::Horizontal, this);

        configPanel = new PidConfigurationPanel(splitter);
        configPanel->setMaximumWidth(350);

        tabWidget = new QTabWidget(splitter);

        stepChart = new PidStepResponseWidget(tabWidget);
        rampChart = new PidRampResponseWidget(tabWidget);
        bodeChart = new PidBodeWidget(tabWidget);
        rootLocusChart = new PidRootLocusWidget(tabWidget);

        tabWidget->addTab(stepChart, "Step Response");
        tabWidget->addTab(rampChart, "Ramp Response");
        tabWidget->addTab(bodeChart, "Bode Plot");
        tabWidget->addTab(rootLocusChart, "Root Locus");

        splitter->addWidget(configPanel);
        splitter->addWidget(tabWidget);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);

        statusBar()->showMessage("Configure PID parameters and press Compute");

        connect(configPanel, &PidConfigurationPanel::ComputeRequested, this, &PidMainWindow::OnComputeRequested);
        connect(rootLocusChart, &PidRootLocusWidget::PoleGainChanged, this, &PidMainWindow::OnPoleGainChanged);
    }

    void PidMainWindow::OnComputeRequested()
    {
        auto config = configPanel->GetConfiguration();
        pidSimulator.Configure(configPanel->CreatePlant(), config);

        try
        {
            auto stepResult = pidSimulator.ComputeStepResponse();
            stepChart->SetData(stepResult);

            auto rampResult = pidSimulator.ComputeRampResponse();
            rampChart->SetData(rampResult);

            auto bodeResult = pidSimulator.ComputeBodeResponse();
            bodeChart->SetData(bodeResult);

            auto rootLocusResult = pidSimulator.ComputeRootLocus();
            rootLocusChart->SetData(rootLocusResult);

            statusBar()->showMessage(
                QString("PID: Kp=%1 Ki=%2 Kd=%3 | Plant: %4")
                    .arg(static_cast<double>(config.tunings.kp), 0, 'f', 3)
                    .arg(static_cast<double>(config.tunings.ki), 0, 'f', 3)
                    .arg(static_cast<double>(config.tunings.kd), 0, 'f', 3)
                    .arg(configPanel->GetPlantDescription()));
        }
        catch (const std::exception& e)
        {
            QMessageBox::warning(this, "Computation Error", e.what());
            statusBar()->showMessage("Computation failed");
        }
    }

    void PidMainWindow::OnPoleGainChanged(float gain)
    {
        configPanel->SetKp(gain);

        auto config = configPanel->GetConfiguration();
        pidSimulator.Configure(configPanel->CreatePlant(), config);

        try
        {
            auto stepResult = pidSimulator.ComputeStepResponse();
            stepChart->SetData(stepResult);

            auto rampResult = pidSimulator.ComputeRampResponse();
            rampChart->SetData(rampResult);

            auto bodeResult = pidSimulator.ComputeBodeResponse();
            bodeChart->SetData(bodeResult);

            auto rootLocusResult = pidSimulator.ComputeRootLocus();
            rootLocusChart->SetData(rootLocusResult);
        }
        catch (const std::exception&)
        {
        }
    }
}
