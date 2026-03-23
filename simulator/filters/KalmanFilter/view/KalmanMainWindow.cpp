#include "simulator/filters/KalmanFilter/view/KalmanMainWindow.hpp"
#include <QMessageBox>
#include <QSplitter>
#include <QStatusBar>

namespace simulator::filters::view
{
    KalmanMainWindow::KalmanMainWindow(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("Kalman Filter Simulator — Pendulum");
        resize(1280, 900);

        auto* splitter = new QSplitter(Qt::Horizontal, this);

        configPanel = new KalmanConfigPanel(splitter);
        configPanel->setMaximumWidth(350);

        tabWidget = new QTabWidget(splitter);

        thetaChart = new ThetaChartWidget(tabWidget);
        thetaDotChart = new ThetaDotChartWidget(tabWidget);
        covarianceChart = new CovarianceChartWidget(tabWidget);
        errorChart = new ErrorChartWidget(tabWidget);

        tabWidget->addTab(thetaChart, "Angle (θ)");
        tabWidget->addTab(thetaDotChart, "Velocity (θ̇)");
        tabWidget->addTab(covarianceChart, "Covariance");
        tabWidget->addTab(errorChart, "Error");

        splitter->addWidget(configPanel);
        splitter->addWidget(tabWidget);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);

        statusBar()->showMessage("Configure parameters and press Compute");

        connect(configPanel, &KalmanConfigPanel::ComputeRequested, this, &KalmanMainWindow::OnComputeRequested);
    }

    void KalmanMainWindow::OnComputeRequested()
    {
        try
        {
            auto config = configPanel->GetConfig();
            auto result = simulator.Run(config);

            thetaChart->SetData(result);
            thetaDotChart->SetData(result);
            covarianceChart->SetData(result);
            errorChart->SetData(result);

            statusBar()->showMessage(
                QString("Simulation complete — %1 steps, dt=%2s")
                    .arg(result.time.size())
                    .arg(static_cast<double>(config.dt), 0, 'f', 3));
        }
        catch (const std::exception& e)
        {
            QMessageBox::warning(this, "Simulation Error", e.what());
            statusBar()->showMessage("Simulation failed");
        }
    }
}
