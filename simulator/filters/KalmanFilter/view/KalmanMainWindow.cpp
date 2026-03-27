#include "simulator/filters/KalmanFilter/view/KalmanMainWindow.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include <QMessageBox>
#include <QSplitter>
#include <QStatusBar>
#include <cmath>

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

        thetaChart = new widgets::TimeSeriesChartWidget(tabWidget);
        thetaDotChart = new widgets::TimeSeriesChartWidget(tabWidget);
        covarianceChart = new widgets::TimeSeriesChartWidget(tabWidget);
        errorChart = new widgets::TimeSeriesChartWidget(tabWidget);

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

            thetaChart->SetTimeAxis(result.time);
            thetaChart->SetPanels({
                {
                    "Angle (θ) Estimation",
                    "θ (rad)",
                    {
                        { "Measurement", QColor(200, 200, 200), result.measuredTheta },
                        { "True", Qt::black, result.trueTheta },
                        { "KF", Qt::blue, result.kf.theta },
                        { "EKF", Qt::red, result.ekf.theta },
                        { "UKF", QColor(0, 150, 0), result.ukf.theta },
                    },
                    1,
                },
            });

            thetaDotChart->SetTimeAxis(result.time);
            thetaDotChart->SetPanels({
                {
                    "Angular Velocity (θ̇) Estimation",
                    "θ̇ (rad/s)",
                    {
                        { "True", Qt::black, result.trueThetaDot },
                        { "KF", Qt::blue, result.kf.thetaDot },
                        { "EKF", Qt::red, result.ekf.thetaDot },
                        { "UKF", QColor(0, 150, 0), result.ukf.thetaDot },
                    },
                    1,
                },
            });

            covarianceChart->SetTimeAxis(result.time);
            covarianceChart->SetPanels({
                {
                    "Position Covariance P(θ,θ)",
                    "Covariance",
                    {
                        { "KF", Qt::blue, result.kf.covarianceTheta },
                        { "EKF", Qt::red, result.ekf.covarianceTheta },
                        { "UKF", QColor(0, 150, 0), result.ukf.covarianceTheta },
                    },
                    1,
                },
            });

            std::vector<float> kfError(result.trueTheta.size());
            std::vector<float> ekfError(result.trueTheta.size());
            std::vector<float> ukfError(result.trueTheta.size());
            for (std::size_t i = 0; i < result.trueTheta.size(); ++i)
            {
                kfError[i] = std::abs(result.kf.theta[i] - result.trueTheta[i]);
                ekfError[i] = std::abs(result.ekf.theta[i] - result.trueTheta[i]);
                ukfError[i] = std::abs(result.ukf.theta[i] - result.trueTheta[i]);
            }

            errorChart->SetTimeAxis(result.time);
            errorChart->SetPanels({
                {
                    "Estimation Error |θ̂ - θ|",
                    "Error (rad)",
                    {
                        { "KF", Qt::blue, kfError },
                        { "EKF", Qt::red, ekfError },
                        { "UKF", QColor(0, 150, 0), ukfError },
                    },
                    1,
                },
            });

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
