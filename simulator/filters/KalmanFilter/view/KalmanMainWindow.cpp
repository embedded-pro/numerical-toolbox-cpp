#include "simulator/filters/KalmanFilter/view/KalmanMainWindow.hpp"
#include "simulator/shell/Guard.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <cmath>

namespace simulator::filters::view
{
    namespace
    {
        constexpr std::array<ui::shell::PageSpec, 4> pages{
            ui::shell::PageSpec{ "Angle (θ)" },
            ui::shell::PageSpec{ "Velocity (θ̇)" },
            ui::shell::PageSpec{ "Covariance" },
            ui::shell::PageSpec{ "Error" }
        };

        const ui::shell::ShellSpec shellSpec{
            "Kalman Filter Simulator — Pendulum",
            ui::Size{ 1280.0f, 900.0f },
            350.0f,
            pages,
            "Configure parameters and press Compute"
        };

    }

    KalmanMainWindow::KalmanMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , thetaView(new ui::backend::qt::QtPaintedWidget{ thetaChart, this })
        , thetaDotView(new ui::backend::qt::QtPaintedWidget{ thetaDotChart, this })
        , covarianceView(new ui::backend::qt::QtPaintedWidget{ covarianceChart, this })
        , errorView(new ui::backend::qt::QtPaintedWidget{ errorChart, this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        thetaView->SetPanCursorEnabled(true);
        thetaDotView->SetPanCursorEnabled(true);
        covarianceView->SetPanCursorEnabled(true);
        errorView->SetPanCursorEnabled(true);

        shell.SetPage(0, thetaView);
        shell.SetPage(1, thetaDotView);
        shell.SetPage(2, covarianceView);
        shell.SetPage(3, errorView);

        form.Model().onActionTriggered = [this](ui::model::ActionId)
        {
            OnComputeRequested();
        };
    }

    void KalmanMainWindow::OnComputeRequested()
    {
        shell::Guard(shell, "Simulation Error", [this]
            {
                auto config = form.BuildConfiguration();
                auto result = simulator.Run(config);

                const auto& theme = ui::theme::Current();

                thetaChart.SetAxisValues(result.time);
                thetaChart.SetPanels({
                    {
                        "Angle (θ) Estimation",
                        "θ (rad)",
                        {
                            { "Measurement", theme.Get(ui::theme::ColorRole::TextMuted), result.measuredTheta },
                            { "True", theme.Get(ui::theme::ColorRole::Text), result.trueTheta },
                            { "KF", theme.Series(0), result.kf.theta },
                            { "EKF", theme.Series(1), result.ekf.theta },
                            { "UKF", theme.Series(2), result.ukf.theta },
                        },
                        1,
                    },
                });

                thetaDotChart.SetAxisValues(result.time);
                thetaDotChart.SetPanels({
                    {
                        "Angular Velocity (θ̇) Estimation",
                        "θ̇ (rad/s)",
                        {
                            { "True", theme.Get(ui::theme::ColorRole::Text), result.trueThetaDot },
                            { "KF", theme.Series(0), result.kf.thetaDot },
                            { "EKF", theme.Series(1), result.ekf.thetaDot },
                            { "UKF", theme.Series(2), result.ukf.thetaDot },
                        },
                        1,
                    },
                });

                covarianceChart.SetAxisValues(result.time);
                covarianceChart.SetPanels({
                    {
                        "Position Covariance P(θ,θ)",
                        "Covariance",
                        {
                            { "KF", theme.Series(0), result.kf.covarianceTheta },
                            { "EKF", theme.Series(1), result.ekf.covarianceTheta },
                            { "UKF", theme.Series(2), result.ukf.covarianceTheta },
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

                errorChart.SetAxisValues(result.time);
                errorChart.SetPanels({
                    {
                        "Estimation Error |θ̂ - θ|",
                        "Error (rad)",
                        {
                            { "KF", theme.Series(0), kfError },
                            { "EKF", theme.Series(1), ekfError },
                            { "UKF", theme.Series(2), ukfError },
                        },
                        1,
                    },
                });

                thetaView->update();
                thetaDotView->update();
                covarianceView->update();
                errorView->update();

                shell.SetStatus(QString("Simulation complete — %1 steps, dt=%2s")
                                    .arg(result.time.size())
                                    .arg(static_cast<double>(config.dt), 0, 'f', 3)
                                    .toStdString());
            });
    }
}
