#include "simulator/filters/KalmanFilter/view/KalmanMainWindow.hpp"
#include "simulator/shell/Guard.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
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

        [[nodiscard]] QColor ToQt(ui::Color color)
        {
            return QColor{ color.red, color.green, color.blue };
        }

        [[nodiscard]] QColor Role(ui::theme::ColorRole role)
        {
            return ToQt(ui::theme::Current().Get(role));
        }

        [[nodiscard]] QColor Series(std::size_t index)
        {
            return ToQt(ui::theme::Current().Series(index));
        }
    }

    KalmanMainWindow::KalmanMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , thetaChart(new widgets::TimeSeriesChartWidget{ this })
        , thetaDotChart(new widgets::TimeSeriesChartWidget{ this })
        , covarianceChart(new widgets::TimeSeriesChartWidget{ this })
        , errorChart(new widgets::TimeSeriesChartWidget{ this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        shell.SetPage(0, thetaChart);
        shell.SetPage(1, thetaDotChart);
        shell.SetPage(2, covarianceChart);
        shell.SetPage(3, errorChart);

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

                thetaChart->SetTimeAxis(result.time);
                thetaChart->SetPanels({
                    {
                        "Angle (θ) Estimation",
                        "θ (rad)",
                        {
                            { "Measurement", Role(ui::theme::ColorRole::TextMuted), result.measuredTheta },
                            { "True", Role(ui::theme::ColorRole::Text), result.trueTheta },
                            { "KF", Series(0), result.kf.theta },
                            { "EKF", Series(1), result.ekf.theta },
                            { "UKF", Series(2), result.ukf.theta },
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
                            { "True", Role(ui::theme::ColorRole::Text), result.trueThetaDot },
                            { "KF", Series(0), result.kf.thetaDot },
                            { "EKF", Series(1), result.ekf.thetaDot },
                            { "UKF", Series(2), result.ukf.thetaDot },
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
                            { "KF", Series(0), result.kf.covarianceTheta },
                            { "EKF", Series(1), result.ekf.covarianceTheta },
                            { "UKF", Series(2), result.ukf.covarianceTheta },
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
                            { "KF", Series(0), kfError },
                            { "EKF", Series(1), ekfError },
                            { "UKF", Series(2), ukfError },
                        },
                        1,
                    },
                });

                shell.SetStatus(QString("Simulation complete — %1 steps, dt=%2s")
                        .arg(result.time.size())
                        .arg(static_cast<double>(config.dt), 0, 'f', 3)
                        .toStdString());
            });
    }
}
