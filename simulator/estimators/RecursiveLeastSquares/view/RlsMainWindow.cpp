#include "simulator/estimators/RecursiveLeastSquares/view/RlsMainWindow.hpp"
#include "simulator/shell/Guard.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include "ui/theme/Theme.hpp"
#include <array>

namespace simulator::estimators::rls::view
{
    namespace
    {
        constexpr std::array<ui::shell::PageSpec, 3> pages{
            ui::shell::PageSpec{ "Output Tracking" },
            ui::shell::PageSpec{ "Coefficient Convergence" },
            ui::shell::PageSpec{ "Estimation Metrics" }
        };

        const ui::shell::ShellSpec shellSpec{
            "Recursive Least Squares Simulator",
            ui::Size{ 1200.0f, 700.0f },
            350.0f,
            pages,
            "Configure RLS parameters and press Compute"
        };

        [[nodiscard]] QColor Series(std::size_t index)
        {
            const auto color = ui::theme::Current().Series(index);
            return QColor{ color.red, color.green, color.blue };
        }
    }

    RlsMainWindow::RlsMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , outputChart(new widgets::TimeSeriesChartWidget{ this })
        , coefficientChart(new widgets::TimeSeriesChartWidget{ this })
        , metricsChart(new widgets::TimeSeriesChartWidget{ this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        shell.SetPage(0, outputChart);
        shell.SetPage(1, coefficientChart);
        shell.SetPage(2, metricsChart);

        form.Model().onActionTriggered = [this](ui::model::ActionId)
        {
            OnComputeRequested();
        };
    }

    void RlsMainWindow::OnComputeRequested()
    {
        shell::Guard(shell, "Computation Error", [this]
            {
                auto config = form.BuildConfiguration();

                RlsSimulator simulator;
                simulator.Configure(config);
                auto result = simulator.Run();

                outputChart->SetTimeAxis(result.sampleIndex);
                outputChart->SetPanels({
                    {
                        "True vs Estimated Output",
                        "Value",
                        {
                            { "True", Series(0), result.trueOutput },
                            { "Estimated", Series(1), result.estimatedOutput },
                        },
                        1,
                    },
                });

                std::vector<widgets::Series> coefficientSeries;

                for (std::size_t i = 0; i < result.coefficientHistory.size(); ++i)
                {
                    const auto trueValue = i < config.rls.trueCoefficients.size() ? config.rls.trueCoefficients[i] : 0.0f;

                    coefficientSeries.push_back({
                        QString("θ%1 (true=%2)").arg(i).arg(static_cast<double>(trueValue), 0, 'f', 2),
                        Series(i % 4),
                        result.coefficientHistory[i],
                    });
                }

                coefficientChart->SetTimeAxis(result.sampleIndex);
                coefficientChart->SetPanels({
                    {
                        "Coefficient Convergence",
                        "Coefficient Value",
                        coefficientSeries,
                        1,
                    },
                });

                metricsChart->SetTimeAxis(result.sampleIndex);
                metricsChart->SetPanels({
                    {
                        "Innovation (pre-update error)",
                        "Error",
                        {
                            { "Innovation", Series(1), result.innovationHistory },
                        },
                        1,
                    },
                    {
                        "Uncertainty (trace of P)",
                        "Trace(P)",
                        {
                            { "Uncertainty", Series(3), result.uncertaintyHistory },
                        },
                        1,
                    },
                });

                shell.SetStatus(QString("RLS estimation complete: %1 samples, λ=%2")
                                    .arg(config.rls.numSamples)
                                    .arg(static_cast<double>(config.rls.forgettingFactor), 0, 'f', 3)
                                    .toStdString());
            });
    }
}
