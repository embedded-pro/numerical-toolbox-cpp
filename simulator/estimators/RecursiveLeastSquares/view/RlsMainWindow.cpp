#include "simulator/estimators/RecursiveLeastSquares/view/RlsMainWindow.hpp"
#include "simulator/shell/Guard.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <format>

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
    }

    RlsMainWindow::RlsMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , outputView(new ui::backend::qt::QtPaintedWidget{ outputChart, this })
        , coefficientView(new ui::backend::qt::QtPaintedWidget{ coefficientChart, this })
        , metricsView(new ui::backend::qt::QtPaintedWidget{ metricsChart, this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        outputView->SetPanCursorEnabled(true);
        coefficientView->SetPanCursorEnabled(true);
        metricsView->SetPanCursorEnabled(true);

        shell.SetPage(0, outputView);
        shell.SetPage(1, coefficientView);
        shell.SetPage(2, metricsView);

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

                const auto& theme = ui::theme::Current();

                outputChart.SetAxisValues(result.sampleIndex);
                outputChart.SetPanels({
                    {
                        "True vs Estimated Output",
                        "Value",
                        {
                            { "True", theme.Series(0), result.trueOutput },
                            { "Estimated", theme.Series(1), result.estimatedOutput },
                        },
                        1,
                    },
                });

                std::vector<ui::charts::Series> coefficientSeries;

                for (std::size_t i = 0; i < result.coefficientHistory.size(); ++i)
                {
                    const auto trueValue = i < config.rls.trueCoefficients.size() ? config.rls.trueCoefficients[i] : 0.0f;

                    coefficientSeries.push_back({
                        std::format("θ{} (true={:.2f})", i, static_cast<double>(trueValue)),
                        theme.Series(i % 4),
                        result.coefficientHistory[i],
                    });
                }

                coefficientChart.SetAxisValues(result.sampleIndex);
                coefficientChart.SetPanels({
                    {
                        "Coefficient Convergence",
                        "Coefficient Value",
                        coefficientSeries,
                        1,
                    },
                });

                metricsChart.SetAxisValues(result.sampleIndex);
                metricsChart.SetPanels({
                    {
                        "Innovation (pre-update error)",
                        "Error",
                        {
                            { "Innovation", theme.Series(1), result.innovationHistory },
                        },
                        1,
                    },
                    {
                        "Uncertainty (trace of P)",
                        "Trace(P)",
                        {
                            { "Uncertainty", theme.Series(3), result.uncertaintyHistory },
                        },
                        1,
                    },
                });

                outputView->update();
                coefficientView->update();
                metricsView->update();

                shell.SetStatus(QString("RLS estimation complete: %1 samples, λ=%2")
                        .arg(config.rls.numSamples)
                        .arg(static_cast<double>(config.rls.forgettingFactor), 0, 'f', 3)
                        .toStdString());
            });
    }
}
