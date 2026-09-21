#include "simulator/controllers/BayesianMpcCalibration/view/BayesianMpcCalibrationView.hpp"
#include "simulator/shell/Guard.hpp"
#include "ui/theme/Theme.hpp"
#include <algorithm>
#include <array>
#include <limits>
#include <vector>

namespace simulator::controllers::view
{
    namespace
    {
        constexpr std::array<ui::shell::PageSpec, 4> pages{
            ui::shell::PageSpec{ "1. Observations" },
            ui::shell::PageSpec{ "2. EM Convergence" },
            ui::shell::PageSpec{ "3. BO Calibration" },
            ui::shell::PageSpec{ "4. Step Response" }
        };

        const ui::shell::ShellSpec shellSpec{
            "Bayesian MPC Calibration Simulator",
            ui::Size{ 1400.0f, 900.0f },
            320.0f,
            pages,
            "Configure parameters and press Run Pipeline"
        };

        [[nodiscard]] QColor ToQt(ui::Color color)
        {
            return QColor{ color.red, color.green, color.blue };
        }

        [[nodiscard]] QColor Series(std::size_t index)
        {
            return ToQt(ui::theme::Current().Series(index));
        }
    }

    BayesianMpcCalibrationView::BayesianMpcCalibrationView(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , observationsChart(new widgets::TimeSeriesChartWidget{ this })
        , emConvergenceChart(new widgets::TimeSeriesChartWidget{ this })
        , boConvergenceChart(new widgets::TimeSeriesChartWidget{ this })
        , stepResponseChart(new widgets::TimeSeriesChartWidget{ this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        shell.SetPage(0, observationsChart);
        shell.SetPage(1, emConvergenceChart);
        shell.SetPage(2, boConvergenceChart);
        shell.SetPage(3, stepResponseChart);

        form.Model().onActionTriggered = [this](ui::model::ActionId)
        {
            OnRunRequested();
        };
    }

    void BayesianMpcCalibrationView::OnRunRequested()
    {
        formView->SetActionEnabled(bayesian::field::run, false);
        shell.SetStatus("Running pipeline (EM + BO) ...");

        shell::Guard(shell, "Error", [this]
            {
                const auto config = form.BuildConfiguration();

                BayesianMpcCalibrationSimulator simulator;
                const auto results = simulator.Run(config);

                DisplayResults(results);

                shell.SetStatus(QString("Done. EM: %1 iters (%2) | Opt Q=%3 R=%4 ISE=%5")
                                    .arg(static_cast<int>(results.emIterations))
                                    .arg(results.emConverged ? "converged" : "max iters")
                                    .arg(static_cast<double>(results.optimalQ), 0, 'f', 2)
                                    .arg(static_cast<double>(results.optimalR), 0, 'f', 2)
                                    .arg(static_cast<double>(results.finalIse), 0, 'e', 3)
                                    .toStdString());
            });

        formView->SetActionEnabled(bayesian::field::run, true);
    }

    void BayesianMpcCalibrationView::DisplayResults(const CalibrationSimulationResults& results)
    {
        observationsChart->SetTimeAxis(results.timeAxis);
        observationsChart->SetPanels({
            {
                "Observations vs True Position",
                "Position",
                {
                    { "True Position", Series(0), results.truePositions },
                    { "Noisy Measurements", Series(1), results.rawMeasurements },
                },
                1,
            },
        });

        std::vector<float> emIterationAxis;
        emIterationAxis.reserve(results.emLogLikelihoodHistory.size());

        for (std::size_t i = 0; i < results.emLogLikelihoodHistory.size(); ++i)
            emIterationAxis.push_back(static_cast<float>(i + 1));

        emConvergenceChart->SetTimeAxis(emIterationAxis);
        emConvergenceChart->SetPanels({
            {
                "EM Log-Likelihood vs Iteration",
                "Log-Likelihood",
                {
                    { "Log-Likelihood", Series(2), results.emLogLikelihoodHistory },
                },
                1,
            },
        });

        std::vector<float> boIterationAxis;
        std::vector<float> runningMinimum;
        boIterationAxis.reserve(results.boIseHistory.size());
        runningMinimum.reserve(results.boIseHistory.size());
        auto minimum = std::numeric_limits<float>::max();

        for (std::size_t i = 0; i < results.boIseHistory.size(); ++i)
        {
            boIterationAxis.push_back(static_cast<float>(i + 1));
            minimum = std::min(minimum, results.boIseHistory[i]);
            runningMinimum.push_back(minimum);
        }

        boConvergenceChart->SetTimeAxis(boIterationAxis);
        boConvergenceChart->SetPanels({
            {
                "BO: Best ISE Found vs Evaluation",
                "ISE",
                {
                    { "All ISE values", Series(4), results.boIseHistory },
                    { "Best so far", Series(1), runningMinimum },
                },
                1,
            },
        });

        std::vector<float> referenceLine(results.stepResponseTime.size(), 1.0f);

        stepResponseChart->SetTimeAxis(results.stepResponseTime);
        stepResponseChart->SetPanels({
            {
                "Position",
                "m",
                {
                    { "Position", Series(0), results.stepResponsePosition },
                    { "Reference", ToQt(ui::theme::Current().Get(ui::theme::ColorRole::TextMuted)), referenceLine },
                },
                2,
            },
            {
                "Velocity",
                "m/s",
                {
                    { "Velocity", Series(2), results.stepResponseVelocity },
                },
                1,
            },
            {
                "Control Input",
                "N",
                {
                    { "Control", Series(1), results.stepResponseControl },
                },
                1,
            },
        });
    }
}
