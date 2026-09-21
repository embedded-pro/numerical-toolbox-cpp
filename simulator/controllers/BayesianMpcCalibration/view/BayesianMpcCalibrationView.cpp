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

    }

    BayesianMpcCalibrationView::BayesianMpcCalibrationView(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , observationsView(new ui::backend::qt::QtPaintedWidget{ observationsChart, this })
        , emConvergenceView(new ui::backend::qt::QtPaintedWidget{ emConvergenceChart, this })
        , boConvergenceView(new ui::backend::qt::QtPaintedWidget{ boConvergenceChart, this })
        , stepResponseView(new ui::backend::qt::QtPaintedWidget{ stepResponseChart, this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        observationsView->SetPanCursorEnabled(true);
        emConvergenceView->SetPanCursorEnabled(true);
        boConvergenceView->SetPanCursorEnabled(true);
        stepResponseView->SetPanCursorEnabled(true);

        shell.SetPage(0, observationsView);
        shell.SetPage(1, emConvergenceView);
        shell.SetPage(2, boConvergenceView);
        shell.SetPage(3, stepResponseView);

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
        const auto& theme = ui::theme::Current();

        observationsChart.SetAxisValues(results.timeAxis);
        observationsChart.SetPanels({
            {
                "Observations vs True Position",
                "Position",
                {
                    { "True Position", theme.Series(0), results.truePositions },
                    { "Noisy Measurements", theme.Series(1), results.rawMeasurements },
                },
                1,
            },
        });

        std::vector<float> emIterationAxis;
        emIterationAxis.reserve(results.emLogLikelihoodHistory.size());

        for (std::size_t i = 0; i < results.emLogLikelihoodHistory.size(); ++i)
            emIterationAxis.push_back(static_cast<float>(i + 1));

        emConvergenceChart.SetAxisValues(emIterationAxis);
        emConvergenceChart.SetPanels({
            {
                "EM Log-Likelihood vs Iteration",
                "Log-Likelihood",
                {
                    { "Log-Likelihood", theme.Series(2), results.emLogLikelihoodHistory },
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

        boConvergenceChart.SetAxisValues(boIterationAxis);
        boConvergenceChart.SetPanels({
            {
                "BO: Best ISE Found vs Evaluation",
                "ISE",
                {
                    { "All ISE values", theme.Series(4), results.boIseHistory },
                    { "Best so far", theme.Series(1), runningMinimum },
                },
                1,
            },
        });

        std::vector<float> referenceLine(results.stepResponseTime.size(), 1.0f);

        stepResponseChart.SetAxisValues(results.stepResponseTime);
        stepResponseChart.SetPanels({
            {
                "Position",
                "m",
                {
                    { "Position", theme.Series(0), results.stepResponsePosition },
                    { "Reference", theme.Get(ui::theme::ColorRole::TextMuted), referenceLine },
                },
                2,
            },
            {
                "Velocity",
                "m/s",
                {
                    { "Velocity", theme.Series(2), results.stepResponseVelocity },
                },
                1,
            },
            {
                "Control Input",
                "N",
                {
                    { "Control", theme.Series(1), results.stepResponseControl },
                },
                1,
            },
        });

        observationsView->update();
        emConvergenceView->update();
        boConvergenceView->update();
        stepResponseView->update();
    }
}
