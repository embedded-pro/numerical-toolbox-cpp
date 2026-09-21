#include "simulator/controllers/Lqg/view/LqgMainWindow.hpp"
#include "simulator/shell/Guard.hpp"
#include "ui/theme/Theme.hpp"

namespace simulator::controllers::lqg::view
{
    namespace
    {
        const ui::shell::ShellSpec shellSpec{
            "LQG Controller Simulator",
            ui::Size{ 1280.0f, 800.0f },
            350.0f,
            {},
            "Configure LQG parameters and press Compute"
        };
    }

    LqgMainWindow::LqgMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , chartView(new ui::backend::qt::QtPaintedWidget{ chart, this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);
        chartView->SetPanCursorEnabled(true);
        shell.SetContent(chartView);

        form.Model().onActionTriggered = [this](ui::model::ActionId)
        {
            OnComputeRequested();
        };
    }

    void LqgMainWindow::OnComputeRequested()
    {
        shell::Guard(shell, "Computation Error", [this]
            {
                auto config = form.BuildConfiguration();
                auto plant = form.CreatePlant();
                lqgSimulator.Configure(plant, config);

                auto result = lqgSimulator.RunStepResponse();
                DisplayResponse(result);

                shell.SetStatus(QString("LQG: Q=%1 R=%2 | ProcessNoise=%3 MeasNoise=%4")
                        .arg(static_cast<double>(config.weights.stateWeight), 0, 'f', 2)
                        .arg(static_cast<double>(config.weights.controlWeight), 0, 'f', 3)
                        .arg(static_cast<double>(config.noise.processNoise), 0, 'f', 3)
                        .arg(static_cast<double>(config.noise.measurementNoise), 0, 'f', 3)
                        .toStdString());
            });
    }

    void LqgMainWindow::DisplayResponse(const LqgTimeResponse& result)
    {
        const auto& theme = ui::theme::Current();

        chart.SetAxisValues(result.time);
        chart.SetPanels({
            { "Position",
                "x [m]",
                {
                    { "True State", theme.Series(0), result.trueState },
                    { "Estimated State", theme.Series(1), result.estimatedState },
                } },
            { "Control Input",
                "u",
                {
                    { "Control", theme.Series(2), result.control },
                } },
        });

        chartView->update();
    }
}
