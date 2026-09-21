#include "simulator/controllers/Lqg/view/LqgMainWindow.hpp"
#include "simulator/shell/Guard.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
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

        [[nodiscard]] QColor Series(std::size_t index)
        {
            const auto color = ui::theme::Current().Series(index);
            return QColor{ color.red, color.green, color.blue };
        }
    }

    LqgMainWindow::LqgMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , chart(new widgets::TimeSeriesChartWidget{ this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);
        shell.SetContent(chart);

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
        chart->SetTimeAxis(result.time);
        chart->SetPanels({
            { "Position",
                "x [m]",
                {
                    { "True State", Series(0), result.trueState },
                    { "Estimated State", Series(1), result.estimatedState },
                } },
            { "Control Input",
                "u",
                {
                    { "Control", Series(2), result.control },
                } },
        });
    }
}
