#include "simulator/controllers/Mpc/view/MpcMainWindow.hpp"
#include "simulator/shell/Guard.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <format>

namespace simulator::controllers::view
{
    namespace
    {
        constexpr std::array<ui::shell::PageSpec, 2> pages{
            ui::shell::PageSpec{ "Step Response" },
            ui::shell::PageSpec{ "Constrained Response" }
        };

        const ui::shell::ShellSpec shellSpec{
            "MPC Controller Simulator",
            ui::Size{ 1280.0f, 900.0f },
            350.0f,
            pages,
            "Configure MPC parameters and press Compute"
        };

    }

    MpcMainWindow::MpcMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , stepView(new ui::backend::qt::QtPaintedWidget{ stepChart, this })
        , constrainedView(new ui::backend::qt::QtPaintedWidget{ constrainedChart, this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        stepView->SetPanCursorEnabled(true);
        constrainedView->SetPanCursorEnabled(true);

        shell.SetPage(0, stepView);
        shell.SetPage(1, constrainedView);

        form.Model().onActionTriggered = [this](ui::model::ActionId)
        {
            OnComputeRequested();
        };
    }

    void MpcMainWindow::OnComputeRequested()
    {
        shell::Guard(shell, "Computation Error", [this]
            {
                auto config = form.BuildConfiguration();
                auto plant = form.CreatePlant();
                mpcSimulator.Configure(plant, config);

                DisplayResponse(stepChart, stepView, mpcSimulator.ComputeStepResponse(), config.referencePosition);
                DisplayResponse(constrainedChart, constrainedView, mpcSimulator.ComputeConstrainedResponse(), config.referencePosition);

                const auto plantDescription = form.PlantDescription();

                shell.SetStatus(QString("MPC: Q=%1 R=%2 | Plant: %3")
                                    .arg(static_cast<double>(config.weights.stateWeight), 0, 'f', 2)
                                    .arg(static_cast<double>(config.weights.controlWeight), 0, 'f', 3)
                                    .arg(QString::fromUtf8(plantDescription.data(), static_cast<qsizetype>(plantDescription.size())))
                                    .toStdString());
            });
    }

    void MpcMainWindow::DisplayResponse(ui::charts::ChartCore& chart, ui::backend::qt::QtPaintedWidget* view, const MpcTimeResponse& result, float referencePosition)
    {
        const auto& theme = ui::theme::Current();

        std::vector<ui::charts::Series> stateSeries;

        for (std::size_t i = 0; i < result.states.size(); ++i)
            stateSeries.push_back({ std::format("x{}", i), theme.Series(i % 4), result.states[i] });

        std::vector<float> referenceLine(result.time.size(), referencePosition);
        stateSeries.push_back({ "Reference", theme.Get(ui::theme::ColorRole::TextMuted), referenceLine });

        chart.SetAxisValues(result.time);
        chart.SetPanels({
            {
                "States",
                "Value",
                stateSeries,
                2,
            },
            {
                "Control Input",
                "u",
                {
                    { "Control", theme.Series(1), result.control },
                },
                1,
            },
            {
                "Cost",
                "J",
                {
                    { "Cost", theme.Series(3), result.cost },
                },
                1,
            },
        });

        view->update();
    }
}
