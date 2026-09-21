#include "simulator/controllers/Mpc/view/MpcMainWindow.hpp"
#include "simulator/shell/Guard.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include "ui/theme/Theme.hpp"
#include <array>

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

        [[nodiscard]] QColor ToQt(ui::Color color)
        {
            return QColor{ color.red, color.green, color.blue };
        }

        [[nodiscard]] QColor Series(std::size_t index)
        {
            return ToQt(ui::theme::Current().Series(index));
        }
    }

    MpcMainWindow::MpcMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , stepChart(new widgets::TimeSeriesChartWidget{ this })
        , constrainedChart(new widgets::TimeSeriesChartWidget{ this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        shell.SetPage(0, stepChart);
        shell.SetPage(1, constrainedChart);

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

                DisplayResponse(stepChart, mpcSimulator.ComputeStepResponse(), config.referencePosition);
                DisplayResponse(constrainedChart, mpcSimulator.ComputeConstrainedResponse(), config.referencePosition);

                const auto plantDescription = form.PlantDescription();

                shell.SetStatus(QString("MPC: Q=%1 R=%2 | Plant: %3")
                        .arg(static_cast<double>(config.weights.stateWeight), 0, 'f', 2)
                        .arg(static_cast<double>(config.weights.controlWeight), 0, 'f', 3)
                        .arg(QString::fromUtf8(plantDescription.data(), static_cast<qsizetype>(plantDescription.size())))
                        .toStdString());
            });
    }

    void MpcMainWindow::DisplayResponse(widgets::TimeSeriesChartWidget* chart, const MpcTimeResponse& result, float referencePosition)
    {
        std::vector<widgets::Series> stateSeries;

        for (std::size_t i = 0; i < result.states.size(); ++i)
            stateSeries.push_back({ QString("x%1").arg(i), Series(i % 4), result.states[i] });

        std::vector<float> referenceLine(result.time.size(), referencePosition);
        stateSeries.push_back({ "Reference", ToQt(ui::theme::Current().Get(ui::theme::ColorRole::TextMuted)), referenceLine });

        chart->SetTimeAxis(result.time);
        chart->SetPanels({
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
                    { "Control", Series(1), result.control },
                },
                1,
            },
            {
                "Cost",
                "J",
                {
                    { "Cost", Series(3), result.cost },
                },
                1,
            },
        });
    }
}
