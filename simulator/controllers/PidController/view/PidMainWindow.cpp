#include "simulator/controllers/PidController/view/PidMainWindow.hpp"
#include "simulator/controllers/PidController/view/PidRootLocusWidget.hpp"
#include "simulator/shell/Guard.hpp"
#include "simulator/widgets/FrequencyChartWidget.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include "ui/theme/Theme.hpp"
#include <array>

namespace simulator::controllers::view
{
    namespace
    {
        constexpr std::array<ui::shell::PageSpec, 4> pages{
            ui::shell::PageSpec{ "Step Response" },
            ui::shell::PageSpec{ "Ramp Response" },
            ui::shell::PageSpec{ "Bode Plot" },
            ui::shell::PageSpec{ "Root Locus" }
        };

        const ui::shell::ShellSpec shellSpec{
            "PID Controller Simulator",
            ui::Size{ 1280.0f, 900.0f },
            350.0f,
            pages,
            "Configure PID parameters and press Compute"
        };

        [[nodiscard]] QColor Series(std::size_t index)
        {
            const auto color = ui::theme::Current().Series(index);
            return QColor{ color.red, color.green, color.blue };
        }
    }

    PidMainWindow::PidMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , stepChart(new widgets::TimeSeriesChartWidget{ this })
        , rampChart(new widgets::TimeSeriesChartWidget{ this })
        , bodeChart(new widgets::FrequencyChartWidget{ this })
        , rootLocusChart(new PidRootLocusWidget{ this })
        , dragTimer(new QTimer{ this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        shell.SetPage(0, stepChart);
        shell.SetPage(1, rampChart);
        shell.SetPage(2, bodeChart);
        shell.SetPage(3, rootLocusChart);

        dragTimer->setSingleShot(true);
        dragTimer->setInterval(30);
        connect(dragTimer, &QTimer::timeout, this, &PidMainWindow::RecomputeAndUpdateCharts);

        connect(rootLocusChart, &PidRootLocusWidget::PoleGainChanged, this, &PidMainWindow::OnPoleGainChanged);

        form.Model().onActionTriggered = [this](ui::model::ActionId)
        {
            OnComputeRequested();
        };
    }

    void PidMainWindow::DisplayTimeResponse(widgets::TimeSeriesChartWidget* chart, const TimeResponse& result)
    {
        chart->SetTimeAxis(result.time);
        chart->SetPanels({
            widgets::ChartPanel{ "Output", "Amplitude", {
                                                            widgets::Series{ "Reference", Series(0), result.reference },
                                                            widgets::Series{ "Output", Series(1), result.output },
                                                        },
                2 },
            widgets::ChartPanel{ "Control Signal", "u(t)", {
                                                               widgets::Series{ "Control", Series(2), result.controlSignal },
                                                           },
                1 },
            widgets::ChartPanel{ "Error", "e(t)", {
                                                      widgets::Series{ "Error", Series(4), result.error },
                                                  },
                1 },
        });
    }

    void PidMainWindow::DisplayBodeResponse(widgets::FrequencyChartWidget* chart, const BodeResult& result)
    {
        chart->SetFrequencyAxis(result.frequencies);
        chart->SetPanels({
            widgets::ChartPanel{ "Magnitude", "dB", {
                                                        widgets::Series{ "Magnitude", Series(0), result.magnitudeDb },
                                                    },
                1 },
            widgets::ChartPanel{ "Phase", "degrees", {
                                                         widgets::Series{ "Phase", Series(1), result.phaseDeg },
                                                     },
                1 },
        });
    }

    void PidMainWindow::RecomputeAndUpdateCharts()
    {
        auto config = form.BuildConfiguration();
        pidSimulator.Configure(form.CreatePlant(), config);

        DisplayTimeResponse(stepChart, pidSimulator.ComputeStepResponse());
        DisplayTimeResponse(rampChart, pidSimulator.ComputeRampResponse());
        DisplayBodeResponse(bodeChart, pidSimulator.ComputeBodeResponse());

        rootLocusChart->SetData(pidSimulator.ComputeRootLocus());
    }

    void PidMainWindow::OnComputeRequested()
    {
        shell::Guard(shell, "Computation Error", [this]
            {
                RecomputeAndUpdateCharts();

                const auto config = form.BuildConfiguration();
                const auto plant = form.PlantDescription();

                shell.SetStatus(QString("PID: Kp=%1 Ki=%2 Kd=%3 | Plant: %4")
                        .arg(static_cast<double>(config.tunings.kp), 0, 'f', 3)
                        .arg(static_cast<double>(config.tunings.ki), 0, 'f', 3)
                        .arg(static_cast<double>(config.tunings.kd), 0, 'f', 3)
                        .arg(QString::fromUtf8(plant.data(), static_cast<qsizetype>(plant.size())))
                        .toStdString());
            });
    }

    void PidMainWindow::OnPoleGainChanged(float gain)
    {
        form.SetProportionalGain(gain);
        formView->Refresh(pid::field::kp);

        if (!dragTimer->isActive())
            dragTimer->start();
    }
}
