#include "simulator/controllers/PidController/view/PidMainWindow.hpp"
#include "simulator/controllers/PidController/view/PidRootLocusWidget.hpp"
#include "simulator/shell/Guard.hpp"
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
    }

    PidMainWindow::PidMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , stepView(new ui::backend::qt::QtPaintedWidget{ stepChart, this })
        , rampView(new ui::backend::qt::QtPaintedWidget{ rampChart, this })
        , bodeView(new ui::backend::qt::QtPaintedWidget{ bodeChart, this })
        , rootLocusChart(new PidRootLocusWidget{ this })
        , dragTimer(new QTimer{ this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        stepView->SetPanCursorEnabled(true);
        rampView->SetPanCursorEnabled(true);
        bodeView->SetPanCursorEnabled(true);

        shell.SetPage(0, stepView);
        shell.SetPage(1, rampView);
        shell.SetPage(2, bodeView);
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

    void PidMainWindow::DisplayTimeResponse(ui::charts::ChartCore& chart, ui::backend::qt::QtPaintedWidget* view, const TimeResponse& result)
    {
        const auto& theme = ui::theme::Current();

        chart.SetAxisValues(result.time);
        chart.SetPanels({
            ui::charts::ChartPanel{ "Output", "Amplitude", {
                                                               ui::charts::Series{ "Reference", theme.Series(0), result.reference },
                                                               ui::charts::Series{ "Output", theme.Series(1), result.output },
                                                           },
                2 },
            ui::charts::ChartPanel{ "Control Signal", "u(t)", {
                                                                  ui::charts::Series{ "Control", theme.Series(2), result.controlSignal },
                                                              },
                1 },
            ui::charts::ChartPanel{ "Error", "e(t)", {
                                                         ui::charts::Series{ "Error", theme.Series(4), result.error },
                                                     },
                1 },
        });

        view->update();
    }

    void PidMainWindow::DisplayBodeResponse(ui::charts::ChartCore& chart, ui::backend::qt::QtPaintedWidget* view, const BodeResult& result)
    {
        const auto& theme = ui::theme::Current();

        chart.SetAxisValues(result.frequencies);
        chart.SetPanels({
            ui::charts::ChartPanel{ "Magnitude", "dB", {
                                                           ui::charts::Series{ "Magnitude", theme.Series(0), result.magnitudeDb },
                                                       },
                1 },
            ui::charts::ChartPanel{ "Phase", "degrees", {
                                                            ui::charts::Series{ "Phase", theme.Series(1), result.phaseDeg },
                                                        },
                1 },
        });

        view->update();
    }

    void PidMainWindow::RecomputeAndUpdateCharts()
    {
        auto config = form.BuildConfiguration();
        pidSimulator.Configure(form.CreatePlant(), config);

        DisplayTimeResponse(stepChart, stepView, pidSimulator.ComputeStepResponse());
        DisplayTimeResponse(rampChart, rampView, pidSimulator.ComputeRampResponse());
        DisplayBodeResponse(bodeChart, bodeView, pidSimulator.ComputeBodeResponse());

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
