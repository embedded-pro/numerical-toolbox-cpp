#include "simulator/filters/FirFilter/view/FirMainWindow.hpp"
#include "simulator/shell/Guard.hpp"
#include "ui/theme/Theme.hpp"
#include <array>

namespace simulator::filters::fir::view
{
    namespace
    {
        constexpr std::array<ui::shell::PageSpec, 3> pages{
            ui::shell::PageSpec{ "Time Domain" },
            ui::shell::PageSpec{ "Frequency Response" },
            ui::shell::PageSpec{ "Impulse Response" }
        };

        const ui::shell::ShellSpec shellSpec{
            "FIR Filter Simulator",
            ui::Size{ 1200.0f, 700.0f },
            350.0f,
            pages,
            "Configure filter parameters and press Compute"
        };
    }

    FirMainWindow::FirMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , timeDomainView(new ui::backend::qt::QtPaintedWidget{ timeDomainChart, this })
        , frequencyView(new ui::backend::qt::QtPaintedWidget{ frequencyChart, this })
        , impulseView(new ui::backend::qt::QtPaintedWidget{ impulseChart, this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        timeDomainView->SetPanCursorEnabled(true);
        frequencyView->SetPanCursorEnabled(true);
        impulseView->SetPanCursorEnabled(true);

        shell.SetPage(0, timeDomainView);
        shell.SetPage(1, frequencyView);
        shell.SetPage(2, impulseView);

        form.Model().onActionTriggered = [this](ui::model::ActionId)
        {
            OnComputeRequested();
        };
    }

    void FirMainWindow::OnComputeRequested()
    {
        shell::Guard(shell, "Computation Error", [this]
            {
                auto config = form.BuildConfiguration();

                FirFilterSimulator simulator;
                simulator.Configure(config);
                auto result = simulator.Run();

                const auto& theme = ui::theme::Current();

                timeDomainChart.SetAxisValues(result.time);
                timeDomainChart.SetPanels({
                    {
                        "Input vs Output",
                        "Amplitude",
                        {
                            { "Input", theme.Series(0), result.inputSignal },
                            { "Output", theme.Series(1), result.outputSignal },
                        },
                        1,
                    },
                });

                frequencyChart.SetAxisValues(result.frequencies);
                frequencyChart.SetPanels({
                    {
                        "Frequency Spectrum (dB)",
                        "Magnitude (dB)",
                        {
                            { "Input", theme.Series(0), result.inputMagnitudeDb },
                            { "Output", theme.Series(1), result.outputMagnitudeDb },
                        },
                        1,
                    },
                });

                impulseChart.SetAxisValues(result.impulseSampleIndex);
                impulseChart.SetPanels({
                    {
                        "Impulse Response (Filter Coefficients)",
                        "Amplitude",
                        {
                            { "h[n]", theme.Series(2), result.impulseResponse },
                        },
                        1,
                    },
                });

                timeDomainView->update();
                frequencyView->update();
                impulseView->update();

                shell.SetStatus(QString("FIR filter computed: order %1, cutoff %2 Hz")
                        .arg(config.filter.order)
                        .arg(static_cast<double>(config.filter.cutoffHz), 0, 'f', 1)
                        .toStdString());
            });
    }
}
