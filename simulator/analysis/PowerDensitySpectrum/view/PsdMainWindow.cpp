#include "simulator/analysis/PowerDensitySpectrum/view/PsdMainWindow.hpp"
#include "simulator/shell/Guard.hpp"
#include "ui/theme/Theme.hpp"
#include <array>

namespace simulator::analysis::psd::view
{
    namespace
    {
        constexpr std::array<ui::shell::PageSpec, 2> pages{
            ui::shell::PageSpec{ "Time Domain" },
            ui::shell::PageSpec{ "Power Spectral Density" }
        };

        const ui::shell::ShellSpec shellSpec{
            "PSD Simulator",
            ui::Size{ 1024.0f, 800.0f },
            320.0f,
            pages,
            "Configure parameters and press Compute PSD"
        };
    }

    PsdMainWindow::PsdMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , timeDomainView(new ui::backend::qt::QtPaintedWidget{ timeDomainChart, this })
        , psdView(new ui::backend::qt::QtPaintedWidget{ psdChart, this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        timeDomainView->SetPanCursorEnabled(true);
        psdView->SetPanCursorEnabled(true);

        shell.SetPage(0, timeDomainView);
        shell.SetPage(1, psdView);

        form.Model().onActionTriggered = [this](ui::model::ActionId)
        {
            OnComputeRequested();
        };
    }

    void PsdMainWindow::OnComputeRequested()
    {
        shell::Guard(shell, "Computation Error", [this]
            {
                auto config = form.BuildConfiguration();
                psdSimulator.Configure(config);

                auto result = psdSimulator.Compute();

                const auto& theme = ui::theme::Current();

                timeDomainChart.SetAxisValues(result.time);
                timeDomainChart.SetPanels({
                    {
                        "Input Signal",
                        "Amplitude",
                        {
                            { "Signal", theme.Series(0), result.signal },
                        },
                        1,
                    },
                });

                psdChart.SetAxisValues(result.frequencies);
                psdChart.SetPanels({
                    {
                        "Power Spectral Density",
                        "Power (dB/Hz)",
                        {
                            { "PSD (dB)", theme.Series(1), result.powerDensityDb },
                        },
                        1,
                    },
                });

                timeDomainView->update();
                psdView->update();

                shell.SetStatus(QString("PSD computed: %1 input samples, %2-point segments, %3% overlap, %4 Hz")
                                    .arg(config.inputSize)
                                    .arg(config.segmentSize)
                                    .arg(config.overlapPercent)
                                    .arg(static_cast<double>(config.sampleRateHz), 0, 'f', 1)
                                    .toStdString());
            });
    }
}
