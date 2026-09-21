#include "simulator/analysis/FastFourierTransform/view/FftMainWindow.hpp"
#include "simulator/shell/Guard.hpp"
#include "ui/theme/Theme.hpp"
#include <array>

namespace simulator::analysis::view
{
    namespace
    {
        constexpr std::array<ui::shell::PageSpec, 2> pages{
            ui::shell::PageSpec{ "Time Domain" },
            ui::shell::PageSpec{ "Frequency Spectrum" }
        };

        const ui::shell::ShellSpec shellSpec{
            "FFT Simulator",
            ui::Size{ 1024.0f, 800.0f },
            320.0f,
            pages,
            "Configure parameters and press Compute FFT"
        };
    }

    FftMainWindow::FftMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , timeDomainView(new ui::backend::qt::QtPaintedWidget{ timeDomainChart, this })
        , frequencyView(new ui::backend::qt::QtPaintedWidget{ frequencyChart, this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        timeDomainView->SetPanCursorEnabled(true);
        frequencyView->SetPanCursorEnabled(true);

        shell.SetPage(0, timeDomainView);
        shell.SetPage(1, frequencyView);

        form.Model().onActionTriggered = [this](ui::model::ActionId)
        {
            OnComputeRequested();
        };
    }

    void FftMainWindow::OnComputeRequested()
    {
        shell::Guard(shell, "Computation Error", [this]
            {
                auto config = form.BuildConfiguration();
                fftSimulator.Configure(config);

                auto result = fftSimulator.Compute();
                const auto& theme = ui::theme::Current();

                timeDomainChart.SetAxisValues(result.time);
                timeDomainChart.SetPanels({
                    {
                        "Input Signal",
                        "Amplitude",
                        {
                            { "Signal", theme.Series(0), result.signal },
                            { "Windowed", theme.Series(1), result.windowedSignal },
                        },
                        1,
                    },
                });

                frequencyChart.SetAxisValues(result.frequencies);
                frequencyChart.SetPanels({
                    {
                        "FFT Magnitude",
                        "Magnitude",
                        {
                            { "Magnitude", theme.Series(0), result.magnitudes },
                        },
                        1,
                    },
                });

                timeDomainView->update();
                frequencyView->update();

                shell.SetStatus(QString("FFT computed: %1 points, sample rate %2 Hz")
                                    .arg(config.fftSize)
                                    .arg(static_cast<double>(config.sampleRateHz), 0, 'f', 1)
                                    .toStdString());
            });
    }
}
