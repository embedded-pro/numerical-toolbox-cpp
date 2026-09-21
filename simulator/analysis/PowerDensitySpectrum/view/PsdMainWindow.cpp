#include "simulator/analysis/PowerDensitySpectrum/view/PsdMainWindow.hpp"
#include "simulator/shell/Guard.hpp"
#include "simulator/widgets/FrequencyChartWidget.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
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

        [[nodiscard]] QColor Series(std::size_t index)
        {
            const auto color = ui::theme::Current().Series(index);
            return QColor{ color.red, color.green, color.blue };
        }
    }

    PsdMainWindow::PsdMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , timeDomainChart(new widgets::TimeSeriesChartWidget{ this })
        , psdChart(new widgets::FrequencyChartWidget{ this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        shell.SetPage(0, timeDomainChart);
        shell.SetPage(1, psdChart);

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

                timeDomainChart->SetTimeAxis(result.time);
                timeDomainChart->SetPanels({
                    {
                        "Input Signal",
                        "Amplitude",
                        {
                            { "Signal", Series(0), result.signal },
                        },
                        1,
                    },
                });

                psdChart->SetFrequencyAxis(result.frequencies);
                psdChart->SetPanels({
                    {
                        "Power Spectral Density",
                        "Power (dB/Hz)",
                        {
                            { "PSD (dB)", Series(1), result.powerDensityDb },
                        },
                        1,
                    },
                });

                shell.SetStatus(QString("PSD computed: %1 input samples, %2-point segments, %3% overlap, %4 Hz")
                        .arg(config.inputSize)
                        .arg(config.segmentSize)
                        .arg(config.overlapPercent)
                        .arg(static_cast<double>(config.sampleRateHz), 0, 'f', 1)
                        .toStdString());
            });
    }
}
