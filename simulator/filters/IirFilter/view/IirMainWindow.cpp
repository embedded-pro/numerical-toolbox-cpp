#include "simulator/filters/IirFilter/view/IirMainWindow.hpp"
#include "simulator/shell/Guard.hpp"
#include "simulator/widgets/FrequencyChartWidget.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include "ui/theme/Theme.hpp"
#include <array>

namespace simulator::filters::iir::view
{
    namespace
    {
        constexpr std::array<ui::shell::PageSpec, 3> pages{
            ui::shell::PageSpec{ "Time Domain" },
            ui::shell::PageSpec{ "Frequency Response" },
            ui::shell::PageSpec{ "Impulse Response" }
        };

        const ui::shell::ShellSpec shellSpec{
            "IIR Filter Simulator",
            ui::Size{ 1200.0f, 700.0f },
            350.0f,
            pages,
            "Configure filter parameters and press Compute"
        };

        [[nodiscard]] QColor Series(std::size_t index)
        {
            const auto color = ui::theme::Current().Series(index);
            return QColor{ color.red, color.green, color.blue };
        }
    }

    IirMainWindow::IirMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , timeDomainChart(new widgets::TimeSeriesChartWidget{ this })
        , frequencyChart(new widgets::FrequencyChartWidget{ this })
        , impulseChart(new widgets::TimeSeriesChartWidget{ this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        shell.SetPage(0, timeDomainChart);
        shell.SetPage(1, frequencyChart);
        shell.SetPage(2, impulseChart);

        form.Model().onActionTriggered = [this](ui::model::ActionId)
        {
            OnComputeRequested();
        };
    }

    void IirMainWindow::OnComputeRequested()
    {
        shell::Guard(shell, "Computation Error", [this]
            {
                auto config = form.BuildConfiguration();

                IirFilterSimulator simulator;
                simulator.Configure(config);
                auto result = simulator.Run();

                timeDomainChart->SetTimeAxis(result.time);
                timeDomainChart->SetPanels({
                    {
                        "Input vs Output",
                        "Amplitude",
                        {
                            { "Input", Series(0), result.inputSignal },
                            { "Output", Series(1), result.outputSignal },
                        },
                        1,
                    },
                });

                frequencyChart->SetFrequencyAxis(result.frequencies);
                frequencyChart->SetPanels({
                    {
                        "Frequency Spectrum (dB)",
                        "Magnitude (dB)",
                        {
                            { "Input", Series(0), result.inputMagnitudeDb },
                            { "Output", Series(1), result.outputMagnitudeDb },
                        },
                        1,
                    },
                });

                impulseChart->SetTimeAxis(result.impulseSampleIndex);
                impulseChart->SetPanels({
                    {
                        "Impulse Response",
                        "Amplitude",
                        {
                            { "h[n]", Series(2), result.impulseResponse },
                        },
                        1,
                    },
                });

                shell.SetStatus(QString("IIR filter computed: cutoff %1 Hz, Q = %2")
                                    .arg(static_cast<double>(config.filter.cutoffHz), 0, 'f', 1)
                                    .arg(static_cast<double>(config.filter.qualityFactor), 0, 'f', 3)
                                    .toStdString());
            });
    }
}
