#include "simulator/analysis/PowerDensitySpectrum/view/PsdMainWindow.hpp"
#include "simulator/widgets/FrequencyChartWidget.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include <QMessageBox>
#include <QSplitter>
#include <QTabWidget>
#include <stdexcept>

namespace simulator::analysis::psd::view
{
    PsdMainWindow::PsdMainWindow(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("PSD Simulator");
        resize(1024, 800);

        auto* splitter = new QSplitter(Qt::Horizontal, this);

        configPanel = new PsdConfigurationPanel(splitter);
        configPanel->setMaximumWidth(320);

        tabWidget = new QTabWidget(splitter);

        timeDomainChart = new widgets::TimeSeriesChartWidget(tabWidget);
        psdChart = new widgets::FrequencyChartWidget(tabWidget);

        tabWidget->addTab(timeDomainChart, "Time Domain");
        tabWidget->addTab(psdChart, "Power Spectral Density");

        splitter->addWidget(configPanel);
        splitter->addWidget(tabWidget);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);

        statusBar()->showMessage("Configure parameters and press Compute PSD");

        connect(configPanel, &PsdConfigurationPanel::ComputeRequested, this, &PsdMainWindow::OnComputeRequested);
    }

    void PsdMainWindow::OnComputeRequested()
    {
        auto config = configPanel->GetConfiguration();
        psdSimulator.Configure(config);

        try
        {
            auto result = psdSimulator.Compute();

            timeDomainChart->SetTimeAxis(result.time);
            timeDomainChart->SetPanels({
                {
                    "Input Signal",
                    "Amplitude",
                    {
                        { "Signal", QColor(41, 128, 185), result.signal },
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
                        { "PSD (dB)", QColor(231, 76, 60), result.powerDensityDb },
                    },
                    1,
                },
            });

            statusBar()->showMessage(
                QString("PSD computed: %1 input samples, %2-point segments, %3% overlap, %4 Hz")
                    .arg(config.inputSize)
                    .arg(config.segmentSize)
                    .arg(config.overlapPercent)
                    .arg(static_cast<double>(config.sampleRateHz), 0, 'f', 1));
        }
        catch (const std::exception& e)
        {
            QMessageBox::warning(this, "Computation Error", e.what());
            statusBar()->showMessage("Computation failed");
        }
    }
}
