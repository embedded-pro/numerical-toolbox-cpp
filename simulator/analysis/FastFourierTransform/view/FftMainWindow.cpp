#include "simulator/analysis/FastFourierTransform/view/FftMainWindow.hpp"
#include "simulator/widgets/FrequencyChartWidget.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include <QMessageBox>
#include <QSplitter>
#include <QTabWidget>
#include <stdexcept>

namespace simulator::analysis::view
{
    FftMainWindow::FftMainWindow(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("FFT Simulator");
        resize(1024, 800);

        auto* splitter = new QSplitter(Qt::Horizontal, this);

        configPanel = new FftConfigurationPanel(splitter);
        configPanel->setMaximumWidth(320);

        tabWidget = new QTabWidget(splitter);

        timeDomainChart = new widgets::TimeSeriesChartWidget(tabWidget);
        frequencyChart = new widgets::FrequencyChartWidget(tabWidget);

        tabWidget->addTab(timeDomainChart, "Time Domain");
        tabWidget->addTab(frequencyChart, "Frequency Spectrum");

        splitter->addWidget(configPanel);
        splitter->addWidget(tabWidget);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);

        statusBar()->showMessage("Configure parameters and press Compute FFT");

        connect(configPanel, &FftConfigurationPanel::ComputeRequested, this, &FftMainWindow::OnComputeRequested);
    }

    void FftMainWindow::OnComputeRequested()
    {
        auto config = configPanel->GetConfiguration();
        fftSimulator.Configure(config);

        try
        {
            auto result = fftSimulator.Compute();

            timeDomainChart->SetTimeAxis(result.time);
            timeDomainChart->SetPanels({
                {
                    "Input Signal",
                    "Amplitude",
                    {
                        { "Signal", QColor(41, 128, 185), result.signal },
                        { "Windowed", QColor(231, 76, 60), result.windowedSignal },
                    },
                    1,
                },
            });

            frequencyChart->SetFrequencyAxis(result.frequencies);
            frequencyChart->SetPanels({
                {
                    "FFT Magnitude",
                    "Magnitude",
                    {
                        { "Magnitude", QColor(41, 128, 185), result.magnitudes },
                    },
                    1,
                },
            });

            statusBar()->showMessage(
                QString("FFT computed: %1 points, sample rate %2 Hz")
                    .arg(config.fftSize)
                    .arg(static_cast<double>(config.sampleRateHz), 0, 'f', 1));
        }
        catch (const std::exception& e)
        {
            QMessageBox::warning(this, "Computation Error", e.what());
            statusBar()->showMessage("Computation failed");
        }
    }
}
