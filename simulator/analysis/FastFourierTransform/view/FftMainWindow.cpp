#include "simulator/analysis/FastFourierTransform/view/FftMainWindow.hpp"
#include "ui/theme/Theme.hpp"
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

        timeDomainView = new ui::backend::qt::QtPaintedWidget(timeDomainChart, tabWidget);
        frequencyView = new ui::backend::qt::QtPaintedWidget(frequencyChart, tabWidget);

        timeDomainView->SetPanCursorEnabled(true);
        frequencyView->SetPanCursorEnabled(true);

        tabWidget->addTab(timeDomainView, "Time Domain");
        tabWidget->addTab(frequencyView, "Frequency Spectrum");

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
