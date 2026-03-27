#include "simulator/filters/FirFilter/view/FirMainWindow.hpp"
#include "simulator/filters/FirFilter/view/FirConfigurationPanel.hpp"
#include "simulator/widgets/FrequencyChartWidget.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>

namespace simulator::filters::fir::view
{
    FirMainWindow::FirMainWindow(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("FIR Filter Simulator");
        resize(1200, 700);

        auto* splitter = new QSplitter(Qt::Horizontal, this);

        configPanel = new FirConfigurationPanel(splitter);
        configPanel->setMaximumWidth(350);

        tabWidget = new QTabWidget(splitter);

        timeDomainChart = new widgets::TimeSeriesChartWidget(tabWidget);
        frequencyChart = new widgets::FrequencyChartWidget(tabWidget);
        impulseChart = new widgets::TimeSeriesChartWidget(tabWidget);

        tabWidget->addTab(timeDomainChart, "Time Domain");
        tabWidget->addTab(frequencyChart, "Frequency Response");
        tabWidget->addTab(impulseChart, "Impulse Response");

        splitter->addWidget(configPanel);
        splitter->addWidget(tabWidget);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);
        statusBar()->showMessage("Configure filter parameters and press Compute");

        connect(configPanel, &FirConfigurationPanel::ComputeRequested, this, &FirMainWindow::OnComputeRequested);
    }

    void FirMainWindow::OnComputeRequested()
    {
        auto config = configPanel->GetConfiguration();

        FirFilterSimulator simulator;
        simulator.Configure(config);
        auto result = simulator.Run();

        timeDomainChart->SetTimeAxis(result.time);
        timeDomainChart->SetPanels({
            {
                "Input vs Output",
                "Amplitude",
                {
                    { "Input", QColor(41, 128, 185), result.inputSignal },
                    { "Output", QColor(231, 76, 60), result.outputSignal },
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
                    { "Input", QColor(41, 128, 185), result.inputMagnitudeDb },
                    { "Output", QColor(231, 76, 60), result.outputMagnitudeDb },
                },
                1,
            },
        });

        impulseChart->SetTimeAxis(result.impulseSampleIndex);
        impulseChart->SetPanels({
            {
                "Impulse Response (Filter Coefficients)",
                "Amplitude",
                {
                    { "h[n]", QColor(39, 174, 96), result.impulseResponse },
                },
                1,
            },
        });

        statusBar()->showMessage(QString("FIR filter computed: order %1, cutoff %2 Hz")
                                     .arg(config.filter.order)
                                     .arg(static_cast<double>(config.filter.cutoffHz), 0, 'f', 1));
    }
}
