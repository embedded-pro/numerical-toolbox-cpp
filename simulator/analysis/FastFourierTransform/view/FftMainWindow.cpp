#include "simulator/analysis/FastFourierTransform/view/FftMainWindow.hpp"
#include <QHBoxLayout>
#include <QMessageBox>
#include <QSplitter>
#include <stdexcept>

namespace simulator::analysis::view
{
    FftMainWindow::FftMainWindow(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("FFT Simulator");
        resize(1024, 600);

        auto* splitter = new QSplitter(Qt::Horizontal, this);

        configPanel = new FftConfigurationPanel(splitter);
        configPanel->setMaximumWidth(320);

        chartWidget = new FftChartWidget(splitter);

        splitter->addWidget(configPanel);
        splitter->addWidget(chartWidget);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);

        statusBar()->showMessage("Configure parameters and press Compute FFT");

        connect(configPanel, &FftConfigurationPanel::ComputeRequested, this, &FftMainWindow::OnComputeRequested);
    }

    void FftMainWindow::OnComputeRequested()
    {
        auto config = configPanel->GetConfiguration();
        simulator.Configure(config);

        try
        {
            auto result = simulator.Compute();
            chartWidget->SetData(result);
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
