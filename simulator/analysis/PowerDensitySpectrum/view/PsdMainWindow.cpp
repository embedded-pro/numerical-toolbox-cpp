#include "simulator/analysis/PowerDensitySpectrum/view/PsdMainWindow.hpp"
#include <QHBoxLayout>
#include <QMessageBox>
#include <QSplitter>
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

        chartWidget = new PsdChartWidget(splitter);

        splitter->addWidget(configPanel);
        splitter->addWidget(chartWidget);
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
            chartWidget->SetData(result);
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
