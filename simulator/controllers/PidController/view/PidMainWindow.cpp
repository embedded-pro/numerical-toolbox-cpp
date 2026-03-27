#include "simulator/controllers/PidController/view/PidMainWindow.hpp"
#include "simulator/controllers/PidController/view/PidRootLocusWidget.hpp"
#include "simulator/widgets/FrequencyChartWidget.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include <QHBoxLayout>
#include <QMessageBox>
#include <QSplitter>
#include <stdexcept>

namespace simulator::controllers::view
{
    PidMainWindow::PidMainWindow(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("PID Controller Simulator");
        resize(1280, 900);

        auto* splitter = new QSplitter(Qt::Horizontal, this);

        configPanel = new PidConfigurationPanel(splitter);
        configPanel->setMaximumWidth(350);

        tabWidget = new QTabWidget(splitter);

        stepChart = new widgets::TimeSeriesChartWidget(tabWidget);
        rampChart = new widgets::TimeSeriesChartWidget(tabWidget);
        bodeChart = new widgets::FrequencyChartWidget(tabWidget);
        rootLocusChart = new PidRootLocusWidget(tabWidget);

        tabWidget->addTab(stepChart, "Step Response");
        tabWidget->addTab(rampChart, "Ramp Response");
        tabWidget->addTab(bodeChart, "Bode Plot");
        tabWidget->addTab(rootLocusChart, "Root Locus");

        splitter->addWidget(configPanel);
        splitter->addWidget(tabWidget);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);

        statusBar()->showMessage("Configure PID parameters and press Compute");

        dragTimer = new QTimer(this);
        dragTimer->setSingleShot(true);
        dragTimer->setInterval(30);
        connect(dragTimer, &QTimer::timeout, this, &PidMainWindow::RecomputeAndUpdateCharts);

        connect(configPanel, &PidConfigurationPanel::ComputeRequested, this, &PidMainWindow::OnComputeRequested);
        connect(rootLocusChart, &PidRootLocusWidget::PoleGainChanged, this, &PidMainWindow::OnPoleGainChanged);
    }

    void PidMainWindow::DisplayTimeResponse(widgets::TimeSeriesChartWidget* chart, const TimeResponse& result)
    {
        chart->SetTimeAxis(result.time);
        chart->SetPanels({
            widgets::ChartPanel{ "Output", "Amplitude", {
                                                            widgets::Series{ "Reference", QColor(41, 128, 185), result.reference },
                                                            widgets::Series{ "Output", QColor(231, 76, 60), result.output },
                                                        },
                2 },
            widgets::ChartPanel{ "Control Signal", "u(t)", {
                                                               widgets::Series{ "Control", QColor(39, 174, 96), result.controlSignal },
                                                           },
                1 },
            widgets::ChartPanel{ "Error", "e(t)", {
                                                      widgets::Series{ "Error", QColor(243, 156, 18), result.error },
                                                  },
                1 },
        });
    }

    void PidMainWindow::DisplayBodeResponse(widgets::FrequencyChartWidget* chart, const BodeResult& result)
    {
        chart->SetFrequencyAxis(result.frequencies);
        chart->SetPanels({
            widgets::ChartPanel{ "Magnitude", "dB", {
                                                        widgets::Series{ "Magnitude", QColor(41, 128, 185), result.magnitudeDb },
                                                    },
                1 },
            widgets::ChartPanel{ "Phase", "degrees", {
                                                         widgets::Series{ "Phase", QColor(231, 76, 60), result.phaseDeg },
                                                     },
                1 },
        });
    }

    void PidMainWindow::RecomputeAndUpdateCharts()
    {
        auto config = configPanel->GetConfiguration();
        pidSimulator.Configure(configPanel->CreatePlant(), config);

        DisplayTimeResponse(stepChart, pidSimulator.ComputeStepResponse());
        DisplayTimeResponse(rampChart, pidSimulator.ComputeRampResponse());
        DisplayBodeResponse(bodeChart, pidSimulator.ComputeBodeResponse());

        auto rootLocusResult = pidSimulator.ComputeRootLocus();
        rootLocusChart->SetData(rootLocusResult);
    }

    void PidMainWindow::OnComputeRequested()
    {
        try
        {
            RecomputeAndUpdateCharts();

            auto config = configPanel->GetConfiguration();
            statusBar()->showMessage(
                QString("PID: Kp=%1 Ki=%2 Kd=%3 | Plant: %4")
                    .arg(static_cast<double>(config.tunings.kp), 0, 'f', 3)
                    .arg(static_cast<double>(config.tunings.ki), 0, 'f', 3)
                    .arg(static_cast<double>(config.tunings.kd), 0, 'f', 3)
                    .arg(configPanel->GetPlantDescription()));
        }
        catch (const std::exception& e)
        {
            QMessageBox::warning(this, "Computation Error", e.what());
            statusBar()->showMessage("Computation failed");
        }
    }

    void PidMainWindow::OnPoleGainChanged(float gain)
    {
        pendingGain = gain;
        configPanel->SetKp(gain);

        if (!dragTimer->isActive())
            dragTimer->start();
    }
}
