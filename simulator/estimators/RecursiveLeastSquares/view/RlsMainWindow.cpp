#include "simulator/estimators/RecursiveLeastSquares/view/RlsMainWindow.hpp"
#include "simulator/estimators/RecursiveLeastSquares/view/RlsConfigurationPanel.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>

namespace simulator::estimators::rls::view
{
    RlsMainWindow::RlsMainWindow(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("Recursive Least Squares Simulator");
        resize(1200, 700);

        auto* splitter = new QSplitter(Qt::Horizontal, this);

        configPanel = new RlsConfigurationPanel(splitter);
        configPanel->setMaximumWidth(350);

        tabWidget = new QTabWidget(splitter);

        outputChart = new widgets::TimeSeriesChartWidget(tabWidget);
        coefficientChart = new widgets::TimeSeriesChartWidget(tabWidget);
        metricsChart = new widgets::TimeSeriesChartWidget(tabWidget);

        tabWidget->addTab(outputChart, "Output Tracking");
        tabWidget->addTab(coefficientChart, "Coefficient Convergence");
        tabWidget->addTab(metricsChart, "Estimation Metrics");

        splitter->addWidget(configPanel);
        splitter->addWidget(tabWidget);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);
        statusBar()->showMessage("Configure RLS parameters and press Compute");

        connect(configPanel, &RlsConfigurationPanel::ComputeRequested, this, &RlsMainWindow::OnComputeRequested);
    }

    void RlsMainWindow::OnComputeRequested()
    {
        auto config = configPanel->GetConfiguration();

        RlsSimulator simulator;
        simulator.Configure(config);
        auto result = simulator.Run();

        outputChart->SetTimeAxis(result.sampleIndex);
        outputChart->SetPanels({
            {
                "True vs Estimated Output",
                "Value",
                {
                    { "True", QColor(41, 128, 185), result.trueOutput },
                    { "Estimated", QColor(231, 76, 60), result.estimatedOutput },
                },
                1,
            },
        });

        std::vector<widgets::Series> coeffSeries;
        const QColor coeffColors[] = {
            QColor(41, 128, 185),
            QColor(231, 76, 60),
            QColor(39, 174, 96),
            QColor(142, 68, 173),
        };
        for (std::size_t i = 0; i < result.coefficientHistory.size(); ++i)
        {
            auto trueVal = (i < config.rls.trueCoefficients.size()) ? config.rls.trueCoefficients[i] : 0.0f;
            coeffSeries.push_back({
                QString("θ%1 (true=%2)").arg(i).arg(static_cast<double>(trueVal), 0, 'f', 2),
                coeffColors[i % 4],
                result.coefficientHistory[i],
            });
        }

        coefficientChart->SetTimeAxis(result.sampleIndex);
        coefficientChart->SetPanels({
            {
                "Coefficient Convergence",
                "Coefficient Value",
                coeffSeries,
                1,
            },
        });

        metricsChart->SetTimeAxis(result.sampleIndex);
        metricsChart->SetPanels({
            {
                "Innovation (pre-update error)",
                "Error",
                {
                    { "Innovation", QColor(231, 76, 60), result.innovationHistory },
                },
                1,
            },
            {
                "Uncertainty (trace of P)",
                "Trace(P)",
                {
                    { "Uncertainty", QColor(142, 68, 173), result.uncertaintyHistory },
                },
                1,
            },
        });

        statusBar()->showMessage(QString("RLS estimation complete: %1 samples, λ=%2")
                                     .arg(config.rls.numSamples)
                                     .arg(static_cast<double>(config.rls.forgettingFactor), 0, 'f', 3));
    }
}
