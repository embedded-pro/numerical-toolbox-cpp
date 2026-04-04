#include "simulator/controllers/Mpc/view/MpcMainWindow.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include <QMessageBox>
#include <QSplitter>
#include <exception>

namespace simulator::controllers::view
{
    MpcMainWindow::MpcMainWindow(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("MPC Controller Simulator");
        resize(1280, 900);

        auto* splitter = new QSplitter(Qt::Horizontal, this);

        configPanel = new MpcConfigurationPanel(splitter);
        configPanel->setMaximumWidth(350);

        tabWidget = new QTabWidget(splitter);

        stepChart = new widgets::TimeSeriesChartWidget(tabWidget);
        constrainedChart = new widgets::TimeSeriesChartWidget(tabWidget);

        tabWidget->addTab(stepChart, "Step Response");
        tabWidget->addTab(constrainedChart, "Constrained Response");

        splitter->addWidget(configPanel);
        splitter->addWidget(tabWidget);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);

        statusBar()->showMessage("Configure MPC parameters and press Compute");

        connect(configPanel, &MpcConfigurationPanel::ComputeRequested, this, &MpcMainWindow::OnComputeRequested);
    }

    void MpcMainWindow::OnComputeRequested()
    {
        try
        {
            auto config = configPanel->GetConfiguration();
            auto plant = configPanel->CreatePlant();
            mpcSimulator.Configure(plant, config);

            auto stepResult = mpcSimulator.ComputeStepResponse();
            DisplayResponse(stepChart, stepResult, config.referencePosition);

            auto constrainedResult = mpcSimulator.ComputeConstrainedResponse();
            DisplayResponse(constrainedChart, constrainedResult, config.referencePosition);

            statusBar()->showMessage(
                QString("MPC: Q=%1 R=%2 | Plant: %3")
                    .arg(static_cast<double>(config.weights.stateWeight), 0, 'f', 2)
                    .arg(static_cast<double>(config.weights.controlWeight), 0, 'f', 3)
                    .arg(configPanel->GetPlantDescription()));
        }
        catch (const std::exception& e)
        {
            QMessageBox::warning(this, "Computation Error", e.what());
            statusBar()->showMessage("Computation failed");
        }
    }

    void MpcMainWindow::DisplayResponse(widgets::TimeSeriesChartWidget* chart, const MpcTimeResponse& result, float referencePosition)
    {
        const QColor stateColors[] = {
            QColor(41, 128, 185),
            QColor(231, 76, 60),
            QColor(39, 174, 96),
            QColor(142, 68, 173),
        };

        std::vector<widgets::Series> stateSeries;
        for (std::size_t i = 0; i < result.states.size(); ++i)
        {
            stateSeries.push_back({
                QString("x%1").arg(i),
                stateColors[i % 4],
                result.states[i],
            });
        }

        std::vector<float> refLine(result.time.size(), referencePosition);
        stateSeries.push_back({ "Reference", QColor(100, 100, 100), refLine });

        chart->SetTimeAxis(result.time);
        chart->SetPanels({
            {
                "States",
                "Value",
                stateSeries,
                2,
            },
            {
                "Control Input",
                "u",
                {
                    { "Control", QColor(231, 76, 60), result.control },
                },
                1,
            },
            {
                "Cost",
                "J",
                {
                    { "Cost", QColor(142, 68, 173), result.cost },
                },
                1,
            },
        });
    }
}
