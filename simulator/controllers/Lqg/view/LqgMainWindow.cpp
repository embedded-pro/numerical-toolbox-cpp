#include "simulator/controllers/Lqg/view/LqgMainWindow.hpp"
#include "simulator/widgets/TimeSeriesChartWidget.hpp"
#include <QSplitter>

namespace simulator::controllers::lqg::view
{
    LqgMainWindow::LqgMainWindow(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("LQG Controller Simulator");
        resize(1280, 800);

        auto* splitter = new QSplitter(Qt::Horizontal, this);

        configPanel = new LqgConfigurationPanel(splitter);
        configPanel->setMaximumWidth(350);

        chart = new widgets::TimeSeriesChartWidget(splitter);

        splitter->addWidget(configPanel);
        splitter->addWidget(chart);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);

        statusBar()->showMessage("Configure LQG parameters and press Compute");

        connect(configPanel, &LqgConfigurationPanel::ComputeRequested, this, &LqgMainWindow::OnComputeRequested);
    }

    void LqgMainWindow::OnComputeRequested()
    {
        auto config = configPanel->GetConfiguration();
        auto plant = configPanel->CreatePlant();
        lqgSimulator.Configure(plant, config);

        auto result = lqgSimulator.RunStepResponse();
        DisplayResponse(result);

        statusBar()->showMessage(
            QString("LQG: Q=%1 R=%2 | ProcessNoise=%3 MeasNoise=%4")
                .arg(static_cast<double>(config.weights.stateWeight), 0, 'f', 2)
                .arg(static_cast<double>(config.weights.controlWeight), 0, 'f', 3)
                .arg(static_cast<double>(config.noise.processNoise), 0, 'f', 3)
                .arg(static_cast<double>(config.noise.measurementNoise), 0, 'f', 3));
    }

    void LqgMainWindow::DisplayResponse(const LqgTimeResponse& result)
    {
        chart->SetTimeAxis(result.time);
        chart->SetPanels({
            { "Position",
                "x [m]",
                {
                    { "True State", QColor(41, 128, 185), result.trueState },
                    { "Estimated State", QColor(231, 76, 60), result.estimatedState },
                } },
            { "Control Input",
                "u",
                {
                    { "Control", QColor(39, 174, 96), result.control },
                } },
        });
    }
}
