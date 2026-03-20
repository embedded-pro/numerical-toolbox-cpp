#include "simulator/controllers/LqrCartPole/view/LqrMainWindow.hpp"
#include <QSplitter>

namespace simulator::controllers::lqr::view
{
    LqrMainWindow::LqrMainWindow(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("LQR Cart-Pole Simulator");
        resize(1280, 700);

        auto* splitter = new QSplitter(Qt::Horizontal, this);

        configPanel = new LqrConfigurationPanel(splitter);
        configPanel->setMaximumWidth(320);

        tabWidget = new QTabWidget(splitter);

        cartPoleWidget = new CartPoleWidget(tabWidget);
        cartPoleWidget->SetSimulator(&simulator);

        evaluationWidget = new LqrEvaluationWidget(tabWidget);

        tabWidget->addTab(cartPoleWidget, "Animation");
        tabWidget->addTab(evaluationWidget, "Evaluation");

        splitter->addWidget(configPanel);
        splitter->addWidget(tabWidget);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);

        statusBar()->showMessage("Configure parameters and press Start to begin simulation");

        connect(configPanel, &LqrConfigurationPanel::ConfigureRequested, this, &LqrMainWindow::OnConfigureRequested);
        connect(configPanel, &LqrConfigurationPanel::StartRequested, this, &LqrMainWindow::OnStartRequested);
        connect(configPanel, &LqrConfigurationPanel::StopRequested, this, &LqrMainWindow::OnStopRequested);
        connect(configPanel, &LqrConfigurationPanel::ResetRequested, this, &LqrMainWindow::OnResetRequested);
        connect(configPanel, &LqrConfigurationPanel::DisturbRequested, this, &LqrMainWindow::OnDisturbRequested);
        connect(cartPoleWidget, &CartPoleWidget::StateUpdated, configPanel, &LqrConfigurationPanel::OnStateUpdated);
        connect(cartPoleWidget, &CartPoleWidget::StateUpdated, evaluationWidget, &LqrEvaluationWidget::OnStateUpdated);

        OnConfigureRequested();
    }

    void LqrMainWindow::OnConfigureRequested()
    {
        auto config = configPanel->GetConfiguration();
        simulator.Configure(config);
        cartPoleWidget->Reset();
        evaluationWidget->SetConfig(config);
        evaluationWidget->Clear();
        statusBar()->showMessage("Configuration applied. Press Start to begin.");
    }

    void LqrMainWindow::OnStartRequested()
    {
        cartPoleWidget->Start();
        statusBar()->showMessage("Simulation running — click and drag to apply force");
    }

    void LqrMainWindow::OnStopRequested()
    {
        cartPoleWidget->Stop();
        statusBar()->showMessage("Simulation paused");
    }

    void LqrMainWindow::OnResetRequested()
    {
        cartPoleWidget->Reset();
        evaluationWidget->Clear();
        statusBar()->showMessage("Simulation reset");
    }

    void LqrMainWindow::OnDisturbRequested()
    {
        auto current = simulator.GetState();
        CartPoleState disturbed = current;
        disturbed.theta += 0.3f;
        simulator.SetState(disturbed);

        statusBar()->showMessage("Disturbance applied!");
    }
}
