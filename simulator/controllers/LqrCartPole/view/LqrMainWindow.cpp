#include "simulator/controllers/LqrCartPole/view/LqrMainWindow.hpp"
#include "simulator/shell/Guard.hpp"
#include <array>

namespace simulator::controllers::lqr::view
{
    namespace
    {
        constexpr std::array<ui::shell::PageSpec, 2> pages{
            ui::shell::PageSpec{ "Animation" },
            ui::shell::PageSpec{ "Evaluation" }
        };

        const ui::shell::ShellSpec shellSpec{
            "LQR Cart-Pole Simulator",
            ui::Size{ 1280.0f, 700.0f },
            320.0f,
            pages,
            "Configure parameters and press Start to begin simulation"
        };
    }

    LqrMainWindow::LqrMainWindow(QWidget* parent)
        : QMainWindow(parent)
        , formView(new ui::backend::qt::QtFormView{ this })
        , shell(*this, shellSpec)
        , cartPoleWidget(new CartPoleWidget{ this })
        , evaluationWidget(new LqrEvaluationWidget{ this })
    {
        formView->Build(form.Model());
        shell.SetPanel(formView);

        cartPoleWidget->SetSimulator(&simulator);

        shell.SetPage(0, cartPoleWidget);
        shell.SetPage(1, evaluationWidget);

        connect(cartPoleWidget, &CartPoleWidget::StateUpdated, this, &LqrMainWindow::OnStateUpdated);
        connect(cartPoleWidget, &CartPoleWidget::StateUpdated, evaluationWidget, &LqrEvaluationWidget::OnStateUpdated);

        form.Model().onActionTriggered = [this](ui::model::ActionId action)
        {
            OnActionTriggered(action);
        };

        OnConfigureRequested();
    }

    void LqrMainWindow::OnActionTriggered(ui::model::ActionId action)
    {
        if (action == field::configure)
            OnConfigureRequested();
        else if (action == field::start)
            OnStartRequested();
        else if (action == field::stop)
            OnStopRequested();
        else if (action == field::reset)
            OnResetRequested();
        else if (action == field::disturb)
            OnDisturbRequested();
    }

    void LqrMainWindow::OnConfigureRequested()
    {
        shell::Guard(shell, "Configuration Error", [this]
            {
                auto config = form.BuildConfiguration();
                simulator.Configure(config);
                cartPoleWidget->Reset();
                evaluationWidget->SetConfig(config);
                evaluationWidget->Clear();
                shell.SetStatus("Configuration applied. Press Start to begin.");
            });
    }

    void LqrMainWindow::OnStartRequested()
    {
        cartPoleWidget->Start();
        shell.SetStatus("Simulation running — click and drag to apply force");
    }

    void LqrMainWindow::OnStopRequested()
    {
        cartPoleWidget->Stop();
        shell.SetStatus("Simulation paused");
    }

    void LqrMainWindow::OnResetRequested()
    {
        cartPoleWidget->Reset();
        evaluationWidget->Clear();
        shell.SetStatus("Simulation reset");
    }

    void LqrMainWindow::OnDisturbRequested()
    {
        auto disturbed = simulator.GetState();
        disturbed.theta += 0.3f;
        simulator.SetState(disturbed);

        shell.SetStatus("Disturbance applied!");
    }

    void LqrMainWindow::OnStateUpdated(float x, float xDot, float theta, float thetaDot, float force)
    {
        form.SetState(x, xDot, theta, thetaDot, force);

        formView->Refresh(field::readOutPosition);
        formView->Refresh(field::readOutVelocity);
        formView->Refresh(field::readOutAngle);
        formView->Refresh(field::readOutAngularRate);
        formView->Refresh(field::readOutForce);
    }
}
