#pragma once

#include "simulator/controllers/LqrCartPole/application/LqrForm.hpp"
#include "simulator/controllers/LqrCartPole/view/CartPoleWidget.hpp"
#include "simulator/controllers/LqrCartPole/view/LqrEvaluationWidget.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/QtFormView.hpp"
#include <QMainWindow>

namespace simulator::controllers::lqr::view
{
    class LqrMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit LqrMainWindow(QWidget* parent = nullptr);

    private:
        void OnActionTriggered(ui::model::ActionId action);
        void OnConfigureRequested();
        void OnStartRequested();
        void OnStopRequested();
        void OnResetRequested();
        void OnDisturbRequested();
        void OnStateUpdated(float x, float xDot, float theta, float thetaDot, float force);

        LqrCartPoleSimulator simulator;
        LqrForm form;
        ui::backend::qt::QtFormView* formView;
        ui::backend::qt::QtAppShell shell;

        CartPoleWidget* cartPoleWidget;
        LqrEvaluationWidget* evaluationWidget;
    };
}
