#pragma once

#include "simulator/controllers/LqrCartPole/application/LqrCartPoleSimulator.hpp"
#include "simulator/controllers/LqrCartPole/view/CartPoleWidget.hpp"
#include "simulator/controllers/LqrCartPole/view/LqrConfigurationPanel.hpp"
#include "simulator/controllers/LqrCartPole/view/LqrEvaluationWidget.hpp"
#include <QMainWindow>
#include <QStatusBar>
#include <QTabWidget>

namespace simulator::controllers::lqr::view
{
    class LqrMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit LqrMainWindow(QWidget* parent = nullptr);

    private:
        void OnConfigureRequested();
        void OnStartRequested();
        void OnStopRequested();
        void OnResetRequested();
        void OnDisturbRequested();

        LqrCartPoleSimulator simulator;
        LqrConfigurationPanel* configPanel;
        CartPoleWidget* cartPoleWidget;
        LqrEvaluationWidget* evaluationWidget;
        QTabWidget* tabWidget;
    };
}
