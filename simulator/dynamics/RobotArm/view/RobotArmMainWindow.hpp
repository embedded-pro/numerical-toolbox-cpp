#pragma once

#include "simulator/dynamics/RobotArm/application/RobotArmSimulator.hpp"
#include "simulator/dynamics/RobotArm/view/RobotArm3DWidget.hpp"
#include "simulator/dynamics/RobotArm/view/RobotArmConfigurationPanel.hpp"
#include <QMainWindow>
#include <QStatusBar>
#include <QTimer>

namespace simulator::dynamics::view
{
    class RobotArmMainWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit RobotArmMainWindow(QWidget* parent = nullptr);

    private:
        void OnStartRequested();
        void OnStopRequested();
        void OnResetRequested();
        void OnSimulationStep();
        void ApplyConfiguration();

        RobotArmSimulator simulator;
        RobotArmConfigurationPanel* configPanel;
        RobotArm3DWidget* view3D;
        QTimer* simulationTimer;
        bool running = false;
    };
}
