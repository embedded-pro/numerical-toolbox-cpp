#pragma once

#include "simulator/dynamics/RobotArm/application/RobotArmSimulator.hpp"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QWidget>
#include <vector>

namespace simulator::dynamics::view
{
    class RobotArmConfigurationPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit RobotArmConfigurationPanel(QWidget* parent = nullptr);

        RobotArmConfig GetConfiguration() const;
        std::vector<float> GetTorques() const;
        std::vector<float> GetInitialPositions() const;

    signals:
        void ConfigurationChanged();
        void StartRequested();
        void StopRequested();
        void ResetRequested();

    private:
        void SetupUi();
        void OnDofChanged(int index);
        void UpdateTorqueLabels();
        void UpdatePositionLabels();

        QComboBox* dofCombo;

        static constexpr int maxDof = 3;

        std::vector<QGroupBox*> linkGroups;
        std::vector<QDoubleSpinBox*> lengthSpins;
        std::vector<QDoubleSpinBox*> massSpins;

        std::vector<QSlider*> torqueSliders;
        std::vector<QLabel*> torqueLabels;
        QGroupBox* torqueGroup;

        std::vector<QSlider*> positionSliders;
        std::vector<QLabel*> positionLabels;
        QGroupBox* positionGroup;

        QDoubleSpinBox* dampingSpin;

        QPushButton* startButton;
        QPushButton* stopButton;
        QPushButton* resetButton;
    };
}
