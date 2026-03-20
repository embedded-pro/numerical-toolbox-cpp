#pragma once

#include "simulator/controllers/PidController/application/PidSimulator.hpp"
#include "simulator/controllers/common/Plant.hpp"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QPushButton>
#include <QWidget>
#include <memory>

namespace simulator::controllers::view
{
    class PidConfigurationPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit PidConfigurationPanel(QWidget* parent = nullptr);

        PidSimulator::Configuration GetConfiguration() const;
        std::unique_ptr<Plant> CreatePlant() const;
        QString GetPlantDescription() const;
        void SetKp(float kp);

    signals:
        void ComputeRequested();

    private:
        void SetupUi();
        void OnPlantOrderChanged(int index);

        QDoubleSpinBox* kpSpinBox;
        QDoubleSpinBox* kiSpinBox;
        QDoubleSpinBox* kdSpinBox;

        QComboBox* plantOrderCombo;

        QGroupBox* firstOrderGroup;
        QDoubleSpinBox* gainSpinBox;
        QDoubleSpinBox* timeConstantSpinBox;

        QGroupBox* secondOrderGroup;
        QDoubleSpinBox* naturalFreqSpinBox;
        QDoubleSpinBox* dampingRatioSpinBox;

        QDoubleSpinBox* durationSpinBox;
        QDoubleSpinBox* sampleTimeSpinBox;
        QDoubleSpinBox* outputMinSpinBox;
        QDoubleSpinBox* outputMaxSpinBox;

        QPushButton* computeButton;
    };
}
