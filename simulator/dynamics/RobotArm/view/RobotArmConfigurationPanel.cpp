#include "simulator/dynamics/RobotArm/view/RobotArmConfigurationPanel.hpp"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <cmath>

namespace simulator::dynamics::view
{
    RobotArmConfigurationPanel::RobotArmConfigurationPanel(QWidget* parent)
        : QWidget(parent)
    {
        SetupUi();
    }

    void RobotArmConfigurationPanel::SetupUi()
    {
        auto* mainLayout = new QVBoxLayout(this);
        mainLayout->setContentsMargins(8, 8, 8, 8);
        mainLayout->setSpacing(6);

        // DOF selection
        auto* dofGroup = new QGroupBox("Robot Configuration", this);
        auto* dofLayout = new QHBoxLayout(dofGroup);
        dofLayout->addWidget(new QLabel("DOF:", this));
        dofCombo = new QComboBox(this);
        dofCombo->addItem("2 DOF (Planar)", 2);
        dofCombo->addItem("3 DOF (Spatial)", 3);
        dofLayout->addWidget(dofCombo);
        mainLayout->addWidget(dofGroup);

        // Link parameters
        std::array<float, 3> defaultLengths = { 0.5f, 0.4f, 0.3f };
        std::array<float, 3> defaultMasses = { 1.0f, 0.8f, 0.6f };

        for (int i = 0; i < maxDof; ++i)
        {
            auto* group = new QGroupBox(QString("Link %1").arg(i + 1), this);
            auto* layout = new QVBoxLayout(group);

            auto* lenLayout = new QHBoxLayout();
            lenLayout->addWidget(new QLabel("Length (m):", this));
            auto* lenSpin = new QDoubleSpinBox(this);
            lenSpin->setRange(0.05, 2.0);
            lenSpin->setSingleStep(0.05);
            lenSpin->setValue(static_cast<double>(defaultLengths[i]));
            lenSpin->setDecimals(2);
            lenLayout->addWidget(lenSpin);
            layout->addLayout(lenLayout);

            auto* massLayout = new QHBoxLayout();
            massLayout->addWidget(new QLabel("Mass (kg):", this));
            auto* massSpin = new QDoubleSpinBox(this);
            massSpin->setRange(0.1, 20.0);
            massSpin->setSingleStep(0.1);
            massSpin->setValue(static_cast<double>(defaultMasses[i]));
            massSpin->setDecimals(2);
            massLayout->addWidget(massSpin);
            layout->addLayout(massLayout);

            linkGroups.push_back(group);
            lengthSpins.push_back(lenSpin);
            massSpins.push_back(massSpin);
            mainLayout->addWidget(group);
        }

        // Torque sliders
        torqueGroup = new QGroupBox("Joint Torques (N·m)", this);
        auto* torqueLayout = new QVBoxLayout(torqueGroup);

        for (int i = 0; i < maxDof; ++i)
        {
            auto* rowLayout = new QHBoxLayout();
            rowLayout->addWidget(new QLabel(QString("τ%1:").arg(i + 1), this));

            auto* slider = new QSlider(Qt::Horizontal, this);
            slider->setRange(-200, 200);
            slider->setValue(0);
            slider->setTickPosition(QSlider::TicksBelow);
            slider->setTickInterval(50);
            rowLayout->addWidget(slider);

            auto* label = new QLabel("0.0", this);
            label->setFixedWidth(55);
            label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            rowLayout->addWidget(label);

            torqueSliders.push_back(slider);
            torqueLabels.push_back(label);
            torqueLayout->addLayout(rowLayout);

            connect(slider, &QSlider::valueChanged, this, &RobotArmConfigurationPanel::UpdateTorqueLabels);
        }
        mainLayout->addWidget(torqueGroup);

        // Initial position sliders
        positionGroup = new QGroupBox("Initial Position (deg)", this);
        auto* posLayout = new QVBoxLayout(positionGroup);

        for (int i = 0; i < maxDof; ++i)
        {
            auto* rowLayout = new QHBoxLayout();
            rowLayout->addWidget(new QLabel(QString("q%1:").arg(i + 1), this));

            auto* slider = new QSlider(Qt::Horizontal, this);
            slider->setRange(-180, 180);
            slider->setValue(0);
            slider->setTickPosition(QSlider::TicksBelow);
            slider->setTickInterval(45);
            rowLayout->addWidget(slider);

            auto* label = new QLabel("0°", this);
            label->setFixedWidth(45);
            label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            rowLayout->addWidget(label);

            positionSliders.push_back(slider);
            positionLabels.push_back(label);
            posLayout->addLayout(rowLayout);

            connect(slider, &QSlider::valueChanged, this, &RobotArmConfigurationPanel::UpdatePositionLabels);
        }
        mainLayout->addWidget(positionGroup);

        // Damping
        auto* dampGroup = new QGroupBox("Simulation", this);
        auto* dampLayout = new QHBoxLayout(dampGroup);
        dampLayout->addWidget(new QLabel("Damping:", this));
        dampingSpin = new QDoubleSpinBox(this);
        dampingSpin->setRange(0.0, 5.0);
        dampingSpin->setSingleStep(0.01);
        dampingSpin->setValue(0.05);
        dampingSpin->setDecimals(3);
        dampLayout->addWidget(dampingSpin);
        mainLayout->addWidget(dampGroup);

        // Control buttons
        auto* controlLayout = new QHBoxLayout();
        startButton = new QPushButton("▶ Start", this);
        stopButton = new QPushButton("⬜ Stop", this);
        resetButton = new QPushButton("↺ Reset", this);

        startButton->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 6px; }");
        stopButton->setStyleSheet("QPushButton { background-color: #f44336; color: white; padding: 6px; }");
        resetButton->setStyleSheet("QPushButton { background-color: #2196F3; color: white; padding: 6px; }");

        controlLayout->addWidget(startButton);
        controlLayout->addWidget(stopButton);
        controlLayout->addWidget(resetButton);
        mainLayout->addLayout(controlLayout);

        mainLayout->addStretch();

        // Connections
        connect(dofCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &RobotArmConfigurationPanel::OnDofChanged);
        connect(startButton, &QPushButton::clicked, this, &RobotArmConfigurationPanel::StartRequested);
        connect(stopButton, &QPushButton::clicked, this, &RobotArmConfigurationPanel::StopRequested);
        connect(resetButton, &QPushButton::clicked, this, &RobotArmConfigurationPanel::ResetRequested);

        OnDofChanged(0);
    }

    void RobotArmConfigurationPanel::OnDofChanged(int index)
    {
        int dof = dofCombo->itemData(index).toInt();
        for (int i = 0; i < maxDof; ++i)
        {
            bool visible = (i < dof);
            linkGroups[i]->setVisible(visible);
            torqueSliders[i]->parentWidget()->setVisible(visible);
            positionSliders[i]->parentWidget()->setVisible(visible);
        }

        emit ConfigurationChanged();
    }

    void RobotArmConfigurationPanel::UpdateTorqueLabels()
    {
        for (int i = 0; i < maxDof; ++i)
        {
            float value = torqueSliders[i]->value() / 10.0f;
            torqueLabels[i]->setText(QString::number(static_cast<double>(value), 'f', 1));
        }
    }

    void RobotArmConfigurationPanel::UpdatePositionLabels()
    {
        for (int i = 0; i < maxDof; ++i)
            positionLabels[i]->setText(QString("%1°").arg(positionSliders[i]->value()));
    }

    RobotArmConfig RobotArmConfigurationPanel::GetConfiguration() const
    {
        RobotArmConfig cfg;
        cfg.dof = dofCombo->currentData().toInt();
        cfg.damping = static_cast<float>(dampingSpin->value());

        cfg.linkLengths.resize(cfg.dof);
        cfg.linkMasses.resize(cfg.dof);

        for (int i = 0; i < cfg.dof; ++i)
        {
            cfg.linkLengths[i] = static_cast<float>(lengthSpins[i]->value());
            cfg.linkMasses[i] = static_cast<float>(massSpins[i]->value());
        }

        return cfg;
    }

    std::vector<float> RobotArmConfigurationPanel::GetTorques() const
    {
        int dof = dofCombo->currentData().toInt();
        std::vector<float> t(dof);
        for (int i = 0; i < dof; ++i)
            t[i] = torqueSliders[i]->value() / 10.0f;
        return t;
    }

    std::vector<float> RobotArmConfigurationPanel::GetInitialPositions() const
    {
        int dof = dofCombo->currentData().toInt();
        std::vector<float> pos(dof);
        for (int i = 0; i < dof; ++i)
            pos[i] = positionSliders[i]->value() * 3.14159265f / 180.0f;
        return pos;
    }
}
