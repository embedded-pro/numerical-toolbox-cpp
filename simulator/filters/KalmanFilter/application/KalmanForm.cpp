#include "simulator/filters/KalmanFilter/application/KalmanForm.hpp"

namespace simulator::filters::kalman
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::FieldKind;
        using ui::model::FieldSpec;
        using ui::model::GroupSpec;

        constexpr std::array<GroupSpec, 3> groups{
            GroupSpec{ field::simulation, "Simulation", {} },
            GroupSpec{ field::noise, "Noise", {} },
            GroupSpec{ field::pendulum, "Pendulum", {} }
        };

        constexpr std::array<FieldSpec, 9> fields{
            FieldSpec{ field::duration, field::simulation, FieldKind::Number, "Duration (s):", "", { 1.0, 60.0, 1.0, 10.0, 1 }, {}, {}, {} },
            FieldSpec{ field::timeStep, field::simulation, FieldKind::Number, "Time Step (s):", "", { 0.001, 0.1, 0.001, 0.01, 3 }, {}, {}, {} },
            FieldSpec{ field::initialAngle, field::simulation, FieldKind::Number, "Initial θ (rad):", "", { -3.14, 3.14, 0.1, 0.5, 2 }, {}, {}, {} },
            FieldSpec{ field::initialAngularRate, field::simulation, FieldKind::Number, "Initial θ̇ (rad/s):", "", { -10.0, 10.0, 0.1, 0.0, 2 }, {}, {}, {} },
            FieldSpec{ field::measurementNoise, field::noise, FieldKind::Number, "Measurement σ:", "", { 0.0, 1.0, 0.01, 0.1, 3 }, {}, {}, {} },
            FieldSpec{ field::processNoise, field::noise, FieldKind::Number, "Process σ:", "", { 0.0, 1.0, 0.001, 0.01, 3 }, {}, {}, {} },
            FieldSpec{ field::length, field::pendulum, FieldKind::Number, "Length (m):", "", { 0.1, 10.0, 0.1, 1.0, 2 }, {}, {}, {} },
            FieldSpec{ field::mass, field::pendulum, FieldKind::Number, "Mass (kg):", "", { 0.1, 10.0, 0.1, 1.0, 2 }, {}, {}, {} },
            FieldSpec{ field::damping, field::pendulum, FieldKind::Number, "Damping:", "", { 0.0, 5.0, 0.01, 0.1, 3 }, {}, {}, {} }
        };

        constexpr std::array<ActionSpec, 1> actions{
            ActionSpec{ field::compute, "Compute", ui::theme::ButtonRole::Primary, 40 }
        };
    }

    KalmanForm::KalmanForm()
        : spec{ groups, fields, actions, {} }
        , model{ spec, values, {} }
    {}

    ui::model::FormModel& KalmanForm::Model()
    {
        return model;
    }

    const ui::model::FormModel& KalmanForm::Model() const
    {
        return model;
    }

    SimulationConfig KalmanForm::BuildConfiguration() const
    {
        SimulationConfig config;

        config.duration = model.Float(field::duration);
        config.dt = model.Float(field::timeStep);
        config.initialTheta = model.Float(field::initialAngle);
        config.initialThetaDot = model.Float(field::initialAngularRate);
        config.measurementNoiseStdDev = model.Float(field::measurementNoise);
        config.processNoiseStdDev = model.Float(field::processNoise);
        config.pendulum.length = model.Float(field::length);
        config.pendulum.mass = model.Float(field::mass);
        config.pendulum.damping = model.Float(field::damping);

        return config;
    }
}
