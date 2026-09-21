#include "simulator/controllers/Lqg/application/LqgForm.hpp"

namespace simulator::controllers::lqg
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::FieldKind;
        using ui::model::FieldSpec;
        using ui::model::GroupSpec;

        constexpr std::array<GroupSpec, 3> groups{
            GroupSpec{ field::weights, "LQR Weights", {} },
            GroupSpec{ field::noise, "Kalman Noise", {} },
            GroupSpec{ field::simulation, "Simulation", {} }
        };

        constexpr std::array<FieldSpec, 7> fields{
            FieldSpec{ field::stateWeight, field::weights, FieldKind::Number, "State Weight Q:", "", { 0.01, 1000.0, 1.0, 10.0, 2 }, {}, {}, {} },
            FieldSpec{ field::controlWeight, field::weights, FieldKind::Number, "Control Weight R:", "", { 0.001, 100.0, 0.01, 0.1, 2 }, {}, {}, {} },
            FieldSpec{ field::processNoise, field::noise, FieldKind::Number, "Process Noise:", "", { 0.001, 10.0, 0.01, 0.1, 2 }, {}, {}, {} },
            FieldSpec{ field::measurementNoise, field::noise, FieldKind::Number, "Measurement Noise:", "", { 0.001, 10.0, 0.1, 1.0, 2 }, {}, {}, {} },
            FieldSpec{ field::sampleTime, field::simulation, FieldKind::Number, "Sample Time (s):", "", { 0.001, 1.0, 0.01, 0.1, 2 }, {}, {}, {} },
            FieldSpec{ field::duration, field::simulation, FieldKind::Number, "Duration (s):", "", { 1.0, 100.0, 1.0, 10.0, 2 }, {}, {}, {} },
            FieldSpec{ field::initialPosition, field::simulation, FieldKind::Number, "Initial Position:", "", { -10.0, 10.0, 1.0, 1.0, 2 }, {}, {}, {} }
        };

        constexpr std::array<ActionSpec, 1> actions{
            ActionSpec{ field::compute, "Compute", ui::theme::ButtonRole::Primary, 0 }
        };
    }

    LqgForm::LqgForm()
        : spec{ groups, fields, actions, {} }
        , model{ spec, values, {} }
    {}

    ui::model::FormModel& LqgForm::Model()
    {
        return model;
    }

    const ui::model::FormModel& LqgForm::Model() const
    {
        return model;
    }

    LqgSimulatorConfig LqgForm::BuildConfiguration() const
    {
        LqgSimulatorConfig config;

        config.weights.stateWeight = model.Float(field::stateWeight);
        config.weights.controlWeight = model.Float(field::controlWeight);
        config.noise.processNoise = model.Float(field::processNoise);
        config.noise.measurementNoise = model.Float(field::measurementNoise);
        config.simulation.sampleTime = model.Float(field::sampleTime);
        config.simulation.duration = model.Float(field::duration);
        config.initialPosition = model.Float(field::initialPosition);

        return config;
    }

    LqgPlant LqgForm::CreatePlant() const
    {
        const auto dt = model.Float(field::sampleTime);

        math::SquareMatrix<float, lqgStateSize> a{
            { 1.0f, dt },
            { 0.0f, 1.0f }
        };
        math::Matrix<float, lqgStateSize, lqgInputSize> b{
            { 0.0f },
            { dt }
        };
        math::Matrix<float, lqgMeasurementSize, lqgStateSize> c{
            { 1.0f, 0.0f }
        };
        math::Matrix<float, lqgMeasurementSize, lqgInputSize> d{};

        return LqgPlant{ a, b, c, d };
    }
}
