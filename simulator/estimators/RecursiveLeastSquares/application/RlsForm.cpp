#include "simulator/estimators/RecursiveLeastSquares/application/RlsForm.hpp"

namespace simulator::estimators::rls
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::FieldKind;
        using ui::model::FieldSpec;
        using ui::model::GroupSpec;

        constexpr std::array<GroupSpec, 2> groups{
            GroupSpec{ field::parameters, "RLS Parameters", {} },
            GroupSpec{ field::coefficients, "True Coefficients (θ₀ + θ₁·x₁ + θ₂·x₂)", {} }
        };

        constexpr std::array<FieldSpec, 7> fields{
            FieldSpec{ field::forgettingFactor, field::parameters, FieldKind::Number, "Forgetting Factor (λ):", "", { 0.9, 1.0, 0.005, 0.99, 3 }, {}, {}, {} },
            FieldSpec{ field::initialCovariance, field::parameters, FieldKind::Number, "Initial Covariance:", "", { 1.0, 10000.0, 100.0, 1000.0, 0 }, {}, {}, {} },
            FieldSpec{ field::sampleCount, field::parameters, FieldKind::Integer, "Number of Samples:", "", { 50.0, 2000.0, 50.0, 200.0, 0 }, {}, {}, {} },
            FieldSpec{ field::noiseAmplitude, field::parameters, FieldKind::Number, "Noise Amplitude:", "", { 0.0, 5.0, 0.05, 0.1, 2 }, {}, {}, {} },
            FieldSpec{ field::bias, field::coefficients, FieldKind::Number, "θ₀ (bias):", "", { -10.0, 10.0, 0.1, 2.0, 2 }, {}, {}, {} },
            FieldSpec{ field::firstCoefficient, field::coefficients, FieldKind::Number, "θ₁:", "", { -10.0, 10.0, 0.1, -1.5, 2 }, {}, {}, {} },
            FieldSpec{ field::secondCoefficient, field::coefficients, FieldKind::Number, "θ₂:", "", { -10.0, 10.0, 0.1, 0.8, 2 }, {}, {}, {} }
        };

        constexpr std::array<ActionSpec, 1> actions{
            ActionSpec{ field::compute, "Compute", ui::theme::ButtonRole::Primary, 0 }
        };
    }

    RlsForm::RlsForm()
        : spec{ groups, fields, actions, {} }
        , model{ spec, values, {} }
    {}

    ui::model::FormModel& RlsForm::Model()
    {
        return model;
    }

    const ui::model::FormModel& RlsForm::Model() const
    {
        return model;
    }

    RlsSimulator::Configuration RlsForm::BuildConfiguration() const
    {
        RlsSimulator::Configuration config;

        config.rls.forgettingFactor = model.Float(field::forgettingFactor);
        config.rls.initialCovariance = model.Float(field::initialCovariance);
        config.rls.numSamples = model.Count(field::sampleCount);
        config.rls.noiseAmplitude = model.Float(field::noiseAmplitude);
        config.rls.trueCoefficients = {
            model.Float(field::bias),
            model.Float(field::firstCoefficient),
            model.Float(field::secondCoefficient)
        };

        return config;
    }
}
