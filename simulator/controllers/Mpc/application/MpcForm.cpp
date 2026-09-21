#include "simulator/controllers/Mpc/application/MpcForm.hpp"

namespace simulator::controllers::mpc
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::Condition;
        using ui::model::FieldKind;
        using ui::model::FieldSpec;
        using ui::model::GroupSpec;
        using ui::model::OptionSpec;

        constexpr std::array<OptionSpec, 2> plantTypes{
            OptionSpec{ "Double Integrator" },
            OptionSpec{ "1st Order + Integrator" }
        };

        constexpr Condition firstOrderSelected{ field::plantType, 0b10 };

        constexpr std::array<GroupSpec, 5> groups{
            GroupSpec{ field::weights, "MPC Weights", {} },
            GroupSpec{ field::constraints, "Constraints", {} },
            GroupSpec{ field::plant, "Plant Model", {} },
            GroupSpec{ field::plantParameters, "Plant Parameters", firstOrderSelected },
            GroupSpec{ field::simulation, "Simulation", {} }
        };

        constexpr std::array<FieldSpec, 11> fields{
            FieldSpec{ field::stateWeight, field::weights, FieldKind::Number, "Q (state):", "", { 0.01, 1000.0, 1.0, 10.0, 2 }, {}, {}, {} },
            FieldSpec{ field::controlWeight, field::weights, FieldKind::Number, "R (control):", "", { 0.001, 100.0, 1.0, 0.1, 3 }, {}, {}, {} },
            FieldSpec{ field::constraintsEnabled, field::constraints, FieldKind::Toggle, "Enable control constraints", "", {}, {}, {}, {} },
            FieldSpec{ field::controlMinimum, field::constraints, FieldKind::Number, "u_min:", "", { -100.0, 0.0, 1.0, -2.0, 2 }, {}, {}, {} },
            FieldSpec{ field::controlMaximum, field::constraints, FieldKind::Number, "u_max:", "", { 0.0, 100.0, 1.0, 2.0, 2 }, {}, {}, {} },
            FieldSpec{ field::plantType, field::plant, FieldKind::Choice, "Type:", "", {}, plantTypes, {}, {} },
            FieldSpec{ field::plantGain, field::plantParameters, FieldKind::Number, "Gain:", "", { 0.01, 100.0, 1.0, 1.0, 2 }, {}, firstOrderSelected, {} },
            FieldSpec{ field::plantTimeConstant, field::plantParameters, FieldKind::Number, "Time Const:", " s", { 0.01, 100.0, 1.0, 1.0, 2 }, {}, firstOrderSelected, {} },
            FieldSpec{ field::duration, field::simulation, FieldKind::Number, "Duration:", " s", { 0.1, 100.0, 1.0, 10.0, 1 }, {}, {}, {} },
            FieldSpec{ field::sampleTime, field::simulation, FieldKind::Number, "Sample Time:", " s", { 0.001, 1.0, 1.0, 0.1, 3 }, {}, {}, {} },
            FieldSpec{ field::reference, field::simulation, FieldKind::Number, "Reference:", "", { -100.0, 100.0, 1.0, 1.0, 2 }, {}, {}, {} }
        };

        constexpr std::array<ActionSpec, 1> actions{
            ActionSpec{ field::compute, "Compute", ui::theme::ButtonRole::Primary, 40 }
        };
    }

    MpcForm::MpcForm()
        : spec{ groups, fields, actions, {} }
        , model{ spec, values, {} }
    {}

    ui::model::FormModel& MpcForm::Model()
    {
        return model;
    }

    const ui::model::FormModel& MpcForm::Model() const
    {
        return model;
    }

    MpcSimulator::Configuration MpcForm::BuildConfiguration() const
    {
        MpcSimulator::Configuration config;

        config.weights.stateWeight = model.Float(field::stateWeight);
        config.weights.controlWeight = model.Float(field::controlWeight);

        config.constraints.enabled = model.Flag(field::constraintsEnabled);
        config.constraints.uMin = model.Float(field::controlMinimum);
        config.constraints.uMax = model.Float(field::controlMaximum);

        config.simulation.duration = model.Float(field::duration);
        config.simulation.sampleTime = model.Float(field::sampleTime);

        config.referencePosition = model.Float(field::reference);

        return config;
    }

    math::LinearTimeInvariant<float, 2, 1> MpcForm::CreatePlant() const
    {
        const auto dt = model.Float(field::sampleTime);

        if (model.Selection(field::plantType) == 0)
            return MakeDoubleIntegrator(dt);

        return MakeFirstOrderWithIntegrator(model.Float(field::plantGain), model.Float(field::plantTimeConstant), dt);
    }

    std::string_view MpcForm::PlantDescription() const
    {
        return model.Field(field::plantType).options[model.Selection(field::plantType)].label;
    }
}
