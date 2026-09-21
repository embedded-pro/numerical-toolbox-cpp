#include "simulator/controllers/PidController/application/PidForm.hpp"
#include <algorithm>

namespace simulator::controllers::pid
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::Condition;
        using ui::model::FieldKind;
        using ui::model::FieldSpec;
        using ui::model::GroupSpec;
        using ui::model::OptionSpec;

        constexpr std::array<OptionSpec, 2> plantOrders{
            OptionSpec{ "1st Order" },
            OptionSpec{ "2nd Order" }
        };

        constexpr Condition firstOrderSelected{ field::plantOrder, 0b01 };
        constexpr Condition secondOrderSelected{ field::plantOrder, 0b10 };

        constexpr std::array<GroupSpec, 5> groups{
            GroupSpec{ field::tuning, "PID Tuning", {} },
            GroupSpec{ field::plant, "Plant Model", {} },
            GroupSpec{ field::firstOrder, "K / (τs + 1)", firstOrderSelected },
            GroupSpec{ field::secondOrder, "ωn² / (s² + 2ζωns + ωn²)", secondOrderSelected },
            GroupSpec{ field::simulation, "Simulation", {} }
        };

        constexpr std::array<FieldSpec, 12> fields{
            FieldSpec{ field::kp, field::tuning, FieldKind::Number, "Kp:", "", { 0.0, 100.0, 1.0, 1.0, 3 }, {}, {}, {} },
            FieldSpec{ field::ki, field::tuning, FieldKind::Number, "Ki:", "", { 0.0, 100.0, 1.0, 0.1, 3 }, {}, {}, {} },
            FieldSpec{ field::kd, field::tuning, FieldKind::Number, "Kd:", "", { 0.0, 100.0, 1.0, 0.05, 3 }, {}, {}, {} },
            FieldSpec{ field::plantOrder, field::plant, FieldKind::Choice, "Order:", "", {}, plantOrders, {}, {} },
            FieldSpec{ field::gain, field::firstOrder, FieldKind::Number, "Gain (K):", "", { 0.01, 1000.0, 1.0, 1.0, 3 }, {}, firstOrderSelected, {} },
            FieldSpec{ field::timeConstant, field::firstOrder, FieldKind::Number, "Time Const (τ):", " s", { 0.01, 100.0, 1.0, 1.0, 3 }, {}, firstOrderSelected, {} },
            FieldSpec{ field::naturalFrequency, field::secondOrder, FieldKind::Number, "Natural Freq (ωn):", " rad/s", { 0.01, 100.0, 1.0, 1.0, 2 }, {}, secondOrderSelected, {} },
            FieldSpec{ field::dampingRatio, field::secondOrder, FieldKind::Number, "Damping Ratio (ζ):", "", { 0.0, 5.0, 1.0, 0.5, 3 }, {}, secondOrderSelected, {} },
            FieldSpec{ field::duration, field::simulation, FieldKind::Number, "Duration:", " s", { 0.1, 1000.0, 1.0, 20.0, 1 }, {}, {}, {} },
            FieldSpec{ field::sampleTime, field::simulation, FieldKind::Number, "Sample Time:", " s", { 0.0001, 1.0, 1.0, 0.01, 4 }, {}, {}, {} },
            FieldSpec{ field::outputMinimum, field::simulation, FieldKind::Number, "Output Min:", "", { -10000.0, 0.0, 1.0, -100.0, 1 }, {}, {}, {} },
            FieldSpec{ field::outputMaximum, field::simulation, FieldKind::Number, "Output Max:", "", { 0.0, 10000.0, 1.0, 100.0, 1 }, {}, {}, {} }
        };

        constexpr std::array<ActionSpec, 1> actions{
            ActionSpec{ field::compute, "Compute", ui::theme::ButtonRole::Primary, 40 }
        };
    }

    PidForm::PidForm()
        : spec{ groups, fields, actions, {} }
        , model{ spec, values, {} }
    {
        model.SetSelection(field::plantOrder, 1);
    }

    ui::model::FormModel& PidForm::Model()
    {
        return model;
    }

    const ui::model::FormModel& PidForm::Model() const
    {
        return model;
    }

    PidSimulator::Configuration PidForm::BuildConfiguration() const
    {
        PidSimulator::Configuration config;

        config.tunings.kp = model.Float(field::kp);
        config.tunings.ki = model.Float(field::ki);
        config.tunings.kd = model.Float(field::kd);

        config.limits.min = model.Float(field::outputMinimum);
        config.limits.max = model.Float(field::outputMaximum);

        config.simulation.duration = model.Float(field::duration);
        config.simulation.sampleTime = model.Float(field::sampleTime);

        return config;
    }

    std::unique_ptr<Plant> PidForm::CreatePlant() const
    {
        if (model.Selection(field::plantOrder) == 0)
            return std::make_unique<FirstOrderPlant>(model.Float(field::gain), model.Float(field::timeConstant));

        return std::make_unique<SecondOrderPlant>(model.Float(field::naturalFrequency), model.Float(field::dampingRatio));
    }

    std::string_view PidForm::PlantDescription() const
    {
        return model.Field(field::plantOrder).options[model.Selection(field::plantOrder)].label;
    }

    void PidForm::SetProportionalGain(float value)
    {
        const auto& traits = model.Field(field::kp).number;

        model.SetNumber(field::kp, std::clamp(static_cast<double>(value), traits.minimum, traits.maximum));
    }
}
