#include "simulator/filters/FirFilter/application/FirForm.hpp"

namespace simulator::filters::fir
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::Condition;
        using ui::model::FieldKind;
        using ui::model::FieldSpec;
        using ui::model::GroupSpec;
        using ui::model::OptionSpec;

        constexpr std::array<OptionSpec, 3> filterTypes{
            OptionSpec{ "Low-Pass", static_cast<std::int64_t>(FilterType::LowPass) },
            OptionSpec{ "High-Pass", static_cast<std::int64_t>(FilterType::HighPass) },
            OptionSpec{ "Band-Pass", static_cast<std::int64_t>(FilterType::BandPass) }
        };

        // The upper cutoff is meaningful only for a band-pass, which the panel expressed as a
        // handler wired after the initial selection; here it is the field's own description.
        constexpr Condition bandPassOnly{ field::filterType, 1u << 2u };

        constexpr std::array<GroupSpec, 3> groups{
            GroupSpec{ field::design, "Filter Design", {} },
            GroupSpec{ field::simulation, "Simulation", {} },
            GroupSpec{ field::signal, "Signal Components", {} }
        };

        constexpr std::array<FieldSpec, 6> fields{
            FieldSpec{ field::filterType, field::design, FieldKind::Choice, "Type:", "", {}, filterTypes, {}, {} },
            FieldSpec{ field::cutoff, field::design, FieldKind::Number, "Cutoff (Hz):", " Hz", { 1.0, 22050.0, 1.0, 1000.0, 1 }, {}, {}, {} },
            FieldSpec{ field::cutoffHigh, field::design, FieldKind::Number, "Upper Cutoff (Hz):", " Hz", { 1.0, 22050.0, 1.0, 3000.0, 1 }, {}, {}, bandPassOnly },
            FieldSpec{ field::order, field::design, FieldKind::Integer, "Order:", "", { 3.0, 127.0, 2.0, 31.0, 0 }, {}, {}, {} },
            FieldSpec{ field::sampleRate, field::simulation, FieldKind::Number, "Sample Rate (Hz):", " Hz", { 100.0, 192000.0, 1.0, 8000.0, 1 }, {}, {}, {} },
            FieldSpec{ field::duration, field::simulation, FieldKind::Number, "Duration (s):", " s", { 0.001, 1.0, 0.01, 0.05, 3 }, {}, {}, {} }
        };

        constexpr std::array<ActionSpec, 1> actions{
            ActionSpec{ field::compute, "Compute", ui::theme::ButtonRole::Primary, 40 }
        };

        constexpr std::array<utils::SignalComponent, 2> defaultComponents{
            utils::SignalComponent{ 200.0f, 1.0f },
            utils::SignalComponent{ 2000.0f, 0.5f }
        };
    }

    FirForm::FirForm()
        : tableSpecs{ ui::model::TableSpec{ field::signal, shell::signalColumns, maximumComponents, "Add", "Remove" } }
        , tables{ ui::model::TableModel{ tableSpecs[0], cells } }
        , spec{ groups, fields, actions, tableSpecs }
        , model{ spec, values, tables }
    {
        shell::SeedFrom(defaultComponents, tables[0]);
    }

    ui::model::FormModel& FirForm::Model()
    {
        return model;
    }

    const ui::model::FormModel& FirForm::Model() const
    {
        return model;
    }

    FirFilterSimulator::Configuration FirForm::BuildConfiguration() const
    {
        FirFilterSimulator::Configuration config;

        config.filter.type = static_cast<FilterType>(model.SelectedData(field::filterType));
        config.filter.cutoffHz = model.Float(field::cutoff);

        // Read unconditionally, exactly as the panel did: a disabled upper cutoff was greyed
        // rather than excluded, and the simulator ignores it outside a band-pass.
        config.filter.cutoffHighHz = model.Float(field::cutoffHigh);
        config.filter.order = model.Count(field::order);
        config.filter.sampleRateHz = model.Float(field::sampleRate);

        config.simulation.sampleRateHz = config.filter.sampleRateHz;
        config.simulation.duration = model.Float(field::duration);
        config.simulation.signalComponents = shell::ToComponents(model.Table(0));

        return config;
    }
}
