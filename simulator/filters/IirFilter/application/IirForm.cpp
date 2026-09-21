#include "simulator/filters/IirFilter/application/IirForm.hpp"

namespace simulator::filters::iir
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::FieldKind;
        using ui::model::FieldSpec;
        using ui::model::GroupSpec;
        using ui::model::OptionSpec;

        constexpr std::array<OptionSpec, 3> filterTypes{
            OptionSpec{ "Low-Pass", static_cast<std::int64_t>(FilterType::LowPass) },
            OptionSpec{ "High-Pass", static_cast<std::int64_t>(FilterType::HighPass) },
            OptionSpec{ "Band-Pass", static_cast<std::int64_t>(FilterType::BandPass) }
        };

        constexpr std::array<GroupSpec, 3> groups{
            GroupSpec{ field::design, "Filter Design", {} },
            GroupSpec{ field::simulation, "Simulation", {} },
            GroupSpec{ field::signal, "Signal Components", {} }
        };

        constexpr std::array<FieldSpec, 5> fields{
            FieldSpec{ field::filterType, field::design, FieldKind::Choice, "Type:", "", {}, filterTypes, {}, {} },
            FieldSpec{ field::cutoff, field::design, FieldKind::Number, "Cutoff (Hz):", " Hz", { 1.0, 22050.0, 1.0, 1000.0, 1 }, {}, {}, {} },
            FieldSpec{ field::quality, field::design, FieldKind::Number, "Q Factor:", "", { 0.1, 20.0, 0.1, 0.707, 3 }, {}, {}, {} },
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

    IirForm::IirForm()
        : tableSpecs{ ui::model::TableSpec{ field::signal, shell::signalColumns, maximumComponents, "Add", "Remove" } }
        , tables{ ui::model::TableModel{ tableSpecs[0], cells } }
        , spec{ groups, fields, actions, tableSpecs }
        , model{ spec, values, tables }
    {
        shell::SeedFrom(defaultComponents, tables[0]);
    }

    ui::model::FormModel& IirForm::Model()
    {
        return model;
    }

    const ui::model::FormModel& IirForm::Model() const
    {
        return model;
    }

    IirFilterSimulator::Configuration IirForm::BuildConfiguration() const
    {
        IirFilterSimulator::Configuration config;

        config.filter.type = static_cast<FilterType>(model.SelectedData(field::filterType));
        config.filter.cutoffHz = model.Float(field::cutoff);
        config.filter.qualityFactor = model.Float(field::quality);
        config.filter.sampleRateHz = model.Float(field::sampleRate);

        config.simulation.sampleRateHz = config.filter.sampleRateHz;
        config.simulation.duration = model.Float(field::duration);
        config.simulation.signalComponents = shell::ToComponents(model.Table(0));

        return config;
    }
}
