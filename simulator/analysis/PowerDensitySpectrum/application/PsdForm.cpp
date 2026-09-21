#include "simulator/analysis/PowerDensitySpectrum/application/PsdForm.hpp"

namespace simulator::analysis::psd
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::Condition;
        using ui::model::FieldKind;
        using ui::model::FieldSpec;
        using ui::model::GroupSpec;
        using ui::model::OptionSpec;

        constexpr std::array<OptionSpec, 5> segmentSizes{
            OptionSpec{ "64", 64 },
            OptionSpec{ "128", 128 },
            OptionSpec{ "256", 256 },
            OptionSpec{ "512", 512 },
            OptionSpec{ "1024", 1024 }
        };

        constexpr std::array<OptionSpec, 4> overlaps{
            OptionSpec{ "0%", 0 },
            OptionSpec{ "25%", 25 },
            OptionSpec{ "50%", 50 },
            OptionSpec{ "75%", 75 }
        };

        constexpr std::array<OptionSpec, 4> windowTypes{
            OptionSpec{ "Rectangular", static_cast<std::int64_t>(WindowType::Rectangular) },
            OptionSpec{ "Hamming", static_cast<std::int64_t>(WindowType::Hamming) },
            OptionSpec{ "Hanning", static_cast<std::int64_t>(WindowType::Hanning) },
            OptionSpec{ "Blackman", static_cast<std::int64_t>(WindowType::Blackman) }
        };

        constexpr std::array<OptionSpec, 2> noiseTypes{
            OptionSpec{ "None", static_cast<std::int64_t>(NoiseType::None) },
            OptionSpec{ "White Gaussian", static_cast<std::int64_t>(NoiseType::WhiteGaussian) }
        };

        constexpr Condition noiseSelected{ field::noiseType, 0b10 };

        constexpr std::array<GroupSpec, 3> groups{
            GroupSpec{ field::spectrum, "PSD Configuration", {} },
            GroupSpec{ field::noise, "Noise Configuration", {} },
            GroupSpec{ field::signal, "Signal Components", {} }
        };

        constexpr std::array<FieldSpec, 7> fields{
            FieldSpec{ field::inputSize, field::spectrum, FieldKind::Number, "Input Size:", "", { 64.0, 4096.0, 1.0, 1024.0, 0 }, {}, {}, {} },
            FieldSpec{ field::segmentSize, field::spectrum, FieldKind::Choice, "Segment Size:", "", {}, segmentSizes, {}, {} },
            FieldSpec{ field::overlap, field::spectrum, FieldKind::Choice, "Overlap (%):", "", {}, overlaps, {}, {} },
            FieldSpec{ field::sampleRate, field::spectrum, FieldKind::Number, "Sample Rate (Hz):", " Hz", { 100.0, 192000.0, 1.0, 44100.0, 1 }, {}, {}, {} },
            FieldSpec{ field::windowType, field::spectrum, FieldKind::Choice, "Window:", "", {}, windowTypes, {}, {} },
            FieldSpec{ field::noiseType, field::noise, FieldKind::Choice, "Noise Type:", "", {}, noiseTypes, {}, {} },
            FieldSpec{ field::noiseAmplitude, field::noise, FieldKind::Number, "Noise Amplitude:", "", { 0.0, 10.0, 0.01, 0.1, 4 }, {}, {}, noiseSelected }
        };

        constexpr std::array<ActionSpec, 1> actions{
            ActionSpec{ field::compute, "Compute PSD", ui::theme::ButtonRole::Primary, 40 }
        };

        constexpr std::array<utils::SignalComponent, 3> defaultComponents{
            utils::SignalComponent{ 1000.0f, 0.15f },
            utils::SignalComponent{ 5000.0f, 0.5f },
            utils::SignalComponent{ 12000.0f, 0.25f }
        };
    }

    PsdForm::PsdForm()
        : tableSpecs{ ui::model::TableSpec{ field::signal, shell::signalColumns, maximumComponents, "Add", "Remove" } }
        , tables{ ui::model::TableModel{ tableSpecs[0], cells } }
        , spec{ groups, fields, actions, tableSpecs }
        , model{ spec, values, tables }
    {
        model.SelectByData(field::segmentSize, 256);
        model.SelectByData(field::overlap, 50);
        model.SetSelection(field::windowType, 1);
        model.SetSelection(field::noiseType, 1);

        shell::SeedFrom(defaultComponents, tables[0]);
    }

    ui::model::FormModel& PsdForm::Model()
    {
        return model;
    }

    const ui::model::FormModel& PsdForm::Model() const
    {
        return model;
    }

    PsdSimulator::Configuration PsdForm::BuildConfiguration() const
    {
        PsdSimulator::Configuration config;

        config.inputSize = model.Count(field::inputSize);
        config.segmentSize = static_cast<std::size_t>(model.SelectedData(field::segmentSize));
        config.sampleRateHz = model.Float(field::sampleRate);
        config.windowType = static_cast<WindowType>(model.SelectedData(field::windowType));
        config.overlapPercent = static_cast<std::size_t>(model.SelectedData(field::overlap));
        config.noise.type = static_cast<NoiseType>(model.SelectedData(field::noiseType));
        config.noise.amplitude = model.Float(field::noiseAmplitude);
        config.signalComponents = shell::ToComponents(model.Table(0));

        return config;
    }
}
