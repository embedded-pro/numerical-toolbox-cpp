#include "simulator/analysis/FastFourierTransform/application/FftForm.hpp"

namespace simulator::analysis
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::FieldKind;
        using ui::model::FieldSpec;
        using ui::model::GroupSpec;
        using ui::model::OptionSpec;

        constexpr std::array<OptionSpec, 7> fftSizes{
            OptionSpec{ "64", 64 },
            OptionSpec{ "128", 128 },
            OptionSpec{ "256", 256 },
            OptionSpec{ "512", 512 },
            OptionSpec{ "1024", 1024 },
            OptionSpec{ "2048", 2048 },
            OptionSpec{ "4096", 4096 }
        };

        constexpr std::array<OptionSpec, 4> windowTypes{
            OptionSpec{ "Rectangular", static_cast<std::int64_t>(WindowType::Rectangular) },
            OptionSpec{ "Hamming", static_cast<std::int64_t>(WindowType::Hamming) },
            OptionSpec{ "Hanning", static_cast<std::int64_t>(WindowType::Hanning) },
            OptionSpec{ "Blackman", static_cast<std::int64_t>(WindowType::Blackman) }
        };

        constexpr std::array<GroupSpec, 2> groups{
            GroupSpec{ field::transform, "FFT Configuration", {} },
            GroupSpec{ field::signal, "Signal Components", {} }
        };

        constexpr std::array<FieldSpec, 3> fields{
            FieldSpec{ field::fftSize, field::transform, FieldKind::Choice, "FFT Size:", "", {}, fftSizes, {}, {} },
            FieldSpec{ field::sampleRate, field::transform, FieldKind::Number, "Sample Rate (Hz):", " Hz", { 100.0, 192000.0, 1.0, 44100.0, 1 }, {}, {}, {} },
            FieldSpec{ field::windowType, field::transform, FieldKind::Choice, "Window:", "", {}, windowTypes, {}, {} }
        };

        constexpr std::array<ActionSpec, 1> actions{
            ActionSpec{ field::compute, "Compute FFT", ui::theme::ButtonRole::Primary, 40 }
        };

        constexpr std::array<utils::SignalComponent, 3> defaultComponents{
            utils::SignalComponent{ 1000.0f, 0.15f },
            utils::SignalComponent{ 5000.0f, 0.5f },
            utils::SignalComponent{ 12000.0f, 0.25f }
        };
    }

    FftForm::FftForm()
        : tableSpecs{ ui::model::TableSpec{ field::signal, shell::signalColumns, maximumComponents, "Add", "Remove" } }
        , tables{ ui::model::TableModel{ tableSpecs[0], cells } }
        , spec{ groups, fields, actions, tableSpecs }
        , model{ spec, values, tables }
    {
        model.SelectByData(field::fftSize, 1024);
        shell::SeedFrom(defaultComponents, tables[0]);
    }

    ui::model::FormModel& FftForm::Model()
    {
        return model;
    }

    const ui::model::FormModel& FftForm::Model() const
    {
        return model;
    }

    FftSimulator::Configuration FftForm::BuildConfiguration() const
    {
        FftSimulator::Configuration config;

        config.fftSize = static_cast<std::size_t>(model.SelectedData(field::fftSize));
        config.sampleRateHz = model.Float(field::sampleRate);
        config.windowType = static_cast<WindowType>(model.SelectedData(field::windowType));
        config.signalComponents = shell::ToComponents(model.Table(0));

        return config;
    }
}
