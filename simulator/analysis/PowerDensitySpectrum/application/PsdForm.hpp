#pragma once

#include "simulator/analysis/PowerDensitySpectrum/application/PsdSimulator.hpp"
#include "simulator/shell/SignalComponents.hpp"
#include "ui/model/FormModel.hpp"
#include <array>

namespace simulator::analysis::psd
{
    namespace field
    {
        inline constexpr ui::model::FieldId inputSize{ 1 };
        inline constexpr ui::model::FieldId segmentSize{ 2 };
        inline constexpr ui::model::FieldId overlap{ 3 };
        inline constexpr ui::model::FieldId sampleRate{ 4 };
        inline constexpr ui::model::FieldId windowType{ 5 };
        inline constexpr ui::model::FieldId noiseType{ 6 };
        inline constexpr ui::model::FieldId noiseAmplitude{ 7 };

        inline constexpr ui::model::GroupId spectrum{ 1 };
        inline constexpr ui::model::GroupId noise{ 2 };
        inline constexpr ui::model::GroupId signal{ 3 };

        inline constexpr ui::model::ActionId compute{ 1 };
    }

    class PsdForm
    {
    public:
        PsdForm();

        [[nodiscard]] ui::model::FormModel& Model();
        [[nodiscard]] const ui::model::FormModel& Model() const;

        [[nodiscard]] PsdSimulator::Configuration BuildConfiguration() const;

    private:
        static constexpr std::size_t fieldCount{ 7 };
        static constexpr std::size_t maximumComponents{ 16 };

        std::array<ui::model::TableSpec, 1> tableSpecs;
        std::array<double, maximumComponents * 2> cells{};
        std::array<ui::model::TableModel, 1> tables;
        std::array<ui::model::FieldValue, fieldCount> values{};

        ui::model::FormSpec spec;
        ui::model::FormModel model;
    };
}
