#pragma once

#include "simulator/filters/FirFilter/application/FirFilterSimulator.hpp"
#include "simulator/shell/SignalComponents.hpp"
#include "ui/model/FormModel.hpp"
#include <array>

namespace simulator::filters::fir
{
    namespace field
    {
        inline constexpr ui::model::FieldId filterType{ 1 };
        inline constexpr ui::model::FieldId cutoff{ 2 };
        inline constexpr ui::model::FieldId cutoffHigh{ 3 };
        inline constexpr ui::model::FieldId order{ 4 };
        inline constexpr ui::model::FieldId sampleRate{ 5 };
        inline constexpr ui::model::FieldId duration{ 6 };

        inline constexpr ui::model::GroupId design{ 1 };
        inline constexpr ui::model::GroupId simulation{ 2 };
        inline constexpr ui::model::GroupId signal{ 3 };

        inline constexpr ui::model::ActionId compute{ 1 };
    }

    class FirForm
    {
    public:
        FirForm();

        [[nodiscard]] ui::model::FormModel& Model();
        [[nodiscard]] const ui::model::FormModel& Model() const;

        [[nodiscard]] FirFilterSimulator::Configuration BuildConfiguration() const;

    private:
        static constexpr std::size_t fieldCount{ 6 };
        static constexpr std::size_t maximumComponents{ 16 };

        std::array<ui::model::TableSpec, 1> tableSpecs;
        std::array<double, maximumComponents * 2> cells{};
        std::array<ui::model::TableModel, 1> tables;
        std::array<ui::model::FieldValue, fieldCount> values{};

        ui::model::FormSpec spec;
        ui::model::FormModel model;
    };
}
