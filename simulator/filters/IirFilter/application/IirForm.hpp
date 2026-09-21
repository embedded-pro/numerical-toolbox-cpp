#pragma once

#include "simulator/filters/IirFilter/application/IirFilterSimulator.hpp"
#include "simulator/shell/SignalComponents.hpp"
#include "ui/model/FormModel.hpp"
#include <array>

namespace simulator::filters::iir
{
    namespace field
    {
        inline constexpr ui::model::FieldId filterType{ 1 };
        inline constexpr ui::model::FieldId cutoff{ 2 };
        inline constexpr ui::model::FieldId quality{ 3 };
        inline constexpr ui::model::FieldId sampleRate{ 4 };
        inline constexpr ui::model::FieldId duration{ 5 };

        inline constexpr ui::model::GroupId design{ 1 };
        inline constexpr ui::model::GroupId simulation{ 2 };
        inline constexpr ui::model::GroupId signal{ 3 };

        inline constexpr ui::model::ActionId compute{ 1 };
    }

    class IirForm
    {
    public:
        IirForm();

        [[nodiscard]] ui::model::FormModel& Model();
        [[nodiscard]] const ui::model::FormModel& Model() const;

        [[nodiscard]] IirFilterSimulator::Configuration BuildConfiguration() const;

    private:
        static constexpr std::size_t fieldCount{ 5 };
        static constexpr std::size_t maximumComponents{ 16 };

        std::array<ui::model::TableSpec, 1> tableSpecs;
        std::array<double, maximumComponents * 2> cells{};
        std::array<ui::model::TableModel, 1> tables;
        std::array<ui::model::FieldValue, fieldCount> values{};

        ui::model::FormSpec spec;
        ui::model::FormModel model;
    };
}
