#pragma once

#include "simulator/analysis/FastFourierTransform/application/FftSimulator.hpp"
#include "simulator/shell/SignalComponents.hpp"
#include "ui/model/FormModel.hpp"
#include <array>

namespace simulator::analysis
{
    namespace field
    {
        inline constexpr ui::model::FieldId fftSize{ 1 };
        inline constexpr ui::model::FieldId sampleRate{ 2 };
        inline constexpr ui::model::FieldId windowType{ 3 };

        inline constexpr ui::model::GroupId transform{ 1 };
        inline constexpr ui::model::GroupId signal{ 2 };

        inline constexpr ui::model::ActionId compute{ 1 };
    }

    class FftForm
    {
    public:
        FftForm();

        [[nodiscard]] ui::model::FormModel& Model();
        [[nodiscard]] const ui::model::FormModel& Model() const;

        [[nodiscard]] FftSimulator::Configuration BuildConfiguration() const;

    private:
        static constexpr std::size_t fieldCount{ 3 };
        static constexpr std::size_t maximumComponents{ 16 };

        std::array<ui::model::TableSpec, 1> tableSpecs;
        std::array<double, maximumComponents * 2> cells{};
        std::array<ui::model::TableModel, 1> tables;
        std::array<ui::model::FieldValue, fieldCount> values{};

        ui::model::FormSpec spec;
        ui::model::FormModel model;
    };
}
