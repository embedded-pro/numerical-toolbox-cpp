#pragma once

#include "simulator/estimators/RecursiveLeastSquares/application/RlsSimulator.hpp"
#include "ui/model/FormModel.hpp"
#include <array>

namespace simulator::estimators::rls
{
    namespace field
    {
        inline constexpr ui::model::FieldId forgettingFactor{ 1 };
        inline constexpr ui::model::FieldId initialCovariance{ 2 };
        inline constexpr ui::model::FieldId sampleCount{ 3 };
        inline constexpr ui::model::FieldId noiseAmplitude{ 4 };
        inline constexpr ui::model::FieldId bias{ 5 };
        inline constexpr ui::model::FieldId firstCoefficient{ 6 };
        inline constexpr ui::model::FieldId secondCoefficient{ 7 };

        inline constexpr ui::model::GroupId parameters{ 1 };
        inline constexpr ui::model::GroupId coefficients{ 2 };

        inline constexpr ui::model::ActionId compute{ 1 };
    }

    class RlsForm
    {
    public:
        RlsForm();

        [[nodiscard]] ui::model::FormModel& Model();
        [[nodiscard]] const ui::model::FormModel& Model() const;

        [[nodiscard]] RlsSimulator::Configuration BuildConfiguration() const;

    private:
        static constexpr std::size_t fieldCount{ 7 };

        std::array<ui::model::FieldValue, fieldCount> values{};

        ui::model::FormSpec spec;
        ui::model::FormModel model;
    };
}
