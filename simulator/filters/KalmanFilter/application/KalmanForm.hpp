#pragma once

#include "simulator/filters/KalmanFilter/application/KalmanFilterSimulator.hpp"
#include "ui/model/FormModel.hpp"
#include <array>

namespace simulator::filters::kalman
{
    namespace field
    {
        inline constexpr ui::model::FieldId duration{ 1 };
        inline constexpr ui::model::FieldId timeStep{ 2 };
        inline constexpr ui::model::FieldId initialAngle{ 3 };
        inline constexpr ui::model::FieldId initialAngularRate{ 4 };
        inline constexpr ui::model::FieldId measurementNoise{ 5 };
        inline constexpr ui::model::FieldId processNoise{ 6 };
        inline constexpr ui::model::FieldId length{ 7 };
        inline constexpr ui::model::FieldId mass{ 8 };
        inline constexpr ui::model::FieldId damping{ 9 };

        inline constexpr ui::model::GroupId simulation{ 1 };
        inline constexpr ui::model::GroupId noise{ 2 };
        inline constexpr ui::model::GroupId pendulum{ 3 };

        inline constexpr ui::model::ActionId compute{ 1 };
    }

    class KalmanForm
    {
    public:
        KalmanForm();

        [[nodiscard]] ui::model::FormModel& Model();
        [[nodiscard]] const ui::model::FormModel& Model() const;

        [[nodiscard]] SimulationConfig BuildConfiguration() const;

    private:
        static constexpr std::size_t fieldCount{ 9 };

        std::array<ui::model::FieldValue, fieldCount> values{};

        ui::model::FormSpec spec;
        ui::model::FormModel model;
    };
}
