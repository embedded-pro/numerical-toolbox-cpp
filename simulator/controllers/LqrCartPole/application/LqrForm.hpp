#pragma once

#include "simulator/controllers/LqrCartPole/application/LqrCartPoleSimulator.hpp"
#include "ui/model/FormModel.hpp"
#include <array>

namespace simulator::controllers::lqr
{
    namespace field
    {
        inline constexpr ui::model::FieldId cartMass{ 1 };
        inline constexpr ui::model::FieldId poleMass{ 2 };
        inline constexpr ui::model::FieldId poleLength{ 3 };
        inline constexpr ui::model::FieldId gravity{ 4 };
        inline constexpr ui::model::FieldId friction{ 5 };
        inline constexpr ui::model::FieldId trackLimit{ 6 };
        inline constexpr ui::model::FieldId weightPosition{ 7 };
        inline constexpr ui::model::FieldId weightVelocity{ 8 };
        inline constexpr ui::model::FieldId weightAngle{ 9 };
        inline constexpr ui::model::FieldId weightAngularRate{ 10 };
        inline constexpr ui::model::FieldId weightForce{ 11 };
        inline constexpr ui::model::FieldId timeStep{ 12 };
        inline constexpr ui::model::FieldId forceLimit{ 13 };
        inline constexpr ui::model::FieldId readOutPosition{ 14 };
        inline constexpr ui::model::FieldId readOutVelocity{ 15 };
        inline constexpr ui::model::FieldId readOutAngle{ 16 };
        inline constexpr ui::model::FieldId readOutAngularRate{ 17 };
        inline constexpr ui::model::FieldId readOutForce{ 18 };

        inline constexpr ui::model::GroupId plant{ 1 };
        inline constexpr ui::model::GroupId weights{ 2 };
        inline constexpr ui::model::GroupId simulation{ 3 };
        inline constexpr ui::model::GroupId state{ 4 };

        inline constexpr ui::model::ActionId configure{ 1 };
        inline constexpr ui::model::ActionId start{ 2 };
        inline constexpr ui::model::ActionId stop{ 3 };
        inline constexpr ui::model::ActionId reset{ 4 };
        inline constexpr ui::model::ActionId disturb{ 5 };
    }

    class LqrForm
    {
    public:
        LqrForm();

        [[nodiscard]] ui::model::FormModel& Model();
        [[nodiscard]] const ui::model::FormModel& Model() const;

        [[nodiscard]] LqrCartPoleConfig BuildConfiguration() const;

        void SetState(float position, float velocity, float angle, float angularRate, float force);

    private:
        static constexpr std::size_t fieldCount{ 18 };

        std::array<ui::model::FieldValue, fieldCount> values{};

        ui::model::FormSpec spec;
        ui::model::FormModel model;
    };
}
