#pragma once

#include "simulator/controllers/Mpc/application/MpcSimulator.hpp"
#include "simulator/controllers/Mpc/application/StateSpacePlant.hpp"
#include "ui/model/FormModel.hpp"
#include <array>
#include <string_view>

namespace simulator::controllers::mpc
{
    namespace field
    {
        inline constexpr ui::model::FieldId stateWeight{ 1 };
        inline constexpr ui::model::FieldId controlWeight{ 2 };
        inline constexpr ui::model::FieldId constraintsEnabled{ 3 };
        inline constexpr ui::model::FieldId controlMinimum{ 4 };
        inline constexpr ui::model::FieldId controlMaximum{ 5 };
        inline constexpr ui::model::FieldId plantType{ 6 };
        inline constexpr ui::model::FieldId plantGain{ 7 };
        inline constexpr ui::model::FieldId plantTimeConstant{ 8 };
        inline constexpr ui::model::FieldId duration{ 9 };
        inline constexpr ui::model::FieldId sampleTime{ 10 };
        inline constexpr ui::model::FieldId reference{ 11 };

        inline constexpr ui::model::GroupId weights{ 1 };
        inline constexpr ui::model::GroupId constraints{ 2 };
        inline constexpr ui::model::GroupId plant{ 3 };
        inline constexpr ui::model::GroupId plantParameters{ 4 };
        inline constexpr ui::model::GroupId simulation{ 5 };

        inline constexpr ui::model::ActionId compute{ 1 };
    }

    class MpcForm
    {
    public:
        MpcForm();

        [[nodiscard]] ui::model::FormModel& Model();
        [[nodiscard]] const ui::model::FormModel& Model() const;

        [[nodiscard]] MpcSimulator::Configuration BuildConfiguration() const;
        [[nodiscard]] math::LinearTimeInvariant<float, 2, 1> CreatePlant() const;
        [[nodiscard]] std::string_view PlantDescription() const;

    private:
        static constexpr std::size_t fieldCount{ 11 };

        std::array<ui::model::FieldValue, fieldCount> values{};

        ui::model::FormSpec spec;
        ui::model::FormModel model;
    };
}
