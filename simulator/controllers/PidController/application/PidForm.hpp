#pragma once

#include "simulator/controllers/PidController/application/PidSimulator.hpp"
#include "simulator/controllers/common/Plant.hpp"
#include "ui/model/FormModel.hpp"
#include <array>
#include <memory>
#include <string_view>

namespace simulator::controllers::pid
{
    namespace field
    {
        inline constexpr ui::model::FieldId kp{ 1 };
        inline constexpr ui::model::FieldId ki{ 2 };
        inline constexpr ui::model::FieldId kd{ 3 };
        inline constexpr ui::model::FieldId plantOrder{ 4 };
        inline constexpr ui::model::FieldId gain{ 5 };
        inline constexpr ui::model::FieldId timeConstant{ 6 };
        inline constexpr ui::model::FieldId naturalFrequency{ 7 };
        inline constexpr ui::model::FieldId dampingRatio{ 8 };
        inline constexpr ui::model::FieldId duration{ 9 };
        inline constexpr ui::model::FieldId sampleTime{ 10 };
        inline constexpr ui::model::FieldId outputMinimum{ 11 };
        inline constexpr ui::model::FieldId outputMaximum{ 12 };

        inline constexpr ui::model::GroupId tuning{ 1 };
        inline constexpr ui::model::GroupId plant{ 2 };
        inline constexpr ui::model::GroupId firstOrder{ 3 };
        inline constexpr ui::model::GroupId secondOrder{ 4 };
        inline constexpr ui::model::GroupId simulation{ 5 };

        inline constexpr ui::model::ActionId compute{ 1 };
    }

    class PidForm
    {
    public:
        PidForm();

        [[nodiscard]] ui::model::FormModel& Model();
        [[nodiscard]] const ui::model::FormModel& Model() const;

        [[nodiscard]] PidSimulator::Configuration BuildConfiguration() const;
        [[nodiscard]] std::unique_ptr<Plant> CreatePlant() const;
        [[nodiscard]] std::string_view PlantDescription() const;

        void SetProportionalGain(float value);

    private:
        static constexpr std::size_t fieldCount{ 12 };

        std::array<ui::model::FieldValue, fieldCount> values{};

        ui::model::FormSpec spec;
        ui::model::FormModel model;
    };
}
