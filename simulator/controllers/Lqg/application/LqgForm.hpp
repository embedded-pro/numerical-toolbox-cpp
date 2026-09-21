#pragma once

#include "simulator/controllers/Lqg/application/LqgSimulator.hpp"
#include "ui/model/FormModel.hpp"
#include <array>

namespace simulator::controllers::lqg
{
    namespace field
    {
        inline constexpr ui::model::FieldId stateWeight{ 1 };
        inline constexpr ui::model::FieldId controlWeight{ 2 };
        inline constexpr ui::model::FieldId processNoise{ 3 };
        inline constexpr ui::model::FieldId measurementNoise{ 4 };
        inline constexpr ui::model::FieldId sampleTime{ 5 };
        inline constexpr ui::model::FieldId duration{ 6 };
        inline constexpr ui::model::FieldId initialPosition{ 7 };

        inline constexpr ui::model::GroupId weights{ 1 };
        inline constexpr ui::model::GroupId noise{ 2 };
        inline constexpr ui::model::GroupId simulation{ 3 };

        inline constexpr ui::model::ActionId compute{ 1 };
    }

    class LqgForm
    {
    public:
        LqgForm();

        [[nodiscard]] ui::model::FormModel& Model();
        [[nodiscard]] const ui::model::FormModel& Model() const;

        [[nodiscard]] LqgSimulatorConfig BuildConfiguration() const;

        [[nodiscard]] LqgPlant CreatePlant() const;

    private:
        static constexpr std::size_t fieldCount{ 7 };

        std::array<ui::model::FieldValue, fieldCount> values{};
        ui::model::FormSpec spec;
        ui::model::FormModel model;
    };
}
