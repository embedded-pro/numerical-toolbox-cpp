#pragma once

#include "simulator/controllers/BayesianMpcCalibration/application/BayesianMpcCalibrationSimulator.hpp"
#include "ui/model/FormModel.hpp"
#include <array>

namespace simulator::controllers::bayesian
{
    namespace field
    {
        inline constexpr ui::model::FieldId timeStep{ 1 };
        inline constexpr ui::model::FieldId processNoise{ 2 };
        inline constexpr ui::model::FieldId measurementNoise{ 3 };
        inline constexpr ui::model::FieldId expectationIterations{ 4 };
        inline constexpr ui::model::FieldId optimisationIterations{ 5 };

        inline constexpr ui::model::GroupId configuration{ 1 };

        inline constexpr ui::model::ActionId run{ 1 };
    }

    class BayesianMpcForm
    {
    public:
        BayesianMpcForm();

        [[nodiscard]] ui::model::FormModel& Model();
        [[nodiscard]] const ui::model::FormModel& Model() const;

        [[nodiscard]] CalibrationSimulationConfig BuildConfiguration() const;

    private:
        static constexpr std::size_t fieldCount{ 5 };

        std::array<ui::model::FieldValue, fieldCount> values{};

        ui::model::FormSpec spec;
        ui::model::FormModel model;
    };
}
