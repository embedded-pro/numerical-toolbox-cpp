#pragma once

#include "numerical/math/Matrix.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace simulator::controllers
{
    struct DoubleIntegratorConfig
    {
        float dt = 0.05f;
        float sigmaQ = 0.01f;
        float sigmaR = 0.5f;
    };

    class DoubleIntegratorPlant
    {
    public:
        static constexpr std::size_t MaxSteps = 200;

        using StateVector = math::Vector<float, 2>;
        using InputVector = math::Vector<float, 1>;
        using MeasurementVector = math::Vector<float, 1>;

        explicit DoubleIntegratorPlant(const DoubleIntegratorConfig& config);

        [[nodiscard]] math::SquareMatrix<float, 2> GetA() const;
        [[nodiscard]] math::Matrix<float, 2, 1> GetB() const;

        void GenerateObservations(
            std::size_t numSteps,
            uint32_t seed,
            std::array<MeasurementVector, MaxSteps>& observations,
            std::vector<float>& truePositions) const;

    private:
        math::SquareMatrix<float, 2> A_;
        math::Matrix<float, 2, 1> B_;
        DoubleIntegratorConfig config_;
    };
}
