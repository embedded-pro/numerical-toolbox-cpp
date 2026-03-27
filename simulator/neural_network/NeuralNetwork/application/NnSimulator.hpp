#pragma once

#include <cstddef>
#include <vector>

namespace simulator::neural_network::nn
{
    enum class DemoType
    {
        Xor,
        SineApproximation
    };

    struct NnConfig
    {
        DemoType demo = DemoType::Xor;
        std::size_t hiddenSize = 8;
        float learningRate = 0.1f;
        std::size_t epochs = 500;
        std::size_t sineSamples = 50;
    };

    struct NnResult
    {
        std::vector<float> epochIndex;
        std::vector<float> lossHistory;
        std::vector<float> predictionInputs;
        std::vector<float> predictionTargets;
        std::vector<float> predictionOutputs;
    };

    class NnSimulator
    {
    public:
        struct Configuration
        {
            NnConfig nn;
        };

        void Configure(const Configuration& config);
        NnResult Run();

    private:
        Configuration configuration;
    };
}
