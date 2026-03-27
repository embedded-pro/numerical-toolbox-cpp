#include "simulator/neural_network/NeuralNetwork/application/NnSimulator.hpp"
#include <cmath>
#include <numbers>
#include <random>
#include <vector>

namespace simulator::neural_network::nn
{
    namespace
    {
        struct TrainingData
        {
            std::vector<std::vector<float>> inputs;
            std::vector<float> targets;
        };

        TrainingData GenerateXorData()
        {
            TrainingData data;
            data.inputs = { { 0.0f, 0.0f }, { 0.0f, 1.0f }, { 1.0f, 0.0f }, { 1.0f, 1.0f } };
            data.targets = { 0.0f, 1.0f, 1.0f, 0.0f };
            return data;
        }

        TrainingData GenerateSineData(std::size_t samples)
        {
            TrainingData data;
            data.inputs.resize(samples);
            data.targets.resize(samples);

            for (std::size_t i = 0; i < samples; ++i)
            {
                float x = static_cast<float>(i) / static_cast<float>(samples - 1) * 2.0f * std::numbers::pi_v<float>;
                data.inputs[i] = { x / (2.0f * std::numbers::pi_v<float>) };
                data.targets[i] = (std::sin(x) + 1.0f) / 2.0f;
            }
            return data;
        }

        float Sigmoid(float x)
        {
            return 1.0f / (1.0f + std::exp(-x));
        }

        float SigmoidDerivative(float output)
        {
            return output * (1.0f - output);
        }

        float TanhActivation(float x)
        {
            return std::tanh(x);
        }

        float TanhDerivative(float output)
        {
            return 1.0f - output * output;
        }

        struct DenseLayer
        {
            std::vector<std::vector<float>> weights;
            std::vector<float> biases;
            std::vector<float> preActivation;
            std::vector<float> output;
            std::vector<float> input;
            std::vector<std::vector<float>> weightGrad;
            std::vector<float> biasGrad;

            DenseLayer(std::size_t inputSize, std::size_t outputSize, std::mt19937& rng)
            {
                float scale = std::sqrt(2.0f / static_cast<float>(inputSize));
                std::normal_distribution<float> dist(0.0f, scale);

                weights.resize(outputSize, std::vector<float>(inputSize));
                biases.resize(outputSize, 0.0f);
                preActivation.resize(outputSize);
                output.resize(outputSize);
                weightGrad.resize(outputSize, std::vector<float>(inputSize, 0.0f));
                biasGrad.resize(outputSize, 0.0f);

                for (auto& row : weights)
                    for (auto& w : row)
                        w = dist(rng);
            }

            void Forward(const std::vector<float>& x, bool useTanh)
            {
                input = x;
                for (std::size_t i = 0; i < weights.size(); ++i)
                {
                    preActivation[i] = biases[i];
                    for (std::size_t j = 0; j < input.size(); ++j)
                        preActivation[i] += weights[i][j] * input[j];

                    output[i] = useTanh ? TanhActivation(preActivation[i]) : Sigmoid(preActivation[i]);
                }
            }

            std::vector<float> Backward(const std::vector<float>& outputGrad, bool useTanh)
            {
                std::vector<float> inputGrad(input.size(), 0.0f);

                for (std::size_t i = 0; i < weights.size(); ++i)
                {
                    float derivative = useTanh ? TanhDerivative(output[i]) : SigmoidDerivative(output[i]);
                    float delta = outputGrad[i] * derivative;

                    for (std::size_t j = 0; j < input.size(); ++j)
                    {
                        weightGrad[i][j] += delta * input[j];
                        inputGrad[j] += delta * weights[i][j];
                    }
                    biasGrad[i] += delta;
                }
                return inputGrad;
            }

            void ApplyGradients(float lr, std::size_t batchSize)
            {
                float scale = lr / static_cast<float>(batchSize);
                for (std::size_t i = 0; i < weights.size(); ++i)
                {
                    for (std::size_t j = 0; j < weights[i].size(); ++j)
                    {
                        weights[i][j] -= scale * weightGrad[i][j];
                        weightGrad[i][j] = 0.0f;
                    }
                    biases[i] -= scale * biasGrad[i];
                    biasGrad[i] = 0.0f;
                }
            }
        };

        NnResult TrainNetwork(const NnConfig& config, const TrainingData& data)
        {
            std::mt19937 rng(42);

            std::size_t inputSize = data.inputs[0].size();
            std::size_t hiddenSize = config.hiddenSize;

            DenseLayer hidden(inputSize, hiddenSize, rng);
            DenseLayer output(hiddenSize, 1, rng);

            NnResult result;
            result.epochIndex.resize(config.epochs);
            result.lossHistory.resize(config.epochs);

            for (std::size_t epoch = 0; epoch < config.epochs; ++epoch)
            {
                float epochLoss = 0.0f;

                for (std::size_t s = 0; s < data.inputs.size(); ++s)
                {
                    hidden.Forward(data.inputs[s], true);
                    output.Forward(hidden.output, false);

                    float error = output.output[0] - data.targets[s];
                    epochLoss += error * error;

                    std::vector<float> outputGrad = { error };
                    auto hiddenGrad = output.Backward(outputGrad, false);
                    hidden.Backward(hiddenGrad, true);
                }

                hidden.ApplyGradients(config.learningRate, data.inputs.size());
                output.ApplyGradients(config.learningRate, data.inputs.size());

                result.epochIndex[epoch] = static_cast<float>(epoch);
                result.lossHistory[epoch] = epochLoss / static_cast<float>(data.inputs.size());
            }

            result.predictionInputs.resize(data.inputs.size());
            result.predictionTargets = data.targets;
            result.predictionOutputs.resize(data.inputs.size());

            for (std::size_t s = 0; s < data.inputs.size(); ++s)
            {
                result.predictionInputs[s] = data.inputs[s][0];
                hidden.Forward(data.inputs[s], true);
                output.Forward(hidden.output, false);
                result.predictionOutputs[s] = output.output[0];
            }

            return result;
        }
    }

    void NnSimulator::Configure(const Configuration& config)
    {
        configuration = config;
    }

    NnResult NnSimulator::Run()
    {
        TrainingData data;

        switch (configuration.nn.demo)
        {
        case DemoType::Xor:
            data = GenerateXorData();
            break;
        case DemoType::SineApproximation:
            data = GenerateSineData(configuration.nn.sineSamples);
            break;
        }

        return TrainNetwork(configuration.nn, data);
    }
}
