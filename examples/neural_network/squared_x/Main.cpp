// #include "neural_network/Dense.hpp"
// #include "neural_network/L2.hpp"
// #include "neural_network/MeanSquaredError.hpp"
// #include "neural_network/Model.hpp"
// #include "optimizers/GradientDescent.hpp"
// #include <iostream>
// #include <random>
// #include <vector>

// namespace
// {
//     class ReLU
//         : public neural_network::ActivationFunction<float>
//     {
//     public:
//         float Forward(float x) const override
//         {
//             return std::max(0.0f, x);
//         }

//         float Backward(float x) const override
//         {
//             return x > 0.0f ? 1.0f : 0.0f;
//         }
//     };

//     std::pair<std::vector<float>, std::vector<float>> generateTrainingData(
//         size_t numSamples,
//         float xMin,
//         float xMax,
//         float noiseStdDev)
//     {
//         std::random_device rd;
//         std::mt19937 gen(rd());
//         std::uniform_real_distribution<float> xDist(xMin, xMax);
//         std::normal_distribution<float> noiseDist(0.0f, noiseStdDev);

//         std::vector<float> inputs;
//         std::vector<float> targets;
//         inputs.reserve(numSamples);
//         targets.reserve(numSamples);

//         for (size_t i = 0; i < numSamples; ++i)
//         {
//             float x = xDist(gen);
//             float y = x * x;
//             float noise = noiseDist(gen);

//             inputs.push_back(x);
//             targets.push_back(y + noise);
//         }

//         return { inputs, targets };
//     }
// }

// int main()
// {
//     neural_network::Model<float, 1, 1,
//         neural_network::Dense<float, 1, 4, ReLU>,
//         neural_network::Dense<float, 4, 1, ReLU>>
//         model;

//     const size_t numSamples = 200;
//     const float xMin = -2.0f;
//     const float xMax = 2.0f;
//     const float noiseStdDev = 0.1f;

//     auto [inputs, targets] = generateTrainingData(numSamples, xMin, xMax, noiseStdDev);

//     std::cout << "First 5 training samples (x, y):" << std::endl;
//     for (size_t i = 0; i < 5 && i < inputs.size(); ++i)
//         std::cout << inputs[i] << " -> " << targets[i] << std::endl;

//     neural_network::L2<float, model.TotalParameters> l2(0.01f);

//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::normal_distribution<float> paramDist(0.0f, 0.1f);

//     for (size_t i = 0; i < numSamples; ++i)
//     {
//         math::Vector<float, 1> input = { inputs[i] };
//         math::Vector<float, 1> target = { targets[i] };

//         neural_network::MeanSquaredError<float, model.TotalParameters> loss(target, l2);

//         optimizer::GradientDescent<float, model.TotalParameters>::Parameters params;
//         params.learningRate = 0.01f;
//         params.maxIterations = 50;
//         optimizer::GradientDescent<float, model.TotalParameters> optimizer(params);

//         math::Vector<float, model.TotalParameters> initialParameters;
//         for (size_t j = 0; j < model.TotalParameters; ++j)
//             initialParameters[j] = paramDist(gen);

//         model.Train(optimizer, loss, initialParameters);
//     }

//     std::cout << "\nTesting the trained model:" << std::endl;
//     std::vector<float> testInputs = { -1.5f, -0.5f, 0.0f, 0.5f, 1.5f };

//     for (float x : testInputs)
//     {
//         math::Vector<float, 1> input = { x };
//         auto prediction = model.Forward(input);
//         float expectedY = x * x;
//         std::cout << "Input: " << x
//                   << ", Prediction: " << prediction[0]
//                   << ", Expected: " << expectedY
//                   << ", Error: " << std::abs(prediction[0] - expectedY)
//                   << std::endl;
//     }

//     return 0;
// }

int main()
{
    return 0;
}
