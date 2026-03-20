#pragma once

#include <complex>
#include <vector>

namespace simulator::controllers
{
    struct TransferFunction
    {
        std::vector<float> numerator;
        std::vector<float> denominator;
    };

    class Plant
    {
    public:
        virtual ~Plant() = default;

        virtual TransferFunction GetTransferFunction() const = 0;
        virtual float Output() const = 0;
        virtual void Step(float input, float dt) = 0;
        virtual void Reset() = 0;
        virtual std::vector<std::complex<float>> Poles() const = 0;
    };

    class FirstOrderPlant
        : public Plant
    {
    public:
        FirstOrderPlant(float gain, float timeConstant);

        TransferFunction GetTransferFunction() const override;
        float Output() const override;
        void Step(float input, float dt) override;
        void Reset() override;
        std::vector<std::complex<float>> Poles() const override;

    private:
        float gain;
        float timeConstant;
        float state = 0.0f;
    };

    class SecondOrderPlant
        : public Plant
    {
    public:
        SecondOrderPlant(float naturalFrequency, float dampingRatio);

        TransferFunction GetTransferFunction() const override;
        float Output() const override;
        void Step(float input, float dt) override;
        void Reset() override;
        std::vector<std::complex<float>> Poles() const override;

    private:
        float naturalFrequency;
        float dampingRatio;
        float x1 = 0.0f;
        float x2 = 0.0f;
    };
}
