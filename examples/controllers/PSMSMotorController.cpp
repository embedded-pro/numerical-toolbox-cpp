#include "controllers/FieldOrientedController.hpp"
#include <algorithm>
#include <iostream>

namespace
{
    template<typename QNumberType>
    class AdvancedFunctionsStub
        : public dsp::AdvancedFunctions<QNumberType>
    {
    public:
        QNumberType Modulus(const QNumberType& real, const QNumberType& imag) const override
        {
            really_assert(real > -1.0f && real < 1.0f);
            really_assert(imag > -1.0f && imag < 1.0f);
            return QNumberType(std::sqrt(std::pow(math::ToFloat(real), 2.0f) + std::pow(math::ToFloat(imag), 2.0f)));
        }

        QNumberType NaturalLogarithm(const QNumberType& value) const override
        {
            really_assert(value > -1.0f && value < 1.0f);
            return QNumberType(std::log(math::ToFloat(value)));
        }

        QNumberType SquareRoot(const QNumberType& value) const override
        {
            really_assert(value >= 0.0f && value < 1.0f);
            return QNumberType(std::sqrt(math::ToFloat(value)));
        }
    };

    template<typename QNumberType>
    class TrigonometricFunctionsStub
        : public dsp::TrigonometricFunctions<QNumberType>
    {
    public:
        constexpr static float _2_PI = 6.2831853f;
        constexpr static float MAX = 0.99999f;

        QNumberType Cosine(const QNumberType& angle) const override
        {
            really_assert(angle >= math::Lowest<QNumberType>() && angle <= math::Max<QNumberType>());
            return QNumberType(std::clamp(std::cos((math::ToFloat(angle) / MAX) * _2_PI), math::Lowest<QNumberType>(), math::Max<QNumberType>()));
        }

        QNumberType Sine(const QNumberType& angle) const override
        {
            really_assert(angle >= math::Lowest<QNumberType>() && angle <= math::Max<QNumberType>());
            return QNumberType(std::clamp(std::sin((math::ToFloat(angle) / MAX) * _2_PI), math::Lowest<QNumberType>(), math::Max<QNumberType>()));
        }

        QNumberType Arctangent(const QNumberType& angle) const override
        {
            really_assert(angle >= math::Lowest<QNumberType>() && angle <= math::Max<QNumberType>());
            return QNumberType(std::clamp(std::atan((math::ToFloat(angle) / MAX) * _2_PI), math::Lowest<QNumberType>(), math::Max<QNumberType>()));
        }

        QNumberType Phase(const QNumberType& real, const QNumberType& imag) const override
        {
            really_assert(real >= math::Lowest<QNumberType>() && real <= math::Max<QNumberType>());
            really_assert(imag >= math::Lowest<QNumberType>() && imag <= math::Max<QNumberType>());

            float atan2 = std::atan2((math::ToFloat(imag) / MAX) * _2_PI, (math::ToFloat(real) / MAX) * _2_PI);

            return QNumberType(std::clamp(atan2, math::Lowest<QNumberType>(), math::Max<QNumberType>()));
        }
    };

    template<typename QNumberType>
    class PmsmMotorModel
    {
    public:
        struct Parameters
        {
            QNumberType Rs;   // Stator resistance [Ohm]
            QNumberType Ld;   // D-axis inductance [H]
            QNumberType Lq;   // Q-axis inductance [H]
            QNumberType phiF; // Permanent magnet flux [Wb]
            QNumberType Ts;   // Sample time [s]
        };

        explicit PmsmMotorModel(const Parameters& params)
            : params(params)
            , id(0)
            , iq(0)
            , theta(0)
            , omega(0)
        {
        }

        // Process voltage inputs and produce phase currents
        controllers::ThreePhase<QNumberType> Process(const typename controllers::SpaceVectorModulation<QNumberType>::Output& voltage)
        {
            // Convert normalized voltages to actual voltages (assuming ±0.5 normalized to match FOC)
            QNumberType vd = voltage.a * 2; // Scale from ±0.5 to ±1
            QNumberType vq = voltage.b * 2;

            // Motor electrical model (simplified)
            QNumberType did = (vd - params.Rs * id + omega * params.Lq * iq) / params.Ld;
            QNumberType diq = (vq - params.Rs * iq - omega * (params.Ld * id + params.phiF)) / params.Lq;

            // Euler integration
            id += did * params.Ts;
            iq += diq * params.Ts;

            // Update angle and speed (simplified mechanical model)
            omega = iq * params.phiF * 0.5f; // Simplified speed calculation
            theta += omega * params.Ts;

            // Normalize theta to [0, 2π]
            while (theta >= 2 * M_PI)
                theta -= 2 * M_PI;
            while (theta < 0)
                theta += 2 * M_PI;

            // Convert dq currents to phase currents
            QNumberType cosTheta = std::cos(theta);
            QNumberType sinTheta = std::sin(theta);

            // Inverse Park transform
            QNumberType ialpha = id * cosTheta - iq * sinTheta;
            QNumberType ibeta = id * sinTheta + iq * cosTheta;

            // Inverse Clarke transform (normalize to ±0.5 for FOC)
            return controllers::ThreePhase<QNumberType>{
                QNumberType(ialpha * 0.5f),
                QNumberType(-0.5f * ialpha + 0.866f * ibeta) * 0.5f,
                QNumberType(-0.5f * ialpha - 0.866f * ibeta) * 0.5f
            };
        }

        QNumberType GetElectricalAngle() const
        {
            return theta / (2 * M_PI); // Normalize to [0,1] for FOC
        }

    private:
        Parameters params;
        QNumberType id;    // D-axis current
        QNumberType iq;    // Q-axis current
        QNumberType theta; // Electrical angle
        QNumberType omega; // Electrical speed
    };

    template<typename QNumberType>
    class PmsmMotorControllerSimulator
    {
    public:
        PmsmMotorControllerSimulator(
            const typename PmsmMotorModel<QNumberType>::Parameters& motorParams,
            const typename controllers::FieldOrientedController<QNumberType>::Configuration& focConfig)
            : motor(motorParams)
            , foc(focConfig, trigFunctions, advancedFunctions)
        {
        }

        void SetCurrentReferences(QNumberType id_ref, QNumberType iq_ref)
        {
            idRef = id_ref;
            iqRef = iq_ref;
        }

        void Step()
        {
            auto angle = motor.GetElectricalAngle();
            auto currents = motor.Process(lastVoltages);

            typename controllers::FieldOrientedController<QNumberType>::PhaseMeasurements measurements{
                currents.a,
                currents.b,
                currents.c,
                angle
            };

            typename controllers::FieldOrientedController<QNumberType>::CurrentReferences refs;
            refs.directAxisCurrent = idRef;
            refs.quadratureAxisCurrent = iqRef;

            lastVoltages = foc.Process(refs, measurements);

            std::cout << "Voltages (normalized ±0.5) - Va: " << lastVoltages.a
                      << " Vb: " << lastVoltages.b
                      << " Vc: " << lastVoltages.c << "\n";

            std::cout << "Currents (normalized ±0.5) - Ia: " << currents.a
                      << " Ib: " << currents.b
                      << " Ic: " << currents.c << "\n\n";
        }

    private:
        TrigonometricFunctionsStub<QNumberType> trigFunctions;
        AdvancedFunctionsStub<QNumberType> advancedFunctions;
        PmsmMotorModel<QNumberType> motor;
        controllers::FieldOrientedController<QNumberType> foc;
        typename controllers::SpaceVectorModulation<QNumberType>::Output lastVoltages{ 0, 0, 0 };
        QNumberType idRef{ 0 };
        QNumberType iqRef{ 0 };
    };
}

int main()
{
    PmsmMotorModel<float>::Parameters motorParams{
        .Rs = 0.5f,
        .Ld = 0.001f,
        .Lq = 0.001f,
        .phiF = 0.1f,
        .Ts = 0.0001f
    };

    controllers::FieldOrientedController<float>::Configuration focConfig{
        .currentTunnings = {
            .kp = 0.2f,
            .ki = 0.05f,
            .kd = 0.0f },
        .currentLimits = { .min = -0.5f, .max = 0.5f },
        .sampleTime = std::chrono::microseconds(100)
    };

    PmsmMotorControllerSimulator<float> simulator(motorParams, focConfig);

    simulator.SetCurrentReferences(0.0f, 0.2f);

    for (int i = 0; i < 100; i++)
        simulator.Step();
}
