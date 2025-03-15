#ifndef CONTROLLERS_FIELD_ORIENTED_CONTROLLER_HPP
#define CONTROLLERS_FIELD_ORIENTED_CONTROLLER_HPP

#include "numerical/controllers/Pid.hpp"
#include "numerical/controllers/SpaceVectorModulation.hpp"
#include "numerical/controllers/TransformsClarkePark.hpp"

namespace controllers
{
    template<typename QNumberType>
    class FieldOrientedController
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "FieldOrientedController can only be instantiated with math::QNumber or floating point types.");

    public:
        struct Configuration
        {
            typename Pid<QNumberType>::Tunnings currentTunnings;
            typename Pid<QNumberType>::Limits currentLimits;
        };

        FieldOrientedController(const Configuration& config,
            const math::TrigonometricFunctions<QNumberType>& trigFunctions,
            const math::AdvancedFunctions<QNumberType>& advancedFunctions)
            : clarke(advancedFunctions)
            , park(trigFunctions)
            , spaceVectorModulation(trigFunctions)
            , dAxisCurrentController(config.currentTunnings, config.currentLimits)
            , qAxisCurrentController(config.currentTunnings, config.currentLimits)
        {
            dAxisCurrentController.SetPoint(QNumberType{ 0.0f });
        }

        typename SpaceVectorModulation<QNumberType>::Output Process(const ThreePhase<QNumberType>& phaseCurrents, QNumberType electricalAngle)
        {
            auto dqCurrents = park.Forward(clarke.Forward(phaseCurrents), electricalAngle);

            QNumberType vd = dAxisCurrentController.Process(dqCurrents.d);
            QNumberType vq = qAxisCurrentController.Process(dqCurrents.q);

            auto voltagePhase = park.Inverse({ vd, vq }, electricalAngle);

            return spaceVectorModulation.Generate(voltagePhase);
        }

        void SetCurrentReference(QNumberType targetCurrent)
        {
            qAxisCurrentController.SetPoint(targetCurrent);
        }

        void Reset()
        {
            dAxisCurrentController.Reset();
            qAxisCurrentController.Reset();
        }

    private:
        Clarke<QNumberType> clarke;
        Park<QNumberType> park;
        SpaceVectorModulation<QNumberType> spaceVectorModulation;
        Pid<QNumberType> dAxisCurrentController;
        Pid<QNumberType> qAxisCurrentController;
    };
}

#endif
