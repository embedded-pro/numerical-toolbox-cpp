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
        struct CurrentReferences
        {
            QNumberType directAxisCurrent;
            QNumberType quadratureAxisCurrent;
        };

        struct PhaseMeasurements
        {
            QNumberType phaseACurrent;
            QNumberType phaseBCurrent;
            QNumberType phaseCCurrent;
            QNumberType electricalAngle;
        };

        struct Configuration
        {
            typename Pid<QNumberType>::Tunnings currentTunnings;
            typename Pid<QNumberType>::Limits currentLimits;
        };

        FieldOrientedController(const Configuration& config,
            const math::TrigonometricFunctions<QNumberType>& trigFunctions,
            const math::AdvancedFunctions<QNumberType>& advancedFunctions)
            : clarkePark(trigFunctions, advancedFunctions)
            , svm(trigFunctions)
            , dAxisCurrentController(config.currentTunnings, config.currentLimits)
            , qAxisCurrentController(config.currentTunnings, config.currentLimits)
        {
        }

        typename SpaceVectorModulation<QNumberType>::Output Process(
            const CurrentReferences& references,
            const PhaseMeasurements& measurements)
        {
            auto dqCurrents = clarkePark.Forward({ measurements.phaseACurrent,
                                                     measurements.phaseBCurrent,
                                                     measurements.phaseCCurrent },
                measurements.electricalAngle);

            QNumberType vd = dAxisCurrentController.Process(dqCurrents.d);
            QNumberType vq = qAxisCurrentController.Process(dqCurrents.q);

            return svm.Generate({ vd, vq }, measurements.electricalAngle);
        }

        void SetCurrentReferences(const CurrentReferences& references)
        {
            dAxisCurrentController.SetPoint(references.directAxisCurrent);
            qAxisCurrentController.SetPoint(references.quadratureAxisCurrent);
        }

        void Reset()
        {
            dAxisCurrentController.Reset();
            qAxisCurrentController.Reset();
        }

    private:
        ClarkePark<QNumberType> clarkePark;
        SpaceVectorModulation<QNumberType> svm;
        Pid<QNumberType> dAxisCurrentController;
        Pid<QNumberType> qAxisCurrentController;
    };
}

#endif
