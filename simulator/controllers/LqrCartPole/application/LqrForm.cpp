#include "simulator/controllers/LqrCartPole/application/LqrForm.hpp"
#include <numbers>

namespace simulator::controllers::lqr
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::FieldKind;
        using ui::model::FieldSpec;
        using ui::model::GroupSpec;

        constexpr float radiansToDegrees{ 180.0f / std::numbers::pi_v<float> };

        constexpr std::array<GroupSpec, 4> groups{
            GroupSpec{ field::plant, "Cart-Pole Plant", {} },
            GroupSpec{ field::weights, "LQR Weights (Q, R)", {} },
            GroupSpec{ field::simulation, "Simulation", {} },
            GroupSpec{ field::state, "State", {} }
        };

        constexpr std::array<FieldSpec, 18> fields{
            FieldSpec{ field::cartMass, field::plant, FieldKind::Number, "Cart Mass:", " kg", { 0.1, 50.0, 1.0, 1.0, 2 }, {}, {}, {} },
            FieldSpec{ field::poleMass, field::plant, FieldKind::Number, "Pole Mass:", " kg", { 0.01, 10.0, 1.0, 0.1, 3 }, {}, {}, {} },
            FieldSpec{ field::poleLength, field::plant, FieldKind::Number, "Pole Length:", " m", { 0.1, 5.0, 1.0, 0.5, 2 }, {}, {}, {} },
            FieldSpec{ field::gravity, field::plant, FieldKind::Number, "Gravity:", " m/s²", { 1.0, 20.0, 1.0, 9.81, 2 }, {}, {}, {} },
            FieldSpec{ field::friction, field::plant, FieldKind::Number, "Friction:", "", { 0.0, 5.0, 1.0, 0.1, 2 }, {}, {}, {} },
            FieldSpec{ field::trackLimit, field::plant, FieldKind::Number, "Track Limit:", " m", { 0.5, 10.0, 1.0, 2.4, 1 }, {}, {}, {} },
            FieldSpec{ field::weightPosition, field::weights, FieldKind::Number, "Q(x):", "", { 0.001, 1000.0, 1.0, 1.0, 3 }, {}, {}, {} },
            FieldSpec{ field::weightVelocity, field::weights, FieldKind::Number, "Q(ẋ):", "", { 0.001, 1000.0, 1.0, 1.0, 3 }, {}, {}, {} },
            FieldSpec{ field::weightAngle, field::weights, FieldKind::Number, "Q(θ):", "", { 0.001, 10000.0, 1.0, 100.0, 1 }, {}, {}, {} },
            FieldSpec{ field::weightAngularRate, field::weights, FieldKind::Number, "Q(θ̇):", "", { 0.001, 1000.0, 1.0, 10.0, 2 }, {}, {}, {} },
            FieldSpec{ field::weightForce, field::weights, FieldKind::Number, "R(F):", "", { 0.0001, 100.0, 1.0, 0.01, 4 }, {}, {}, {} },
            FieldSpec{ field::timeStep, field::simulation, FieldKind::Number, "Time Step:", " s", { 0.001, 0.1, 1.0, 0.01, 3 }, {}, {}, {} },
            FieldSpec{ field::forceLimit, field::simulation, FieldKind::Number, "Force Limit:", " N", { 1.0, 500.0, 1.0, 50.0, 1 }, {}, {}, {} },
            FieldSpec{ field::readOutPosition, field::state, FieldKind::ReadOut, "x:", " m", { 0.0, 0.0, 0.0, 0.0, 3 }, {}, {}, {} },
            FieldSpec{ field::readOutVelocity, field::state, FieldKind::ReadOut, "v:", " m/s", { 0.0, 0.0, 0.0, 0.0, 3 }, {}, {}, {} },
            FieldSpec{ field::readOutAngle, field::state, FieldKind::ReadOut, "θ:", "°", { 0.0, 0.0, 0.0, 0.0, 2 }, {}, {}, {} },
            FieldSpec{ field::readOutAngularRate, field::state, FieldKind::ReadOut, "ω:", "°/s", { 0.0, 0.0, 0.0, 0.0, 2 }, {}, {}, {} },
            FieldSpec{ field::readOutForce, field::state, FieldKind::ReadOut, "Control:", " N", { 0.0, 0.0, 0.0, 0.0, 2 }, {}, {}, {} }
        };

        constexpr std::array<ActionSpec, 5> actions{
            ActionSpec{ field::configure, "Apply Configuration", ui::theme::ButtonRole::Primary, 35 },
            ActionSpec{ field::start, "Start", ui::theme::ButtonRole::Start, 35 },
            ActionSpec{ field::stop, "Stop", ui::theme::ButtonRole::Stop, 35 },
            ActionSpec{ field::reset, "Reset", ui::theme::ButtonRole::Reset, 35 },
            ActionSpec{ field::disturb, "Disturb", ui::theme::ButtonRole::Default, 35 }
        };
    }

    LqrForm::LqrForm()
        : spec{ groups, fields, actions, {} }
        , model{ spec, values, {} }
    {}

    ui::model::FormModel& LqrForm::Model()
    {
        return model;
    }

    const ui::model::FormModel& LqrForm::Model() const
    {
        return model;
    }

    LqrCartPoleConfig LqrForm::BuildConfiguration() const
    {
        LqrCartPoleConfig config;

        config.plantParams.cartMass = model.Float(field::cartMass);
        config.plantParams.poleMass = model.Float(field::poleMass);
        config.plantParams.poleLength = model.Float(field::poleLength);
        config.plantParams.gravity = model.Float(field::gravity);
        config.plantParams.cartFriction = model.Float(field::friction);
        config.plantParams.trackLimit = model.Float(field::trackLimit);

        config.weights.qX = model.Float(field::weightPosition);
        config.weights.qXDot = model.Float(field::weightVelocity);
        config.weights.qTheta = model.Float(field::weightAngle);
        config.weights.qThetaDot = model.Float(field::weightAngularRate);
        config.weights.rForce = model.Float(field::weightForce);

        config.simulation.dt = model.Float(field::timeStep);
        config.simulation.forceLimit = model.Float(field::forceLimit);

        return config;
    }

    void LqrForm::SetState(float position, float velocity, float angle, float angularRate, float force)
    {
        model.SetNumber(field::readOutPosition, static_cast<double>(position));
        model.SetNumber(field::readOutVelocity, static_cast<double>(velocity));
        model.SetNumber(field::readOutAngle, static_cast<double>(angle * radiansToDegrees));
        model.SetNumber(field::readOutAngularRate, static_cast<double>(angularRate * radiansToDegrees));
        model.SetNumber(field::readOutForce, static_cast<double>(force));
    }
}
