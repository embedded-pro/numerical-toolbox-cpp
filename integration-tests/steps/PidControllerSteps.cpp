#include "cucumber_cpp/library/Context.hpp"
#include "cucumber_cpp/library/Steps.hpp"
#include "numerical/controllers/implementations/PidIncremental.hpp"
#include "numerical/controllers/interfaces/PidController.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

GIVEN(R"(a PID controller with gains kp {float} ki {float} kd {float})", (float kp, float ki, float kd))
{
    context.InsertAt("tunings", controllers::PidTunings<float>{ kp, ki, kd });
}

GIVEN(R"(output limits between {float} and {float})", (float min, float max))
{
    context.InsertAt("limits", controllers::PidLimits<float>{ min, max });
}

GIVEN(R"(a set point of {float})", (float setPoint))
{
    context.InsertAt("setPoint", setPoint);
}

WHEN(R"(a measurement of {float} is processed)", (float measurement))
{
    const auto& tunings = context.Get<controllers::PidTunings<float>>("tunings");
    const auto& limits = context.Get<controllers::PidLimits<float>>("limits");
    const auto& setPoint = context.Get<float>("setPoint");

    controllers::PidIncrementalSynchronous<float> controller{ tunings, limits };
    controller.Enable();
    controller.SetPoint(setPoint);

    context.InsertAt("output", controller.Process(measurement));
}

THEN(R"(the control output should be greater than {float})", (float threshold))
{
    EXPECT_THAT(context.Get<float>("output"), testing::Gt(threshold));
}

THEN(R"(the control output should be {float})", (float expected))
{
    EXPECT_THAT(context.Get<float>("output"), testing::FloatNear(expected, 1.0e-4f));
}
