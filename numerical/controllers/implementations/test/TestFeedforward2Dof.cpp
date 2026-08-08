#include "numerical/controllers/implementations/Feedforward2Dof.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace
{
    class MockFeedforward
        : public controllers::Feedforward<float>
    {
    public:
        MOCK_METHOD(float, Evaluate, (float), (const, override));
    };

    class MockFeedbackLaw
        : public controllers::FeedbackLaw<float>
    {
    public:
        MOCK_METHOD(float, Process, (float), (override));
        MOCK_METHOD(void, Reset, (), (override));
    };

    class TestFeedforward2Dof
        : public ::testing::Test
    {
    public:
        testing::StrictMock<MockFeedforward> ff;
        testing::StrictMock<MockFeedbackLaw> fb;
        controllers::Saturation<float> clamp{ -1.0f, 1.0f };
        controllers::Feedforward2Dof<float> controller{ ff, fb, clamp };
    };
}

TEST_F(TestFeedforward2Dof, sums_feedforward_and_feedback)
{
    EXPECT_CALL(ff, Evaluate(0.5f)).WillOnce(testing::Return(0.3f));
    EXPECT_CALL(fb, Process(0.4f)).WillOnce(testing::Return(0.2f));

    float result{ controller.Compute(0.5f, 0.1f) };

    EXPECT_NEAR(result, 0.5f, math::Tolerance<float>());
}

TEST_F(TestFeedforward2Dof, passes_correct_error_to_feedback)
{
    EXPECT_CALL(ff, Evaluate(0.4f)).WillOnce(testing::Return(0.0f));
    EXPECT_CALL(fb, Process(0.3f)).WillOnce(testing::Return(0.3f));

    float result{ controller.Compute(0.4f, 0.1f) };

    EXPECT_NEAR(result, 0.3f, math::Tolerance<float>());
}

TEST_F(TestFeedforward2Dof, passes_reference_to_feedforward)
{
    EXPECT_CALL(ff, Evaluate(0.6f)).WillOnce(testing::Return(0.0f));
    EXPECT_CALL(fb, Process(testing::_)).WillOnce(testing::Return(0.0f));

    controller.Compute(0.6f, 0.0f);
}

TEST_F(TestFeedforward2Dof, zero_feedforward_reduces_to_feedback)
{
    EXPECT_CALL(ff, Evaluate(testing::_)).WillOnce(testing::Return(0.0f));
    EXPECT_CALL(fb, Process(testing::_)).WillOnce(testing::Return(0.4f));

    float result{ controller.Compute(0.5f, 0.1f) };

    EXPECT_NEAR(result, 0.4f, math::Tolerance<float>());
}

TEST_F(TestFeedforward2Dof, output_is_clamped)
{
    EXPECT_CALL(ff, Evaluate(testing::_)).WillOnce(testing::Return(0.9f));
    EXPECT_CALL(fb, Process(testing::_)).WillOnce(testing::Return(0.9f));

    float result{ controller.Compute(1.0f, 0.0f) };

    EXPECT_NEAR(result, 1.0f, math::Tolerance<float>());
}

TEST_F(TestFeedforward2Dof, perfect_feedforward_zero_error)
{
    EXPECT_CALL(ff, Evaluate(0.7f)).WillOnce(testing::Return(0.7f));
    EXPECT_CALL(fb, Process(0.0f)).WillOnce(testing::Return(0.0f));

    float result{ controller.Compute(0.7f, 0.7f) };

    EXPECT_NEAR(result, 0.7f, math::Tolerance<float>());
}

TEST_F(TestFeedforward2Dof, output_is_clamped_negative)
{
    EXPECT_CALL(ff, Evaluate(testing::_)).WillOnce(testing::Return(-0.9f));
    EXPECT_CALL(fb, Process(testing::_)).WillOnce(testing::Return(-0.9f));

    float result{ controller.Compute(-1.0f, 0.0f) };

    EXPECT_NEAR(result, -1.0f, math::Tolerance<float>());
}

TEST_F(TestFeedforward2Dof, zero_feedback_reduces_to_feedforward)
{
    EXPECT_CALL(ff, Evaluate(0.5f)).WillOnce(testing::Return(0.4f));
    EXPECT_CALL(fb, Process(testing::_)).WillOnce(testing::Return(0.0f));

    float result{ controller.Compute(0.5f, 0.1f) };

    EXPECT_NEAR(result, 0.4f, math::Tolerance<float>());
}

TEST_F(TestFeedforward2Dof, reset_delegates_to_feedback)
{
    EXPECT_CALL(fb, Reset()).Times(1);

    controller.Reset();
}

TEST_F(TestFeedforward2Dof, negative_reference_handled)
{
    EXPECT_CALL(ff, Evaluate(-0.3f)).WillOnce(testing::Return(-0.2f));
    EXPECT_CALL(fb, Process(-0.3f)).WillOnce(testing::Return(-0.1f));

    float result{ controller.Compute(-0.3f, 0.0f) };

    EXPECT_NEAR(result, -0.3f, math::Tolerance<float>());
}
