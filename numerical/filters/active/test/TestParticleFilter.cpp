#include "numerical/filters/active/ParticleFilter.hpp"
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <limits>

namespace
{
    using Vector = math::Vector<float, 1>;
    using Grid = filters::ParticleFilter<float, 1, 41>;
    using Four = filters::ParticleFilter<float, 1, 4>;

    Vector GridPoint(std::size_t i)
    {
        return Vector{ -2.0f + 4.0f * static_cast<float>(i) / 40.0f };
    }

    Vector Index(std::size_t i)
    {
        return Vector{ static_cast<float>(i) };
    }

    float Gaussian(const Vector& x, float z, float sigma)
    {
        const float r{ (z - x[0]) / sigma };
        return -0.5f * r * r;
    }

    float MeasurementAt05(const Vector& x)
    {
        return Gaussian(x, 0.5f, 0.5f);
    }

    float MeasurementAt05Shifted(const Vector& x)
    {
        return Gaussian(x, 0.5f, 0.5f) - 1000.0f;
    }

    float MeasurementAt07(const Vector& x)
    {
        return Gaussian(x, 0.7f, 0.5f);
    }

    float Ramp(const Vector& x)
    {
        return std::log(0.1f * (x[0] + 1.0f));
    }

    float Impossible(const Vector&)
    {
        return -std::numeric_limits<float>::infinity();
    }

    void Shift(Vector& x, std::size_t)
    {
        x[0] += 1.0f;
    }

    class TestParticleFilter
        : public ::testing::Test
    {
    protected:
        TestParticleFilter()
        {
            grid.Initialize(GridPoint);
            four.Initialize(Index);
            four.Update(Ramp);
        }

        static void ExpectWeights(const Four& filter, float w0, float w1, float w2, float w3)
        {
            const auto weights{ filter.Weights() };
            EXPECT_NEAR(weights[0], w0, 1e-6f);
            EXPECT_NEAR(weights[1], w1, 1e-6f);
            EXPECT_NEAR(weights[2], w2, 1e-6f);
            EXPECT_NEAR(weights[3], w3, 1e-6f);
        }

        static void ExpectParticles(const Four& filter, float p0, float p1, float p2, float p3)
        {
            const auto particles{ filter.Particles() };
            EXPECT_EQ(particles[0][0], p0);
            EXPECT_EQ(particles[1][0], p1);
            EXPECT_EQ(particles[2][0], p2);
            EXPECT_EQ(particles[3][0], p3);
        }

        Grid grid;
        Four four;
    };
}

TEST_F(TestParticleFilter, initialize_sets_uniform_weights)
{
    for (const float weight : grid.Weights())
        EXPECT_NEAR(weight, 1.0f / 41.0f, 1e-7f);
    EXPECT_NEAR(grid.EffectiveSampleSize(), 41.0f, 1e-4f);
}

TEST_F(TestParticleFilter, update_normalises_in_the_log_domain)
{
    Grid shifted;
    shifted.Initialize(GridPoint);

    EXPECT_TRUE(grid.Update(MeasurementAt05));
    EXPECT_TRUE(shifted.Update(MeasurementAt05Shifted));

    EXPECT_NEAR(grid.Mean()[0], 0.498389f, 1e-5f);
    EXPECT_NEAR(grid.EffectiveSampleSize(), 17.690938f, 1e-4f);
    EXPECT_NEAR(grid.MaximumWeight()[0], 0.5f, 1e-6f);
    float sum{ 0.0f };
    for (std::size_t i = 0; i < 41; ++i)
    {
        sum += grid.Weights()[i];
        EXPECT_NEAR(shifted.Weights()[i], grid.Weights()[i], 1e-5f);
    }
    EXPECT_NEAR(sum, 1.0f, 1e-5f);
}

TEST_F(TestParticleFilter, sequential_updates_multiply_likelihoods)
{
    grid.Update(MeasurementAt05);
    grid.Update(MeasurementAt07);

    EXPECT_NEAR(grid.Mean()[0], 0.599970f, 1e-5f);
    EXPECT_NEAR(grid.EffectiveSampleSize(), 12.532656f, 1e-4f);
}

TEST_F(TestParticleFilter, systematic_resampling_is_deterministic)
{
    Four fresh;
    fresh.Initialize(Index);
    fresh.Update(Ramp);

    EXPECT_NEAR(four.EffectiveSampleSize(), 3.333333f, 1e-5f);
    four.Resample(0.5f);
    fresh.Resample(0.0f);

    ExpectParticles(four, 1.0f, 2.0f, 3.0f, 3.0f);
    ExpectWeights(four, 0.25f, 0.25f, 0.25f, 0.25f);
    ExpectParticles(fresh, 0.0f, 1.0f, 2.0f, 3.0f);
}

TEST_F(TestParticleFilter, resample_if_needed_respects_the_threshold)
{
    EXPECT_FALSE(four.ResampleIfNeeded(0.5f, 0.5f));
    ExpectParticles(four, 0.0f, 1.0f, 2.0f, 3.0f);
    ExpectWeights(four, 0.1f, 0.2f, 0.3f, 0.4f);

    EXPECT_TRUE(four.ResampleIfNeeded(0.9f, 0.5f));
    ExpectParticles(four, 1.0f, 2.0f, 3.0f, 3.0f);
}

TEST_F(TestParticleFilter, predict_moves_every_particle_and_keeps_weights)
{
    four.Predict(Shift);

    EXPECT_NEAR(four.Mean()[0], 3.0f, 1e-5f);
    EXPECT_EQ(four.Particles()[3][0], 4.0f);
    ExpectWeights(four, 0.1f, 0.2f, 0.3f, 0.4f);
}

TEST_F(TestParticleFilter, impossible_measurement_leaves_weights_unchanged)
{
    EXPECT_FALSE(four.Update(Impossible));

    ExpectWeights(four, 0.1f, 0.2f, 0.3f, 0.4f);
}
