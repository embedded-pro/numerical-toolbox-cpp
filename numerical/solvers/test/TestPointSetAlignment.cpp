#include "numerical/math/Geometry3D.hpp"
#include "numerical/solvers/PointSetAlignment.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Point2 = math::Vector<float, 2>;
    using Point3 = math::Vector<float, 3>;
    using Matrix3 = math::SquareMatrix<float, 3>;
    using Planar = solvers::PointSetAlignment<float, 2>;
    using Spatial = solvers::PointSetAlignment<float, 3>;

    constexpr float pi{ std::numbers::pi_v<float> };

    class TestPointSetAlignment
        : public ::testing::Test
    {
    protected:
        static Point3 Normalized(float x, float y, float z)
        {
            const float norm{ std::sqrt(x * x + y * y + z * z) };
            return Point3{ x / norm, y / norm, z / norm };
        }

        static std::array<Point2, 4> MovePlanar(const std::array<Point2, 4>& points, float angle, const Point2& shift)
        {
            std::array<Point2, 4> moved{};
            for (std::size_t i = 0; i < points.size(); ++i)
                moved[i] = Point2{ std::cos(angle) * points[i][0] - std::sin(angle) * points[i][1] + shift[0], std::sin(angle) * points[i][0] + std::cos(angle) * points[i][1] + shift[1] };
            return moved;
        }

        std::array<Point3, 5> MoveSpatial(float scale) const
        {
            std::array<Point3, 5> moved{};
            for (std::size_t i = 0; i < spatial.size(); ++i)
                moved[i] = rotation * spatial[i] * scale + offset;
            return moved;
        }

        static void ExpectNear(const Point3& actual, const Point3& expected, float tolerance)
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], tolerance);
        }

        static void ExpectNear(const Matrix3& actual, const Matrix3& expected)
        {
            for (std::size_t r = 0; r < 3; ++r)
                for (std::size_t c = 0; c < 3; ++c)
                    EXPECT_NEAR(actual.at(r, c), expected.at(r, c), 1e-5f);
        }

        std::array<Point2, 4> planar{ Point2{ 0.0f, 0.0f }, Point2{ 1.0f, 0.0f }, Point2{ 0.0f, 2.0f }, Point2{ -1.0f, 1.0f } };
        std::array<Point3, 5> spatial{ Point3{ 0.0f, 0.0f, 0.0f }, Point3{ 1.0f, 0.0f, 0.0f }, Point3{ 0.0f, 2.0f, 0.0f }, Point3{ 0.0f, 0.0f, 3.0f }, Point3{ 1.0f, 1.0f, 1.0f } };
        Matrix3 rotation{ math::RotationAboutAxis(Normalized(1.0f, 2.0f, 3.0f), 0.7f) };
        Point3 offset{ 0.2f, -0.4f, 1.0f };
    };
}

TEST_F(TestPointSetAlignment, planar_rigid_motion_is_recovered_exactly)
{
    const auto target{ MovePlanar(planar, pi / 6.0f, Point2{ 0.5f, -1.0f }) };

    const auto alignment{ Planar::Align(planar, target) };

    ASSERT_TRUE(alignment.has_value());
    EXPECT_NEAR(std::atan2(alignment->rotation.at(1, 0), alignment->rotation.at(0, 0)), pi / 6.0f, 1e-5f);
    EXPECT_NEAR(alignment->translation[0], 0.5f, 1e-5f);
    EXPECT_NEAR(alignment->translation[1], -1.0f, 1e-5f);
    EXPECT_NEAR(alignment->rms, 0.0f, 1e-5f);
}

TEST_F(TestPointSetAlignment, spatial_rigid_motion_is_recovered_exactly)
{
    const auto target{ MoveSpatial(1.0f) };

    const auto alignment{ Spatial::Align(spatial, target) };

    ASSERT_TRUE(alignment.has_value());
    ExpectNear(alignment->rotation, rotation);
    ExpectNear(alignment->translation, offset, 1e-5f);
    EXPECT_NEAR(alignment->rms, 0.0f, 1e-5f);
}

TEST_F(TestPointSetAlignment, umeyama_recovers_the_scale)
{
    const auto target{ MoveSpatial(2.5f) };

    const auto alignment{ Spatial::Align(spatial, target, true) };

    ASSERT_TRUE(alignment.has_value());
    EXPECT_NEAR(alignment->scale, 2.5f, 1e-5f);
    ExpectNear(alignment->translation, offset, 1e-4f);
    EXPECT_NEAR(alignment->rms, 0.0f, 1e-4f);
}

TEST_F(TestPointSetAlignment, mirrored_points_still_give_a_proper_rotation)
{
    std::array<Point3, 5> mirrored{ spatial };
    for (auto& point : mirrored)
        point[2] = -point[2];

    const auto alignment{ Spatial::Align(spatial, mirrored) };

    ASSERT_TRUE(alignment.has_value());
    const auto& r{ alignment->rotation };
    const float determinant{ r.at(0, 0) * (r.at(1, 1) * r.at(2, 2) - r.at(1, 2) * r.at(2, 1)) - r.at(0, 1) * (r.at(1, 0) * r.at(2, 2) - r.at(1, 2) * r.at(2, 0)) +
                             r.at(0, 2) * (r.at(1, 0) * r.at(2, 1) - r.at(1, 1) * r.at(2, 0)) };
    EXPECT_NEAR(determinant, 1.0f, 1e-5f);
    EXPECT_NEAR(alignment->rms, 0.925196f, 1e-5f);
}

TEST_F(TestPointSetAlignment, zero_weight_removes_an_outlier)
{
    auto target{ MoveSpatial(1.0f) };
    target[4] = target[4] + Point3{ 5.0f, 5.0f, 5.0f };
    const std::array<float, 5> weights{ 1.0f, 1.0f, 1.0f, 1.0f, 0.0f };

    const auto weighted{ Spatial::Align(spatial, target, false, weights) };
    const auto unweighted{ Spatial::Align(spatial, target) };

    ASSERT_TRUE(weighted.has_value());
    ExpectNear(weighted->rotation, rotation);
    EXPECT_NEAR(weighted->rms, 0.0f, 1e-5f);
    ASSERT_TRUE(unweighted.has_value());
    EXPECT_NEAR(unweighted->rms, 3.442415f, 1e-4f);
}

TEST_F(TestPointSetAlignment, noisy_planar_fit_matches_the_reference)
{
    auto target{ MovePlanar(planar, pi / 6.0f, Point2{ 0.5f, -1.0f }) };
    const std::array<Point2, 4> noise{ Point2{ 0.01f, -0.02f }, Point2{ 0.0f, 0.015f }, Point2{ -0.01f, 0.0f }, Point2{ 0.02f, 0.01f } };
    for (std::size_t i = 0; i < target.size(); ++i)
        target[i] = target[i] + noise[i];

    const auto alignment{ Planar::Align(planar, target) };

    ASSERT_TRUE(alignment.has_value());
    EXPECT_NEAR(std::atan2(alignment->rotation.at(1, 0), alignment->rotation.at(0, 0)), 0.528695f, 1e-5f);
    EXPECT_NEAR(alignment->translation[0], 0.508305f, 1e-5f);
    EXPECT_NEAR(alignment->translation[1], -0.996831f, 1e-5f);
    EXPECT_NEAR(alignment->rms, 0.016549f, 1e-5f);
}

TEST_F(TestPointSetAlignment, degenerate_inputs_are_rejected)
{
    const std::array<Point3, 3> collinear{ Point3{ 0.0f, 0.0f, 0.0f }, Point3{ 1.0f, 1.0f, 1.0f }, Point3{ 2.0f, 2.0f, 2.0f } };
    const std::array<Point2, 1> single{ Point2{ 1.0f, 2.0f } };
    const std::array<Point2, 2> coincident{ Point2{ 1.0f, 2.0f }, Point2{ 1.0f, 2.0f } };
    const std::array<Point2, 3> three{ Point2{ 0.0f, 0.0f }, Point2{ 1.0f, 0.0f }, Point2{ 0.0f, 1.0f } };

    EXPECT_FALSE(Spatial::Align(collinear, collinear).has_value());
    EXPECT_FALSE(Planar::Align(single, single).has_value());
    EXPECT_FALSE(Planar::Align(planar, three).has_value());
    EXPECT_FALSE(Planar::Align(coincident, coincident).has_value());
}
