#include "numerical/controllers/TrajectoryGenerator.hpp"
#include <iomanip>
#include <iostream>
#include <vector>

using namespace controllers;

int main()
{
    std::cout << "=== Trajectory Generator Example ===" << std::endl;

    // Create trajectory generator with constraints
    TrajectoryGenerator<float>::Constraints constraints;
    constraints.maxVelocity = 10.0f;    // 10 units/second max velocity
    constraints.maxAcceleration = 5.0f; // 5 units/second² max acceleration
    constraints.maxDeceleration = 5.0f; // 5 units/second² max deceleration

    TrajectoryGenerator<float> generator(constraints);

    // Set initial position
    generator.SetInitialConditions(0.0f, 0.0f); // Start at position 0 with 0 velocity

    // Generate trajectory to target position
    float targetPosition = 25.0f;
    generator.SetTarget(targetPosition);

    std::cout << "Generating trajectory from 0.0 to " << targetPosition << std::endl;
    std::cout << "Max Velocity: " << constraints.maxVelocity << " units/s" << std::endl;
    std::cout << "Max Acceleration: " << constraints.maxAcceleration << " units/s²" << std::endl;
    std::cout << std::endl;

    // Simulate trajectory execution
    std::vector<TrajectoryGenerator<float>::MotionProfile> trajectory;
    float time = 0.0f;
    float dt = 0.1f; // 100ms time steps

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Time\tPosition\tVelocity\tAcceleration" << std::endl;
    std::cout << "----\t--------\t--------\t------------" << std::endl;

    while (!generator.IsComplete() && time < 10.0f) // Safety limit
    {
        auto profile = generator.Update(dt);
        trajectory.push_back(profile);

        std::cout << time << "\t"
                  << profile.position << "\t\t"
                  << profile.velocity << "\t\t"
                  << profile.acceleration << std::endl;

        time += dt;
    }

    std::cout << std::endl;
    std::cout << "Trajectory completed in " << time << " seconds" << std::endl;
    std::cout << "Final position: " << trajectory.back().position << std::endl;
    std::cout << "Final velocity: " << trajectory.back().velocity << std::endl;

    // Test triangular profile (short distance)
    std::cout << std::endl
              << "=== Triangular Profile Example ===" << std::endl;

    generator.Reset(0.0f);
    generator.SetTarget(5.0f); // Short distance

    time = 0.0f;
    std::cout << "Time\tPosition\tVelocity\tAcceleration" << std::endl;
    std::cout << "----\t--------\t--------\t------------" << std::endl;

    while (!generator.IsComplete() && time < 5.0f)
    {
        auto profile = generator.Update(dt);

        std::cout << time << "\t"
                  << profile.position << "\t\t"
                  << profile.velocity << "\t\t"
                  << profile.acceleration << std::endl;

        time += dt;
    }

    std::cout << std::endl;
    std::cout << "Short trajectory completed in " << time << " seconds" << std::endl;

    return 0;
}
