#pragma once

#include "numerical/controllers/TrajectoryGenerator.hpp"
#include <cmath>

namespace controllers
{
    template<typename QNumberType>
    class TrajectoryGeneratorTrapezoidal
        : public TrajectoryGenerator<QNumberType>
    {
    public:
        struct Constraints
        {
            QNumberType maxVelocity;
            QNumberType maxAcceleration;
            QNumberType maxDeceleration;
        };

        TrajectoryGeneratorTrapezoidal(const Constraints& constraints);

        void SetTarget(QNumberType targetPosition) override;
        TrajectoryGenerator<QNumberType>::MotionProfile Update(QNumberType dt) override;
        bool IsComplete() const override;
        void Reset(QNumberType currentPosition) override;
        void SetInitialConditions(QNumberType position, QNumberType velocity) override;

    private:
        enum class Phase
        {
            acceleration,
            constantVelocity,
            deceleration,
            complete
        };

        Constraints constraints;
        QNumberType currentPosition = QNumberType(0.0f);
        QNumberType currentVelocity = QNumberType(0.0f);
        QNumberType targetPosition = QNumberType(0.0f);
        QNumberType time = QNumberType(0.0f);

        QNumberType accelerationTime = QNumberType(0.0f);
        QNumberType constantVelocityTime = QNumberType(0.0f);
        QNumberType decelerationTime = QNumberType(0.0f);
        QNumberType totalTime = QNumberType(0.0f);

        Phase currentPhase = Phase::complete;
        bool trajectoryCalculated = false;

        void CalculateTrajectory();
        QNumberType CalculateTriangularProfile();
        QNumberType CalculateTrapezoidalProfile();
    };

    // Implementation

    template<typename QNumberType>
    TrajectoryGeneratorTrapezoidal<QNumberType>::TrajectoryGeneratorTrapezoidal(const Constraints& constraints)
        : constraints(constraints)
    {
    }

    template<typename QNumberType>
    void TrajectoryGeneratorTrapezoidal<QNumberType>::SetTarget(QNumberType targetPosition)
    {
        this->targetPosition = targetPosition;
        time = QNumberType(0.0f);
        currentPhase = Phase::acceleration;
        trajectoryCalculated = false;
        CalculateTrajectory();
    }

    template<typename QNumberType>
    typename TrajectoryGenerator<QNumberType>::MotionProfile
    TrajectoryGeneratorTrapezoidal<QNumberType>::Update(QNumberType dt)
    {
        if (!trajectoryCalculated || currentPhase == Phase::complete)
            return { currentPosition, QNumberType(0.0f), QNumberType(0.0f) };

        time += dt;

        auto acceleration = QNumberType(0.0f);

        if (time <= accelerationTime)
        {
            currentPhase = Phase::acceleration;
            acceleration = (targetPosition > currentPosition) ? constraints.maxAcceleration : -constraints.maxAcceleration;
            currentVelocity += acceleration * dt;
            currentPosition += currentVelocity * dt;
        }
        else if (time <= accelerationTime + constantVelocityTime)
        {
            currentPhase = Phase::constantVelocity;
            acceleration = QNumberType(0.0f);
            currentPosition += currentVelocity * dt;
        }
        else if (time <= totalTime)
        {
            currentPhase = Phase::deceleration;
            acceleration = (targetPosition > currentPosition) ? constraints.maxDeceleration : -constraints.maxDeceleration;
            currentVelocity += acceleration * dt;
            currentPosition += currentVelocity * dt;
        }
        else
        {
            currentPhase = Phase::complete;
            currentPosition = targetPosition;
            currentVelocity = QNumberType(0.0f);
            acceleration = QNumberType(0.0f);
        }

        return { currentPosition, currentVelocity, acceleration };
    }

    template<typename QNumberType>
    bool TrajectoryGeneratorTrapezoidal<QNumberType>::IsComplete() const
    {
        return currentPhase == Phase::complete;
    }

    template<typename QNumberType>
    void TrajectoryGeneratorTrapezoidal<QNumberType>::Reset(QNumberType currentPosition)
    {
        this->currentPosition = currentPosition;
        currentVelocity = QNumberType(0.0f);
        targetPosition = currentPosition;
        time = QNumberType(0.0f);
        currentPhase = Phase::complete;
        trajectoryCalculated = false;
    }

    template<typename QNumberType>
    void TrajectoryGeneratorTrapezoidal<QNumberType>::SetInitialConditions(QNumberType position, QNumberType velocity)
    {
        this->currentPosition = position;
        this->currentVelocity = velocity;
    }

    template<typename QNumberType>
    void TrajectoryGeneratorTrapezoidal<QNumberType>::CalculateTrajectory()
    {
        auto distance = targetPosition - currentPosition;

        if (std::abs(distance) < QNumberType(1e-6f))
        {
            accelerationTime = QNumberType(0.0f);
            constantVelocityTime = QNumberType(0.0f);
            decelerationTime = QNumberType(0.0f);
            totalTime = QNumberType(0.0f);
            currentPhase = Phase::complete;
            trajectoryCalculated = true;
            return;
        }

        auto timeToMaxVelocity = constraints.maxVelocity / constraints.maxAcceleration;
        auto distanceToMaxVelocity = QNumberType(0.5f) * constraints.maxAcceleration * timeToMaxVelocity * timeToMaxVelocity;

        auto timeToStop = constraints.maxVelocity / constraints.maxDeceleration;
        auto distanceToStop = QNumberType(0.5f) * constraints.maxDeceleration * timeToStop * timeToStop;

        auto minDistanceForTrapezoid = distanceToMaxVelocity + distanceToStop;

        if (std::abs(distance) < minDistanceForTrapezoid)
            CalculateTriangularProfile();
        else
            CalculateTrapezoidalProfile();

        trajectoryCalculated = true;
    }

    template<typename QNumberType>
    QNumberType TrajectoryGeneratorTrapezoidal<QNumberType>::CalculateTriangularProfile()
    {
        auto distance = targetPosition - currentPosition;
        distance = std::abs(distance);

        auto totalAcceleration = constraints.maxAcceleration + constraints.maxDeceleration;
        accelerationTime = std::sqrt((QNumberType(2.0f) * distance) / totalAcceleration);
        decelerationTime = accelerationTime;
        constantVelocityTime = QNumberType(0.0f);
        totalTime = accelerationTime + decelerationTime;

        return totalTime;
    }

    template<typename QNumberType>
    QNumberType TrajectoryGeneratorTrapezoidal<QNumberType>::CalculateTrapezoidalProfile()
    {
        auto distance = targetPosition - currentPosition;
        distance = std::abs(distance);

        accelerationTime = constraints.maxVelocity / constraints.maxAcceleration;
        decelerationTime = constraints.maxVelocity / constraints.maxDeceleration;

        auto accelerationDistance = QNumberType(0.5f) * constraints.maxAcceleration * accelerationTime * accelerationTime;
        auto decelerationDistance = QNumberType(0.5f) * constraints.maxDeceleration * decelerationTime * decelerationTime;
        auto constantVelocityDistance = distance - accelerationDistance - decelerationDistance;

        constantVelocityTime = constantVelocityDistance / constraints.maxVelocity;
        totalTime = accelerationTime + constantVelocityTime + decelerationTime;

        return totalTime;
    }
}
