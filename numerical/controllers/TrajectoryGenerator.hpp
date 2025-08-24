#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include "numerical/math/QNumber.hpp"
#include <cmath>

namespace controllers
{
    template<typename QNumberType>
    class TrajectoryGenerator
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "TrajectoryGenerator can only be instantiated with math::QNumber or floating point types.");

    public:
        struct MotionProfile
        {
            QNumberType position;
            QNumberType velocity;
            QNumberType acceleration;
        };

        struct Constraints
        {
            QNumberType maxVelocity;
            QNumberType maxAcceleration;
            QNumberType maxDeceleration;
        };

        TrajectoryGenerator(const Constraints& constraints);

        // Set new target and generate trajectory
        void SetTarget(QNumberType targetPosition);

        // Get current motion profile at given time step
        MotionProfile Update(QNumberType dt);

        // Check if trajectory is complete
        bool IsComplete() const;

        // Reset to current position
        void Reset(QNumberType currentPosition);

        // Set current position and velocity
        void SetInitialConditions(QNumberType position, QNumberType velocity = QNumberType(0));

    private:
        enum class Phase
        {
            Acceleration,
            ConstantVelocity,
            Deceleration,
            Complete
        };

        Constraints constraints_;
        QNumberType currentPosition_;
        QNumberType currentVelocity_;
        QNumberType targetPosition_;
        QNumberType time_;

        // Trajectory timing
        QNumberType accelerationTime_;
        QNumberType constantVelocityTime_;
        QNumberType decelerationTime_;
        QNumberType totalTime_;

        Phase currentPhase_;
        bool trajectoryCalculated_;

        void CalculateTrajectory();
        QNumberType CalculateTriangularProfile();
        QNumberType CalculateTrapezoidalProfile();
    };

    // Template specializations for common types
    using TrajectoryGeneratorF = TrajectoryGenerator<float>;
    using TrajectoryGeneratorD = TrajectoryGenerator<double>;

    // Implementation

    template<typename QNumberType>
    TrajectoryGenerator<QNumberType>::TrajectoryGenerator(const Constraints& constraints)
        : constraints_(constraints)
        , currentPosition_(QNumberType(0))
        , currentVelocity_(QNumberType(0))
        , targetPosition_(QNumberType(0))
        , time_(QNumberType(0))
        , accelerationTime_(QNumberType(0))
        , constantVelocityTime_(QNumberType(0))
        , decelerationTime_(QNumberType(0))
        , totalTime_(QNumberType(0))
        , currentPhase_(Phase::Complete)
        , trajectoryCalculated_(false)
    {
    }

    template<typename QNumberType>
    void TrajectoryGenerator<QNumberType>::SetTarget(QNumberType targetPosition)
    {
        targetPosition_ = targetPosition;
        time_ = QNumberType(0);
        currentPhase_ = Phase::Acceleration;
        trajectoryCalculated_ = false;
        CalculateTrajectory();
    }

    template<typename QNumberType>
    typename TrajectoryGenerator<QNumberType>::MotionProfile
    TrajectoryGenerator<QNumberType>::Update(QNumberType dt)
    {
        if (!trajectoryCalculated_ || currentPhase_ == Phase::Complete)
        {
            return { currentPosition_, QNumberType(0), QNumberType(0) };
        }

        time_ += dt;

        QNumberType acceleration = QNumberType(0);

        if (time_ <= accelerationTime_)
        {
            // Acceleration phase
            currentPhase_ = Phase::Acceleration;
            QNumberType direction = (targetPosition_ > currentPosition_) ? QNumberType(1) : QNumberType(-1);
            acceleration = direction * constraints_.maxAcceleration;
            currentVelocity_ += acceleration * dt;
            currentPosition_ += currentVelocity_ * dt;
        }
        else if (time_ <= accelerationTime_ + constantVelocityTime_)
        {
            // Constant velocity phase
            currentPhase_ = Phase::ConstantVelocity;
            acceleration = QNumberType(0);
            currentPosition_ += currentVelocity_ * dt;
        }
        else if (time_ <= totalTime_)
        {
            // Deceleration phase
            currentPhase_ = Phase::Deceleration;
            QNumberType direction = (targetPosition_ > currentPosition_) ? QNumberType(-1) : QNumberType(1);
            acceleration = direction * constraints_.maxDeceleration;
            currentVelocity_ += acceleration * dt;
            currentPosition_ += currentVelocity_ * dt;
        }
        else
        {
            // Trajectory complete
            currentPhase_ = Phase::Complete;
            currentPosition_ = targetPosition_;
            currentVelocity_ = QNumberType(0);
            acceleration = QNumberType(0);
        }

        return { currentPosition_, currentVelocity_, acceleration };
    }

    template<typename QNumberType>
    bool TrajectoryGenerator<QNumberType>::IsComplete() const
    {
        return currentPhase_ == Phase::Complete;
    }

    template<typename QNumberType>
    void TrajectoryGenerator<QNumberType>::Reset(QNumberType currentPosition)
    {
        currentPosition_ = currentPosition;
        currentVelocity_ = QNumberType(0);
        targetPosition_ = currentPosition;
        time_ = QNumberType(0);
        currentPhase_ = Phase::Complete;
        trajectoryCalculated_ = false;
    }

    template<typename QNumberType>
    void TrajectoryGenerator<QNumberType>::SetInitialConditions(QNumberType position, QNumberType velocity)
    {
        currentPosition_ = position;
        currentVelocity_ = velocity;
    }

    template<typename QNumberType>
    void TrajectoryGenerator<QNumberType>::CalculateTrajectory()
    {
        QNumberType distance = targetPosition_ - currentPosition_;

        if (std::abs(distance) < QNumberType(1e-6))
        {
            // Already at target
            accelerationTime_ = QNumberType(0);
            constantVelocityTime_ = QNumberType(0);
            decelerationTime_ = QNumberType(0);
            totalTime_ = QNumberType(0);
            currentPhase_ = Phase::Complete;
            trajectoryCalculated_ = true;
            return;
        }

        // Determine if we need triangular or trapezoidal profile
        QNumberType timeToMaxVelocity = constraints_.maxVelocity / constraints_.maxAcceleration;
        QNumberType distanceToMaxVelocity = QNumberType(0.5) * constraints_.maxAcceleration * timeToMaxVelocity * timeToMaxVelocity;

        QNumberType timeToStop = constraints_.maxVelocity / constraints_.maxDeceleration;
        QNumberType distanceToStop = QNumberType(0.5) * constraints_.maxDeceleration * timeToStop * timeToStop;

        QNumberType minDistanceForTrapezoid = distanceToMaxVelocity + distanceToStop;

        if (std::abs(distance) < minDistanceForTrapezoid)
        {
            // Triangular profile
            CalculateTriangularProfile();
        }
        else
        {
            // Trapezoidal profile
            CalculateTrapezoidalProfile();
        }

        trajectoryCalculated_ = true;
    }

    template<typename QNumberType>
    QNumberType TrajectoryGenerator<QNumberType>::CalculateTriangularProfile()
    {
        QNumberType distance = targetPosition_ - currentPosition_;
        QNumberType direction = (distance > QNumberType(0)) ? QNumberType(1) : QNumberType(-1);
        distance = std::abs(distance);

        // For triangular profile: t_acc = t_dec and no constant velocity phase
        QNumberType totalAcceleration = constraints_.maxAcceleration + constraints_.maxDeceleration;
        accelerationTime_ = std::sqrt((QNumberType(2) * distance) / totalAcceleration);
        decelerationTime_ = accelerationTime_;
        constantVelocityTime_ = QNumberType(0);
        totalTime_ = accelerationTime_ + decelerationTime_;

        return totalTime_;
    }

    template<typename QNumberType>
    QNumberType TrajectoryGenerator<QNumberType>::CalculateTrapezoidalProfile()
    {
        QNumberType distance = targetPosition_ - currentPosition_;
        QNumberType direction = (distance > QNumberType(0)) ? QNumberType(1) : QNumberType(-1);
        distance = std::abs(distance);

        accelerationTime_ = constraints_.maxVelocity / constraints_.maxAcceleration;
        decelerationTime_ = constraints_.maxVelocity / constraints_.maxDeceleration;

        QNumberType accelerationDistance = QNumberType(0.5) * constraints_.maxAcceleration * accelerationTime_ * accelerationTime_;
        QNumberType decelerationDistance = QNumberType(0.5) * constraints_.maxDeceleration * decelerationTime_ * decelerationTime_;
        QNumberType constantVelocityDistance = distance - accelerationDistance - decelerationDistance;

        constantVelocityTime_ = constantVelocityDistance / constraints_.maxVelocity;
        totalTime_ = accelerationTime_ + constantVelocityTime_ + decelerationTime_;

        return totalTime_;
    }
}

#endif // TRAJECTORY_GENERATOR_HPP
