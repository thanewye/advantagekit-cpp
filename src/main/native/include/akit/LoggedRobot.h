#pragma once

#include <wpi/framework/IterativeRobotBase.hpp>
#include <wpi/hal/Types.h>

namespace akit {
    class LoggedRobot : public wpi::IterativeRobotBase {
    public:
        static constexpr double kDefaultPeriodSeconds = 0.02;
        static bool IsBaseConstructed() { return baseConstructed_; }

        void StartCompetition() override;
        void EndCompetition() override;
        void SetUseTiming(bool useTiming) { useTiming_ = useTiming; }

    protected:
        explicit LoggedRobot(double period = kDefaultPeriodSeconds);
        ~LoggedRobot() override;

    private:
        int64_t periodNs_;
        int64_t nextCycleNs_{0};
        bool useTiming_{true};
        HAL_NotifierHandle notifier_;
        inline static bool baseConstructed_ = false;
    };
} // namespace akit
