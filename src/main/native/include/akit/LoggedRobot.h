#pragma once

#include <frc/IterativeRobotBase.h>

namespace akit {
    class LoggedRobot : public frc::IterativeRobotBase {
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
        uint64_t periodUs_;
        uint64_t nextCycleUs_{0};
        bool useTiming_{true};
        HAL_NotifierHandle notifier_;
        inline static bool baseConstructed_ = false;
    };
} // namespace akit
