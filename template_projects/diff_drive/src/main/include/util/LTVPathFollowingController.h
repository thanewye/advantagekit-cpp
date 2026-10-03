#pragma once

#include <pathplanner/lib/controllers/PathFollowingController.h>
#include <wpi/math/controller/LTVUnicycleController.hpp>

class LTVPathFollowingController : public pathplanner::PathFollowingController {
public:
    explicit LTVPathFollowingController(wpi::units::second_t period)
        : controller_(period) {}

    wpi::math::ChassisVelocities calculateRobotRelativeSpeeds(const wpi::math::Pose2d& currentPose,
                                                              const pathplanner::PathPlannerTrajectoryState& targetState) override {
        positionalError_ = currentPose.Translation().Distance(targetState.pose.Translation());
        return controller_.Calculate(currentPose, targetState.pose, targetState.linearVelocity, targetState.fieldSpeeds.omega);
    }

    void reset(const wpi::math::Pose2d&, const wpi::math::ChassisVelocities&) override { positionalError_ = 0_m; }

    wpi::units::meter_t getPositionalError() override { return positionalError_; }

    bool isHolonomic() override { return false; }

private:
    wpi::math::LTVUnicycleController controller_;
    wpi::units::meter_t positionalError_{0_m};
};
