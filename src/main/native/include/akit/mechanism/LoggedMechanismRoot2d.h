#pragma once

#include <string_view>
#include <vector>

#include <wpi/math/geometry/Pose3d.hpp>

#include "akit/mechanism/LoggedMechanismObject2d.h"

namespace akit::mechanism {
    /** Root LoggedMechanism2d node, mirroring wpi::MechanismRoot2d. Obtain instances from LoggedMechanism2d::GetRoot. */
    class LoggedMechanismRoot2d : private LoggedMechanismObject2d {
        friend class LoggedMechanism2d;
        struct private_init {};

    public:
        LoggedMechanismRoot2d(std::string_view name, double x, double y, const private_init&);

        void SetPosition(double x, double y);

        /** Converts this root's children into poses, depth first, with the root at (x, 0, y). */
        std::vector<wpi::math::Pose3d> Generate3dMechanism() const;

        void LogTo(wpi::telemetry::TelemetryTable& table) const override;

        using LoggedMechanismObject2d::GetName;

        using LoggedMechanismObject2d::Append;

    private:
        void LogEntries(const LogTable& table) const override;
        double GetObject2dRange() override { return 0.0; }
        double GetAngle() override { return 0.0; }
        double x_;
        double y_;
    };
} // namespace akit::mechanism
