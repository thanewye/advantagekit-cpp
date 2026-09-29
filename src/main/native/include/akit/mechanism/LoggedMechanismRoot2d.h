#pragma once

#include <memory>
#include <string_view>
#include <vector>

#include <frc/geometry/Pose3d.h>
#include <networktables/DoubleTopic.h>

#include "akit/mechanism/LoggedMechanismObject2d.h"

namespace akit::mechanism {
    /** Root LoggedMechanism2d node, mirroring frc::MechanismRoot2d. Obtain instances from LoggedMechanism2d::GetRoot. */
    class LoggedMechanismRoot2d : private LoggedMechanismObject2d {
        friend class LoggedMechanism2d;
        struct private_init {};

    public:
        LoggedMechanismRoot2d(std::string_view name, double x, double y, const private_init&);

        void SetPosition(double x, double y);

        /** Converts this root's children into poses, depth first, with the root at (x, 0, y). */
        std::vector<frc::Pose3d> Generate3dMechanism() const;

        using LoggedMechanismObject2d::GetName;

        using LoggedMechanismObject2d::Append;

    private:
        void UpdateEntries(std::shared_ptr<nt::NetworkTable> table) override;
        void LogEntries(const LogTable& table) const override;
        double GetObject2dRange() override { return 0.0; }
        double GetAngle() override { return 0.0; }
        inline void Flush();
        double x_;
        double y_;
        nt::DoublePublisher xPub_;
        nt::DoublePublisher yPub_;
    };
} // namespace akit::mechanism
