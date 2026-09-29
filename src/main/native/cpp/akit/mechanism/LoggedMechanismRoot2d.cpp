#include "pch.h"

#include "akit/mechanism/LoggedMechanismRoot2d.h"

#include <frc/geometry/Rotation3d.h>
#include <units/length.h>

namespace akit::mechanism {
    LoggedMechanismRoot2d::LoggedMechanismRoot2d(std::string_view name, double x, double y, const private_init&)
        : LoggedMechanismObject2d{name}
        , x_{x}
        , y_{y} {}

    void LoggedMechanismRoot2d::SetPosition(double x, double y) {
        std::scoped_lock lock(mutex_);
        x_ = x;
        y_ = y;
        Flush();
    }

    std::vector<frc::Pose3d> LoggedMechanismRoot2d::Generate3dMechanism() const {
        double x;
        double y;
        {
            std::scoped_lock lock(mutex_);
            x = x_;
            y = y_;
        }
        return LoggedMechanismObject2d::Generate3dMechanism(frc::Pose3d{units::meter_t{x}, units::meter_t{0}, units::meter_t{y}, frc::Rotation3d{}});
    }

    void LoggedMechanismRoot2d::UpdateEntries(std::shared_ptr<nt::NetworkTable> table) {
        xPub_ = table->GetDoubleTopic("x").Publish();
        yPub_ = table->GetDoubleTopic("y").Publish();
        Flush();
    }

    void LoggedMechanismRoot2d::LogEntries(const LogTable& table) const {
        table.Put("x", x_);
        table.Put("y", y_);
    }

    inline void LoggedMechanismRoot2d::Flush() {
        if (xPub_) xPub_.Set(x_);
        if (yPub_) yPub_.Set(y_);
    }
} // namespace akit::mechanism
