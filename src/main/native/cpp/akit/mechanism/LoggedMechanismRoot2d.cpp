#include "pch.h"

#include "akit/mechanism/LoggedMechanismRoot2d.h"

#include <array>
#include <span>

#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/telemetry/TelemetryTable.hpp>
#include <wpi/units/length.hpp>

namespace akit::mechanism {
    LoggedMechanismRoot2d::LoggedMechanismRoot2d(std::string_view name, double x, double y, const private_init&)
        : LoggedMechanismObject2d{name}
        , x_{x}
        , y_{y} {}

    void LoggedMechanismRoot2d::SetPosition(double x, double y) {
        std::scoped_lock lock(mutex_);
        x_ = x;
        y_ = y;
    }

    std::vector<wpi::math::Pose3d> LoggedMechanismRoot2d::Generate3dMechanism() const {
        double x;
        double y;
        {
            std::scoped_lock lock(mutex_);
            x = x_;
            y = y_;
        }
        return LoggedMechanismObject2d::Generate3dMechanism(
            wpi::math::Pose3d{wpi::units::meter_t{x}, wpi::units::meter_t{0}, wpi::units::meter_t{y}, wpi::math::Rotation3d{}});
    }

    void LoggedMechanismRoot2d::LogTo(wpi::telemetry::TelemetryTable& table) const {
        {
            std::scoped_lock lock(mutex_);
            table.Log("position", {x_, y_});
        }
        LoggedMechanismObject2d::LogTo(table);
    }

    void LoggedMechanismRoot2d::LogEntries(const LogTable& table) const {
        const std::array<double, 2> position{x_, y_};
        table.Put("position", std::span<const double>(position));
    }
} // namespace akit::mechanism
