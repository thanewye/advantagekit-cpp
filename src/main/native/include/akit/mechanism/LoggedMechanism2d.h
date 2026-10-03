#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/telemetry/TelemetryLoggable.hpp>
#include <wpi/util/Color8Bit.hpp>
#include <wpi/util/mutex.hpp>

#include "akit/log/LogTable.h"
#include "akit/mechanism/LoggedMechanismRoot2d.h"
#include "akit/util/LinkedHashMap.h"

namespace akit::mechanism {
    /** wpi::Mechanism2d that can also be recorded with Logger::RecordOutput. */
    class LoggedMechanism2d : public wpi::telemetry::TelemetryLoggable {
    public:
        LoggedMechanism2d(double width, double height, const wpi::util::Color8Bit& backgroundColor = {0, 0, 32});

        LoggedMechanismRoot2d* GetRoot(std::string_view name, double x, double y);

        void SetBackgroundColor(const wpi::util::Color8Bit& color);

        void LogTo(wpi::telemetry::TelemetryTable& table) const override;

        std::string_view GetTelemetryType() const override;

        /** Records the current mechanism to the log. Called by Logger::RecordOutput, not user code. */
        void LogOutput(const LogTable& table) const;

        /** Converts the mechanism into poses for 3D components, roots in insertion order, each depth first. */
        std::vector<wpi::math::Pose3d> Generate3dMechanism() const;

    private:
        double width_;
        double height_;
        std::string color_;
        mutable wpi::util::mutex mutex_;
        util::LinkedHashMap<std::string, std::unique_ptr<LoggedMechanismRoot2d>> roots_;
    };
} // namespace akit::mechanism
