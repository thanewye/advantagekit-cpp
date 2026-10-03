#pragma once

#include <string_view>

#include <wpi/units/angle.hpp>
#include <wpi/util/Color8Bit.hpp>

#include "akit/mechanism/LoggedMechanismObject2d.h"

namespace akit::mechanism {
    /** Ligament node on a LoggedMechanism2d, mirroring wpi::MechanismLigament2d. */
    class LoggedMechanismLigament2d : public LoggedMechanismObject2d {
    public:
        LoggedMechanismLigament2d(std::string_view name, double length, wpi::units::degree_t angle, double lineWidth = 6,
                                  const wpi::util::Color8Bit& color = {235, 137, 52});

        void SetColor(const wpi::util::Color8Bit& color);

        wpi::util::Color8Bit GetColor();

        void SetLength(double length);

        double GetLength();

        void SetAngle(wpi::units::degree_t angle);

        double GetAngle() override;

        void SetLineWeight(double lineWidth);

        double GetLineWeight();

        double GetObject2dRange() override { return GetLength(); }

        void LogTo(wpi::telemetry::TelemetryTable& table) const override;

        std::string_view GetTelemetryType() const override;

    protected:
        void LogEntries(const LogTable& table) const override;

    private:
        double length_;
        double angle_;
        double weight_;
        char color_[10]{};
    };
} // namespace akit::mechanism
