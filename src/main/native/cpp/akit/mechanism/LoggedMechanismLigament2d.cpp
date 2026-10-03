#include "pch.h"

#include "akit/mechanism/LoggedMechanismLigament2d.h"

#include <cstdio>

#include <wpi/telemetry/TelemetryTable.hpp>
#include <wpi/util/StringExtras.hpp>

namespace akit::mechanism {
    static constexpr std::string_view kTelemetryType = "line";

    LoggedMechanismLigament2d::LoggedMechanismLigament2d(std::string_view name, double length, wpi::units::degree_t angle, double lineWeight,
                                                         const wpi::util::Color8Bit& color)
        : LoggedMechanismObject2d{name}
        , length_{length}
        , angle_{angle.value()}
        , weight_{lineWeight} {
        SetColor(color);
    }

    void LoggedMechanismLigament2d::LogTo(wpi::telemetry::TelemetryTable& table) const {
        {
            std::scoped_lock lock(mutex_);
            table.Log("angle", angle_);
            table.Log("length", length_);
            table.Log("color", std::string_view{color_});
            table.Log("weight", weight_);
        }
        LoggedMechanismObject2d::LogTo(table);
    }

    std::string_view LoggedMechanismLigament2d::GetTelemetryType() const {
        return kTelemetryType;
    }

    void LoggedMechanismLigament2d::LogEntries(const LogTable& table) const {
        table.Put(".type", kTelemetryType);
        table.Put("angle", angle_);
        table.Put("length", length_);
        table.Put("color", std::string_view{color_});
        table.Put("weight", weight_);
    }

    void LoggedMechanismLigament2d::SetColor(const wpi::util::Color8Bit& color) {
        std::scoped_lock lock(mutex_);
        wpi::util::format_to_n_c_str(color_, sizeof(color_), "#{:02X}{:02X}{:02X}", color.red, color.green, color.blue);
    }

    void LoggedMechanismLigament2d::SetAngle(wpi::units::degree_t angle) {
        std::scoped_lock lock(mutex_);
        angle_ = angle.value();
    }

    void LoggedMechanismLigament2d::SetLineWeight(double lineWidth) {
        std::scoped_lock lock(mutex_);
        weight_ = lineWidth;
    }

    wpi::util::Color8Bit LoggedMechanismLigament2d::GetColor() {
        std::scoped_lock lock(mutex_);
        unsigned int r = 0, g = 0, b = 0;
        std::sscanf(color_, "#%02X%02X%02X", &r, &g, &b);
        return {static_cast<int>(r), static_cast<int>(g), static_cast<int>(b)};
    }

    double LoggedMechanismLigament2d::GetAngle() {
        std::scoped_lock lock(mutex_);
        return angle_;
    }

    double LoggedMechanismLigament2d::GetLength() {
        std::scoped_lock lock(mutex_);
        return length_;
    }

    double LoggedMechanismLigament2d::GetLineWeight() {
        std::scoped_lock lock(mutex_);
        return weight_;
    }

    void LoggedMechanismLigament2d::SetLength(double length) {
        std::scoped_lock lock(mutex_);
        length_ = length;
    }
} // namespace akit::mechanism
