#include "pch.h"

#include "akit/mechanism/LoggedMechanismLigament2d.h"

#include <cstdio>
#include <cstring>

#include <wpi/StringExtras.h>
#include <wpi/json.h>

namespace akit::mechanism {
    static constexpr std::string_view kSmartDashboardType = "line";

    LoggedMechanismLigament2d::LoggedMechanismLigament2d(std::string_view name, double length, units::degree_t angle, double lineWeight,
                                                         const frc::Color8Bit& color)
        : LoggedMechanismObject2d{name}
        , length_{length}
        , angle_{angle.value()}
        , weight_{lineWeight} {
        SetColor(color);
    }

    void LoggedMechanismLigament2d::UpdateEntries(std::shared_ptr<nt::NetworkTable> table) {
        typePub_ = table->GetStringTopic(".type").PublishEx(nt::StringTopic::kTypeString, {{"SmartDashboard", kSmartDashboardType}});
        typePub_.Set(kSmartDashboardType);

        colorEntry_ = table->GetStringTopic("color").GetEntry("");
        colorEntry_.Set(color_);
        angleEntry_ = table->GetDoubleTopic("angle").GetEntry(0.0);
        angleEntry_.Set(angle_);
        weightEntry_ = table->GetDoubleTopic("weight").GetEntry(0.0);
        weightEntry_.Set(weight_);
        lengthEntry_ = table->GetDoubleTopic("length").GetEntry(0.0);
        lengthEntry_.Set(length_);
    }

    void LoggedMechanismLigament2d::LogEntries(const LogTable& table) const {
        table.Put(".type", kSmartDashboardType);
        table.Put("angle", angle_);
        table.Put("length", length_);
        table.Put("color", std::string_view{color_});
        table.Put("weight", weight_);
    }

    void LoggedMechanismLigament2d::SetColor(const frc::Color8Bit& color) {
        std::scoped_lock lock(mutex_);

        wpi::format_to_n_c_str(color_, sizeof(color_), "#{:02X}{:02X}{:02X}", color.red, color.green, color.blue);

        if (colorEntry_) colorEntry_.Set(color_);
    }

    void LoggedMechanismLigament2d::SetAngle(units::degree_t angle) {
        std::scoped_lock lock(mutex_);
        angle_ = angle.value();
        if (angleEntry_) angleEntry_.Set(angle_);
    }

    void LoggedMechanismLigament2d::SetLineWeight(double lineWidth) {
        std::scoped_lock lock(mutex_);
        weight_ = lineWidth;
        if (weightEntry_) weightEntry_.Set(weight_);
    }

    frc::Color8Bit LoggedMechanismLigament2d::GetColor() {
        std::scoped_lock lock(mutex_);
        if (colorEntry_) {
            auto color = colorEntry_.Get();
            std::strncpy(color_, color.c_str(), sizeof(color_));
            color_[sizeof(color_) - 1] = '\0';
        }
        unsigned int r = 0, g = 0, b = 0;
        std::sscanf(color_, "#%02X%02X%02X", &r, &g, &b);
        return {static_cast<int>(r), static_cast<int>(g), static_cast<int>(b)};
    }

    double LoggedMechanismLigament2d::GetAngle() {
        std::scoped_lock lock(mutex_);
        if (angleEntry_) angle_ = angleEntry_.Get();
        return angle_;
    }

    double LoggedMechanismLigament2d::GetLength() {
        std::scoped_lock lock(mutex_);
        if (lengthEntry_) length_ = lengthEntry_.Get();
        return length_;
    }

    double LoggedMechanismLigament2d::GetLineWeight() {
        std::scoped_lock lock(mutex_);
        if (weightEntry_) weight_ = weightEntry_.Get();
        return weight_;
    }

    void LoggedMechanismLigament2d::SetLength(double length) {
        std::scoped_lock lock(mutex_);
        length_ = length;
        if (lengthEntry_) lengthEntry_.Set(length);
    }
} // namespace akit::mechanism
