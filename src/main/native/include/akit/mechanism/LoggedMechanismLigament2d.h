#pragma once

#include <memory>
#include <string_view>

#include <frc/util/Color8Bit.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>
#include <units/angle.h>

#include "akit/mechanism/LoggedMechanismObject2d.h"

namespace akit::mechanism {
    /** Ligament node on a LoggedMechanism2d, mirroring frc::MechanismLigament2d. */
    class LoggedMechanismLigament2d : public LoggedMechanismObject2d {
    public:
        LoggedMechanismLigament2d(std::string_view name, double length, units::degree_t angle, double lineWidth = 6,
                                  const frc::Color8Bit& color = {235, 137, 52});

        void SetColor(const frc::Color8Bit& color);

        frc::Color8Bit GetColor();

        void SetLength(double length);

        double GetLength();

        void SetAngle(units::degree_t angle);

        double GetAngle() override;

        void SetLineWeight(double lineWidth);

        double GetLineWeight();

        double GetObject2dRange() override { return GetLength(); }

    protected:
        void UpdateEntries(std::shared_ptr<nt::NetworkTable> table) override;
        void LogEntries(const LogTable& table) const override;

    private:
        nt::StringPublisher typePub_;
        double length_;
        nt::DoubleEntry lengthEntry_;
        double angle_;
        nt::DoubleEntry angleEntry_;
        double weight_;
        nt::DoubleEntry weightEntry_;
        char color_[10]{};
        nt::StringEntry colorEntry_;
    };
} // namespace akit::mechanism
