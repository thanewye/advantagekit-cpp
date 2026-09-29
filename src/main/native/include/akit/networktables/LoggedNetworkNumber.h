#pragma once

#include <string>
#include <string_view>

#include <networktables/DoubleTopic.h>

#include "akit/networktables/LoggedNetworkInput.h"

namespace akit::networktables {
    class LoggedNetworkNumber : public LoggedNetworkInput {
    public:
        explicit LoggedNetworkNumber(std::string_view key);
        LoggedNetworkNumber(std::string_view key, double defaultValue);

        void SetDefault(double defaultValue);
        void Set(double value);

        [[nodiscard]] double Get() const { return value_; }
        [[nodiscard]] double GetAsDouble() const { return value_; }

        void Periodic() override;

    private:
        std::string key_;
        nt::DoubleEntry entry_;
        double defaultValue_ = 0.0;
        double value_ = 0.0;
    };
} // namespace akit::networktables
