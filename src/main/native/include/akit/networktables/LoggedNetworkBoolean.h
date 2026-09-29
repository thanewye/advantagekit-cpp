#pragma once

#include <string>
#include <string_view>

#include <networktables/BooleanTopic.h>

#include "akit/networktables/LoggedNetworkInput.h"

namespace akit::networktables {
    class LoggedNetworkBoolean : public LoggedNetworkInput {
    public:
        explicit LoggedNetworkBoolean(std::string_view key);
        LoggedNetworkBoolean(std::string_view key, bool defaultValue);

        void SetDefault(bool defaultValue);
        void Set(bool value);

        [[nodiscard]] bool Get() const { return value_; }
        [[nodiscard]] bool GetAsBoolean() const { return value_; }

        void Periodic() override;

    private:
        std::string key_;
        nt::BooleanEntry entry_;
        bool defaultValue_ = false;
        bool value_ = false;
    };
} // namespace akit::networktables
