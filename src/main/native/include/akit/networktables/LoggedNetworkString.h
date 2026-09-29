#pragma once

#include <string>
#include <string_view>

#include <networktables/StringTopic.h>

#include "akit/networktables/LoggedNetworkInput.h"

namespace akit::networktables {
    class LoggedNetworkString : public LoggedNetworkInput {
    public:
        explicit LoggedNetworkString(std::string_view key);
        LoggedNetworkString(std::string_view key, std::string_view defaultValue);

        void SetDefault(std::string_view defaultValue);
        void Set(std::string_view value);

        [[nodiscard]] const std::string& Get() const { return value_; }

        void Periodic() override;

    private:
        std::string key_;
        nt::StringEntry entry_;
        std::string defaultValue_;
        std::string value_;
    };
} // namespace akit::networktables
