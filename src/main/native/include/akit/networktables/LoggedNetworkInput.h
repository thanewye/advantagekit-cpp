#pragma once

#include <string>
#include <string_view>

namespace akit::networktables {
    class LoggedNetworkInput {
    public:
        virtual ~LoggedNetworkInput();

        virtual void Periodic() = 0;

    protected:
        static constexpr std::string_view kPrefix = "NetworkInputs";

        static std::string RemoveSlash(std::string_view key);

        LoggedNetworkInput() = default;
    };
} // namespace akit::networktables
