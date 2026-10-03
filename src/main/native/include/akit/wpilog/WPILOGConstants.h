#pragma once

#include <string_view>

namespace akit::WPILOGConstants {
    inline constexpr std::string_view kExtraHeader = "AdvantageKit";
    inline constexpr std::string_view kEntryMetadata = "{\"source\":\"AdvantageKit\"}";
    inline constexpr std::string_view kEntryMetadataUnits = "{\"source\":\"AdvantageKit\",\"unit\":\"$UNITSTR\"}";
    inline constexpr std::string_view kTimestampUnit = "nanoseconds";
} // namespace akit::WPILOGConstants
