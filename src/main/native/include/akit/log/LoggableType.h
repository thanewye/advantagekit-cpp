#pragma once

#include <array>
#include <string_view>

namespace akit {
    enum class LoggableType { kRaw = 0, kBoolean, kInteger, kFloat, kDouble, kString, kBooleanArray, kIntegerArray, kFloatArray, kDoubleArray, kStringArray };

    inline constexpr std::array<std::string_view, 11> kWPILOGTypes = {"raw",       "boolean", "int64",   "float",    "double",  "string",
                                                                      "boolean[]", "int64[]", "float[]", "double[]", "string[]"};

    inline constexpr std::array<std::string_view, 11> kNT4Types = {"raw",       "boolean", "int",     "float",    "double",  "string",
                                                                   "boolean[]", "int[]",   "float[]", "double[]", "string[]"};

    inline std::string_view GetWPILOGType(const LoggableType& type) {
        return kWPILOGTypes[static_cast<int>(type)];
    }

    inline std::string_view GetNT4Type(const LoggableType& type) {
        return kNT4Types[static_cast<int>(type)];
    }

    inline LoggableType FromWPILOGType(const std::string_view type) {
        for (int i = 0; i < static_cast<int>(kWPILOGTypes.size()); i++) {
            if (kWPILOGTypes[i] == type) return static_cast<LoggableType>(i);
        }
        if (type == "json") return LoggableType::kString;
        return LoggableType::kRaw;
    }

    inline LoggableType FromNT4Type(std::string_view type) {
        for (int i = 0; i < static_cast<int>(kNT4Types.size()); i++) {
            if (kNT4Types[i] == type) return static_cast<LoggableType>(i);
        }
        if (type == "json") return LoggableType::kString;
        return LoggableType::kRaw;
    }
} // namespace akit
