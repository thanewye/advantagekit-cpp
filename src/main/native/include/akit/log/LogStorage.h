#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "akit/log/LoggableType.h"

namespace akit {
    using LogValueVariant = std::variant<std::vector<uint8_t>,    // Raw
                                         bool,                    // Boolean
                                         int64_t,                 // Integer
                                         float,                   // Float
                                         double,                  // Double
                                         std::string,             // String
                                         std::vector<bool>,       // BooleanArray
                                         std::vector<int64_t>,    // IntegerArray
                                         std::vector<float>,      // FloatArray
                                         std::vector<double>,     // DoubleArray
                                         std::vector<std::string> // StringArray
                                         >;

    struct LogValue {
        LogValueVariant value;
        LoggableType type;
        std::string customTypeStr;
        std::optional<std::string> unitStr;

        explicit LogValue(std::vector<uint8_t> v, std::string typeStr = "")
            : value(std::move(v))
            , type(LoggableType::kRaw)
            , customTypeStr(std::move(typeStr)) {}

        explicit LogValue(bool v, std::string typeStr = "")
            : value(v)
            , type(LoggableType::kBoolean)
            , customTypeStr(std::move(typeStr)) {}

        explicit LogValue(int64_t v, std::string typeStr = "")
            : value(v)
            , type(LoggableType::kInteger)
            , customTypeStr(std::move(typeStr)) {}

        explicit LogValue(float v, std::string typeStr = "", std::optional<std::string> unit = std::nullopt)
            : value(v)
            , type(LoggableType::kFloat)
            , customTypeStr(std::move(typeStr))
            , unitStr(std::move(unit)) {}

        explicit LogValue(double v, std::string typeStr = "", std::optional<std::string> unit = std::nullopt)
            : value(v)
            , type(LoggableType::kDouble)
            , customTypeStr(std::move(typeStr))
            , unitStr(std::move(unit)) {}

        explicit LogValue(std::string v, std::string typeStr = "")
            : value(std::move(v))
            , type(LoggableType::kString)
            , customTypeStr(std::move(typeStr)) {}

        explicit LogValue(std::vector<bool> v, std::string typeStr = "")
            : value(std::move(v))
            , type(LoggableType::kBooleanArray)
            , customTypeStr(std::move(typeStr)) {}

        explicit LogValue(std::vector<int64_t> v, std::string typeStr = "")
            : value(std::move(v))
            , type(LoggableType::kIntegerArray)
            , customTypeStr(std::move(typeStr)) {}

        explicit LogValue(std::vector<float> v, std::string typeStr = "")
            : value(std::move(v))
            , type(LoggableType::kFloatArray)
            , customTypeStr(std::move(typeStr)) {}

        explicit LogValue(std::vector<double> v, std::string typeStr = "")
            : value(std::move(v))
            , type(LoggableType::kDoubleArray)
            , customTypeStr(std::move(typeStr)) {}

        explicit LogValue(std::vector<std::string> v, std::string typeStr = "")
            : value(std::move(v))
            , type(LoggableType::kStringArray)
            , customTypeStr(std::move(typeStr)) {}

        // Returns the WPILOG type string: customTypeStr if set, otherwise the primitive type name.
        // This matches Java's LogValue.getWPILOGType() — the custom type string overrides for
        // struct/protobuf values stored as Raw bytes with a semantic type name.
        [[nodiscard]] std::string GetWPILOGType() const {
            if (customTypeStr.empty()) return std::string(akit::GetWPILOGType(type));
            return customTypeStr;
        }

        [[nodiscard]] std::string GetNT4Type() const {
            if (customTypeStr.empty()) return std::string(akit::GetNT4Type(type));
            return customTypeStr;
        }

        bool operator==(const LogValue& other) const {
            return type == other.type && customTypeStr == other.customTypeStr && unitStr == other.unitStr && value == other.value;
        }

        bool operator!=(const LogValue& other) const { return !(*this == other); }
    };

    struct LogStorage {
        std::unordered_map<std::string, LogValue> values;
        // timestamp in Us
        int64_t timestamp = 0;
        void Clear() { values.clear(); }
        [[nodiscard]] bool Empty() const { return values.empty(); }
    };
} // namespace akit
