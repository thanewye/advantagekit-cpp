#pragma once

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
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

    /** Matches Java's Double.equals/Float.equals: NaN equals NaN, otherwise compares bit patterns. */
    template<typename Floating> bool FloatingBitsEqual(Floating lhs, Floating rhs) {
        using Bits = std::conditional_t<std::is_same_v<Floating, float>, uint32_t, uint64_t>;
        return (std::isnan(lhs) && std::isnan(rhs)) || std::bit_cast<Bits>(lhs) == std::bit_cast<Bits>(rhs);
    }

    /** Compares variant contents with Java's LogValue.equals semantics for floating-point values and arrays. */
    inline bool LogValueVariantsEqual(const LogValueVariant& lhs, const LogValueVariant& rhs) {
        if (lhs.index() != rhs.index()) return false;
        return std::visit(
            [&rhs]<typename T>(const T& lhsValue) {
                const T& rhsValue = std::get<T>(rhs);
                if constexpr (std::is_same_v<T, float> || std::is_same_v<T, double>) {
                    return FloatingBitsEqual(lhsValue, rhsValue);
                } else if constexpr (std::is_same_v<T, std::vector<float>> || std::is_same_v<T, std::vector<double>>) {
                    return std::ranges::equal(lhsValue, rhsValue, [](auto a, auto b) { return FloatingBitsEqual(a, b); });
                } else {
                    return lhsValue == rhsValue;
                }
            },
            lhs);
    }

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
            return type == other.type && customTypeStr == other.customTypeStr && unitStr == other.unitStr && LogValueVariantsEqual(value, other.value);
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
